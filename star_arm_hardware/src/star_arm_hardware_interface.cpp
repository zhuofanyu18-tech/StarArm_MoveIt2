#include "star_arm_hardware/star_arm_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <cmath>
#include <cstring>

namespace star_arm_hardware
{

// ==========================================================================
//  舵机校准数据
//  回转关节公式: servo_pos = scale * rad + offset, scale=±652 (4096/2π)
//  直线关节公式: servo_pos = scale * m + offset (end_joint1 夹爪)
//
//  校准来源（用户提供）:
//  joint1: min{-1.6,3120} center{0,2048} max{1.6,1010} → scale=-652, offset=2048
//  joint2: min{-1.75,3211} center{0,2048} max{1.75,929} → scale=-652, offset=2048
//  joint3: min{0,3080} center{0,3080} max{3.0,减少} → scale=-652, offset=3080
//  joint4: min{-1.75,907} center{0,2048} max{1.75,3189} → scale=+652, offset=2048
//  end_joint1: min{0,2048} max{0.02,2800} → scale=+37600, offset=2048 (直线夹爪, m)
// ==========================================================================
struct JointCalib {
    double scale;   // 含方向符号的 scale (rad→servo)
    double offset;  // rad=0 时的 servo 值
};

static const JointCalib kJointCalibTable[] = {
    // index 0: joint1
    {-652.0, 2048.0},
    // index 1: joint2
    {-652.0, 2048.0},
    // index 2: joint3
    {-652.0, 3080.0},
    // index 3: joint4
    {+652.0, 2048.0},
    // index 4: joint5 (暂不控制，占位)
    {+652.0, 2048.0},
    // index 5: end_joint1 (直线夹爪, servo/m)
    {+37600.0, 2048.0},  // 闭合=2048, 张开=2800, scale=(2800-2048)/0.02
};

// ╔══════════════════════════════════════════════════════════════════╗
// ║                     生命周期:on_init                              ║
// ╚══════════════════════════════════════════════════════════════════╝
hardware_interface::CallbackReturn StarArmHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    size_t n = info_.joints.size();
    hw_states_.resize(n, 0.0);
    hw_commands_.resize(n, 0.0);

    joint_servo_ids_.resize(n, -1);
    joint_offsets_.resize(n, 2048.0);
    joint_scales_.resize(n, 652.0);
    joint_directions_.resize(n, 1);

    // 根据关节名称设置: 舵机ID、offset、scale符号
    for (size_t i = 0; i < n; i++) {
        const std::string& name = info_.joints[i].name;
        if (name == "joint1") {
            joint_servo_ids_[i] = 1;
            joint_scales_[i]   = -652.0;
            joint_offsets_[i]  = 2048.0;
        } else if (name == "joint2") {
            joint_servo_ids_[i] = 2;
            joint_scales_[i]   = -652.0;
            joint_offsets_[i]  = 2048.0;
        } else if (name == "joint3") {
            joint_servo_ids_[i] = 3;
            joint_scales_[i]   = -652.0;
            joint_offsets_[i]  = 3080.0;
        } else if (name == "joint4") {
            joint_servo_ids_[i] = 4;
            joint_scales_[i]   = +652.0;
            joint_offsets_[i]  = 2048.0;
        } else if (name == "end_joint1") {
            joint_servo_ids_[i] = 6;
            joint_scales_[i]   = +37600.0;  // (2800-2048)/0.02
            joint_offsets_[i]  = 2048.0;    // 闭合(joint_pos=0)时 servo 值
        }
        // joint5 → servo_id=-1, 不控制
    }

    // 从 URDF 读取可选参数（可覆盖校准表）
    auto it = info_.hardware_parameters.find("default_speed");
    if (it != info_.hardware_parameters.end()) {
        default_speed_ = std::stoi(it->second);
    }
    it = info_.hardware_parameters.find("default_acc");
    if (it != info_.hardware_parameters.end()) {
        default_acc_ = std::stoi(it->second);
    }
    it = info_.hardware_parameters.find("read_throttle");
    if (it != info_.hardware_parameters.end()) {
        read_throttle_ = std::stoi(it->second);
    }
    it = info_.hardware_parameters.find("write_throttle");
    if (it != info_.hardware_parameters.end()) {
        write_throttle_ = std::stoi(it->second);
    }

    joint_speeds_.resize(n, default_speed_);
    for (size_t i = 0; i < n; i++) {
        std::string key = info_.joints[i].name + "_speed";
        auto jt = info_.hardware_parameters.find(key);
        if (jt != info_.hardware_parameters.end()) {
            joint_speeds_[i] = std::stoi(jt->second);
            RCLCPP_INFO(rclcpp::get_logger("StarArmHardware"),
                "关节 %s 速度设为 %d", info_.joints[i].name.c_str(), joint_speeds_[i]);
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("StarArmHardware"),
        "on_init: %zu joints, speed=%d, acc=%d, r_throttle=%d, w_throttle=%d | "
        "j1(s=%.0f/o=%.0f) j2(s=%.0f/o=%.0f) j3(s=%.0f/o=%.0f) "
        "j4(s=%.0f/o=%.0f) end(s=%.0f/o=%.0f)",
        n, default_speed_, default_acc_, read_throttle_, write_throttle_,
        joint_scales_[0], joint_offsets_[0],
        joint_scales_[1], joint_offsets_[1],
        joint_scales_[2], joint_offsets_[2],
        joint_scales_[3], joint_offsets_[3],
        joint_scales_[5], joint_offsets_[5]);

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ╔══════════════════════════════════════════════════════════════════╗
// ║                  生命周期：on_configure                         ║
// ╚══════════════════════════════════════════════════════════════════╝
hardware_interface::CallbackReturn StarArmHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    std::string port = info_.hardware_parameters["serial_port"];

    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("StarArmHardware"),
            "无法打开串口 %s", port.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    struct termios options;
    tcgetattr(serial_fd_, &options);
    cfmakeraw(&options);  // 原始二进制模式，禁用软件流控 XON/XOFF
    cfsetispeed(&options, B1000000);
    cfsetospeed(&options, B1000000);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    tcsetattr(serial_fd_, TCSANOW, &options);

    // cfmakeraw 已设置非阻塞行为，VMIN=0/VTIME=0 使 read() 立即返回

    RCLCPP_INFO(rclcpp::get_logger("StarArmHardware"),
        "串口 %s 已打开 (1Mbps, 8N1)", port.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ╔══════════════════════════════════════════════════════════════════╗
// ║                  导出接口                                        ║
// ╚══════════════════════════════════════════════════════════════════╝
std::vector<hardware_interface::StateInterface>
StarArmHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_states_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
StarArmHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_commands_[i]));
    }
    return command_interfaces;
}

// ╔══════════════════════════════════════════════════════════════════╗
// ║                  生命周期：on_activate / on_deactivate           ║
// ╚══════════════════════════════════════════════════════════════════╝
hardware_interface::CallbackReturn StarArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    for (size_t i = 0; i < hw_states_.size(); i++) {
        hw_commands_[i] = hw_states_[i];
    }
    write_counter_ = 0;
    read_servo_idx_ = 0;
    RCLCPP_INFO(rclcpp::get_logger("StarArmHardware"), "硬件接口已激活");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StarArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (serial_fd_ != -1) {
        close(serial_fd_);
        serial_fd_ = -1;
        RCLCPP_INFO(rclcpp::get_logger("StarArmHardware"), "串口已关闭");
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ╔══════════════════════════════════════════════════════════════════╗
// ║       read() — 轮询读取舵机实际位置 (每周期 2 个舵机)              ║
// ║  每舵机 poll 最多 2ms, 单周期总耗时 ≤4ms, 不阻塞控制循环          ║
// ║  5 个舵机全部刷新 ~3 周期 = 60ms                                 ║
// ╚══════════════════════════════════════════════════════════════════╝
hardware_interface::return_type StarArmHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // 每周期读 2 个舵机
    size_t n = hw_states_.size();
    for (int count = 0; count < 2; count++) {
        size_t idx = read_servo_idx_ % n;
        read_servo_idx_++;

        int servo_id = joint_servo_ids_[idx];
        if (servo_id < 0) continue;  // 被动关节 joint5

        int16_t pos = 0;
        if (read_servo_position(static_cast<uint8_t>(servo_id), pos)) {
            hw_states_[idx] = servo_to_rad(pos, idx);
        }
        // 读取失败: 保持上一次的值
    }

    return hardware_interface::return_type::OK;
}

// ╔══════════════════════════════════════════════════════════════════╗
// ║                  write() — 发送命令到舵机                        ║
// ╚══════════════════════════════════════════════════════════════════╝
hardware_interface::return_type StarArmHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // 写节流: 降低更新频率让舵机内部平滑完成运动
    write_counter_++;
    if (write_throttle_ > 1 && (write_counter_ % write_throttle_ != 0)) {
        return hardware_interface::return_type::OK;
    }

    if (!send_sync_write(hw_commands_)) {
        RCLCPP_ERROR(rclcpp::get_logger("StarArmHardware"),
            "SyncWrite 发送失败");
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

// ╔══════════════════════════════════════════════════════════════════╗
// ║              send_sync_write() — Feetech SyncWrite 协议         ║
// ╚══════════════════════════════════════════════════════════════════╝
bool StarArmHardwareInterface::send_sync_write(
    const std::vector<double>& commands)
{
    if (serial_fd_ == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("StarArmHardware"),
            "串口未打开，无法发送");
        return false;
    }

    int servo_count = 0;
    for (size_t i = 0; i < commands.size(); i++) {
        if (joint_servo_ids_[i] >= 0) servo_count++;
    }
    if (servo_count == 0) return true;

    uint8_t txpacket[128];
    int param_length = servo_count * 8;

    txpacket[0] = 0xFF;
    txpacket[1] = 0xFF;
    txpacket[2] = 0xFE;
    txpacket[3] = param_length + 4;
    txpacket[4] = 0x83; // INST_SYNC_WRITE
    txpacket[5] = 41;   // 起始地址: 加速度寄存器
    txpacket[6] = 7;    // 每舵机数据长度(不含ID)

    int idx = 7;
    int checksum = txpacket[2] + txpacket[3] + txpacket[4]
                 + txpacket[5] + txpacket[6];

    for (int i = commands.size() - 1; i >= 0; i--) {
        int servo_id = joint_servo_ids_[i];
        if (servo_id < 0) continue;

        int pos = static_cast<int>(rad_to_servo(commands[i], i));
        pos = std::max(0, std::min(pos, 4095));

        int speed = joint_speeds_[i];
        int acc = default_acc_;

        txpacket[idx++] = static_cast<uint8_t>(servo_id);
        txpacket[idx++] = acc & 0xFF;
        txpacket[idx++] = pos & 0xFF;
        txpacket[idx++] = (pos >> 8) & 0xFF;
        txpacket[idx++] = 0;
        txpacket[idx++] = 0;
        txpacket[idx++] = speed & 0xFF;
        txpacket[idx++] = (speed >> 8) & 0xFF;

        checksum += (servo_id + (acc & 0xFF) + (pos & 0xFF)
                     + ((pos >> 8) & 0xFF) + 0 + 0
                     + (speed & 0xFF) + ((speed >> 8) & 0xFF));
    }

    txpacket[idx] = ~checksum & 0xFF;

    ssize_t written = ::write(serial_fd_, txpacket, idx + 1);
    if (written != idx + 1) {
        RCLCPP_ERROR(rclcpp::get_logger("StarArmHardware"),
            "串口写入不完整: 期望 %d 字节，实际 %ld 字节", idx + 1, written);
        return false;
    }
    return true;
}

// ╔══════════════════════════════════════════════════════════════════╗
// ║          read_servo_position() — Feetech READ 指令 + poll       ║
// ║     先 tcflush 清空缓冲区，发一次命令，poll 累加读取 (处理分片)    ║
// ╚══════════════════════════════════════════════════════════════════╝
bool StarArmHardwareInterface::read_servo_position(
    uint8_t id, int16_t &position)
{
    if (serial_fd_ == -1) return false;

    uint8_t tx[8];
    tx[0] = 0xFF;
    tx[1] = 0xFF;
    tx[2] = id;
    tx[3] = 4;
    tx[4] = 0x02;  // INST_READ
    tx[5] = 56;    // PRESENT_POSITION (0x38)
    tx[6] = 2;     // 读 2 字节
    tx[7] = ~(id + 4 + 0x02 + 56 + 2) & 0xFF;

    tcflush(serial_fd_, TCIFLUSH);

    ssize_t written = ::write(serial_fd_, tx, 8);
    if (written != 8) {
        return false;
    }

    // poll 累加读取，处理 USB 串口芯片数据分片
    uint8_t rx[10] = {0};
    int received_len = 0;
    const int expected_len = 7;

    for (int retry = 0; retry < 3; retry++) {
        struct pollfd pfd;
        pfd.fd = serial_fd_;
        pfd.events = POLLIN;

        int ready = poll(&pfd, 1, 1);  // 1ms 超时
        if (ready < 0) {
            return false;
        }
        if (ready == 0) {
            continue;  // 无数据，等下一个 1ms
        }

        if (!(pfd.revents & POLLIN)) {
            continue;
        }

        // 只读剩余字节，拼接到 rx 缓冲区末尾
        ssize_t n = ::read(serial_fd_, rx + received_len,
                           expected_len - received_len);
        if (n > 0) {
            received_len += n;
            if (received_len >= expected_len) {
                break;
            }
        }
    }

    // 读满 7 字节才解析
    if (received_len >= expected_len &&
        rx[0] == 0xFF && rx[1] == 0xFF &&
        rx[2] == id && rx[4] == 0x00) {
        position = rx[5] | (rx[6] << 8);
        return true;
    }

    return false;
}

// ╔══════════════════════════════════════════════════════════════════╗
// ║          坐标转换：弧度 ↔ 舵机单位（每关节独立校准）              ║
// ║  servo = scale * rad + offset                                   ║
// ║  scale 含方向符号: +652 表示 rad↑→servo↑, -652 表示 rad↑→servo↓    ║
// ╚══════════════════════════════════════════════════════════════════╝
double StarArmHardwareInterface::rad_to_servo(double rad, size_t joint_idx) const
{
    return joint_scales_[joint_idx] * rad + joint_offsets_[joint_idx];
}

double StarArmHardwareInterface::servo_to_rad(int16_t servo_pos, size_t joint_idx) const
{
    uint16_t raw = static_cast<uint16_t>(servo_pos);
    int pos;
    if (raw & (1 << 15)) {
        pos = -static_cast<int>(raw & 0x7FFF);
    } else {
        pos = static_cast<int>(raw);
    }
    return (static_cast<double>(pos) - joint_offsets_[joint_idx])
           / joint_scales_[joint_idx];
}

}  // namespace star_arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    star_arm_hardware::StarArmHardwareInterface,
    hardware_interface::SystemInterface)
