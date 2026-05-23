#ifndef STAR_ARM_HARDWARE_INTERFACE_HPP_
#define STAR_ARM_HARDWARE_INTERFACE_HPP_

#include <vector>
#include <string>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace star_arm_hardware
{

class StarArmHardwareInterface : public hardware_interface::SystemInterface
{

public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    int serial_fd_ = -1;

    // ========== 协议相关 ==========
    bool send_sync_write(const std::vector<double>& commands);
    bool read_servo_position(uint8_t id, int16_t &position);

    // ========== 坐标转换（每个关节独立校准） ==========
    double rad_to_servo(double rad, size_t joint_idx) const;
    double servo_to_rad(int16_t servo_pos, size_t joint_idx) const;

    // ========== 数据存储 ==========
    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;
    std::vector<int> joint_servo_ids_;  // 每个关节 index → 舵机 ID（-1=被动）

    // ========== 每关节校准参数 ==========
    // servo = direction * scale * rad + offset
    std::vector<double> joint_offsets_;     // servo value at rad=0
    std::vector<double> joint_scales_;      // servo units per radian (absolute)
    std::vector<int>    joint_directions_;  // 1 = 正向, -1 = 反向

    int default_speed_ = 400;
    int default_acc_ = 40;
    std::vector<int> joint_speeds_;

    int read_throttle_ = 1;    // 保留参数，当前每周期全读
    int write_throttle_ = 1;   // 每 N 个周期写一次
    int write_counter_ = 0;
    int read_servo_idx_ = 0;   // 轮询读取的当前舵机索引
};

}  // namespace star_arm_hardware

#endif
