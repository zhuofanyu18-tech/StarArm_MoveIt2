## 测试 llama-server 是否正常
    cd ~/llama.cpp
    ./build/bin/llama-server \
    -m models/qwen2.5-7b-instruct-q4_k_m.gguf \
    --host 0.0.0.0 --port 8080 -c 4096

    另开终端测试
    curl http://localhost:8080/v1/chat/completions \
    -H "Content-Type: application/json" \
    -d '{"model":"gpt-4","messages":[{"role":"user","content":"你好，你是谁？"}]}'


## 终端 1 — 启动 llama-server：

cd ~/llama.cpp
./build/bin/llama-server -m models/qwen2.5-7b-instruct-q4_k_m.gguf --host 0.0.0.0 --port 8080 -c 4096

## 终端 2 — 启动 ROS2 节点：

source /opt/ros/humble/setup.bash
source ~/starbot_arm_ws/install/setup.bash
ros2 run llm_bridge llm_bridge_node

## 终端 3 — 发消息测试：
source /opt/ros/humble/setup.bash

# 发送问题
ros2 topic pub /llm/query std_msgs/msg/String "data: '你好，告诉我采摘小车的机械臂有几个关节'"

# 监听回答
ros2 topic echo /llm/response

# 清除对话历史
ros2 topic pub /llm/query std_msgs/msg/String "data: '/clear'"