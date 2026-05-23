#!/bin/bash
# start_llm.sh - 一键启动 llama-server + ROS2 llm_bridge
#
# 用法:
#   ./start_llm.sh                                      # 使用默认路径
#   LLM_MODEL_PATH=/path/to/model.gguf ./start_llm.sh    # 指定模型
#
# 环境变量:
#   LLM_MODEL_PATH  - GGUF 模型路径 (默认: ~/llama.cpp/models/qwen2.5-7b-q4_k_m.gguf)
#   LLM_SERVER_BIN  - llama-server 可执行文件 (默认: ~/llama.cpp/build/bin/llama-server)
#   LLM_PORT         - HTTP 端口 (默认: 8080)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODEL_PATH="${LLM_MODEL_PATH:-$HOME/llama.cpp/models/qwen2.5-7b-q4_k_m.gguf}"
SERVER_BIN="${LLM_SERVER_BIN:-$HOME/llama.cpp/build/bin/llama-server}"
PORT="${LLM_PORT:-8080}"

echo "============================================"
echo "  ROS2 + llama.cpp LLM Bridge Launcher"
echo "============================================"
echo "  Model : $MODEL_PATH"
echo "  Server: $SERVER_BIN"
echo "  Port  : $PORT"
echo "============================================"

# 检查必要文件
if [ ! -f "$SERVER_BIN" ]; then
    echo "ERROR: llama-server not found at $SERVER_BIN"
    echo "Build it first: cd ~/llama.cpp && cmake -B build && cmake --build build -j"
    exit 1
fi

if [ ! -f "$MODEL_PATH" ]; then
    echo "ERROR: Model not found at $MODEL_PATH"
    echo "Download a GGUF model first, e.g.:"
    echo "  cd ~/llama.cpp/models"
    echo "  wget https://huggingface.co/.../qwen2.5-7b-instruct-q4_k_m.gguf"
    exit 1
fi

# 清理旧进程
echo "Stopping old llama-server if running..."
pkill -f "llama-server" 2>/dev/null || true
sleep 1

# 启动 llama-server
echo "Starting llama-server..."
"$SERVER_BIN" -m "$MODEL_PATH" --host 0.0.0.0 --port "$PORT" -c 4096 &
SERVER_PID=$!
echo "llama-server PID: $SERVER_PID"

# 等待服务器就绪
echo "Waiting for server to be ready..."
for i in $(seq 1 30); do
    if curl -s "http://localhost:$PORT/health" >/dev/null 2>&1; then
        echo "Server is ready!"
        break
    fi
    if [ $i -eq 30 ]; then
        echo "ERROR: Server did not become ready in 30 seconds"
        kill $SERVER_PID 2>/dev/null
        exit 1
    fi
    sleep 1
done

# Source ROS2 环境
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f "$HOME/starbot_arm_ws/install/setup.bash" ]; then
    source "$HOME/starbot_arm_ws/install/setup.bash"
fi

echo "Starting llm_bridge_node..."
echo ""
echo "  Now you can query the LLM via ROS2:"
echo "    ros2 topic pub /llm/query std_msgs/msg/String \"data: '你好，告诉我机器人的状态'\""
echo "    ros2 topic echo /llm/response"
echo ""
echo "  Say /clear to reset the chat history"
echo "  Press Ctrl+C to stop"
echo ""

# 启动 llm_bridge_node (前台运行)
ros2 run llm_bridge llm_bridge_node

# 清理
echo "Shutting down..."
kill $SERVER_PID 2>/dev/null || true
