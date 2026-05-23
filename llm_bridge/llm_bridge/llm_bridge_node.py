#!/usr/bin/env python3
"""ROS2 node that bridges /llm/query and /llm/response to llama-server's OpenAI API."""

import json
import urllib.request
import urllib.error
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LLMBridgeNode(Node):
    def __init__(self):
        super().__init__('llm_bridge_node')

        self.declare_parameter('server_url', 'http://localhost:8080')
        self.declare_parameter('model', 'gpt-4')
        self.declare_parameter('max_tokens', 1024)
        self.declare_parameter('temperature', 0.7)
        self.declare_parameter('timeout', 120.0)
        self.declare_parameter('system_prompt',
                               '你是一个采摘机器人的语音助手，部署在 Jetson Orin 上。'
                               '你可以回答关于机器人状态、任务和操作的问题。用中文回答。')

        self.server_url = self.get_parameter('server_url').value
        self.model = self.get_parameter('model').value
        self.max_tokens = self.get_parameter('max_tokens').value
        self.temperature = self.get_parameter('temperature').value
        self.timeout = self.get_parameter('timeout').value
        self.system_prompt = self.get_parameter('system_prompt').value

        self.chat_history = []
        self._busy = False
        self._last_query = ''
        self._lock = threading.Lock()

        self.sub = self.create_subscription(
            String, '/llm/query', self.query_callback, 10)
        self.pub = self.create_publisher(String, '/llm/response', 10)

        self.get_logger().info(f'LLM Bridge ready, server: {self.server_url}')

    def query_callback(self, msg):
        user_text = msg.data.strip()
        if not user_text:
            return

        if user_text == '/clear':
            with self._lock:
                self.chat_history.clear()
                self._last_query = ''
            self._publish('对话历史已清除')
            return

        with self._lock:
            if self._busy:
                self.get_logger().warn('Busy, skipping duplicate query')
                return
            if user_text == self._last_query:
                self.get_logger().warn('Duplicate query skipped')
                return
            self._last_query = user_text
            self._busy = True

        self.get_logger().info(f'Query: {user_text}')
        self.chat_history.append({'role': 'user', 'content': user_text})

        thread = threading.Thread(target=self._do_chat, daemon=True)
        thread.start()

    def _do_chat(self):
        try:
            reply = self._call_chat_api()
            self._publish(reply)
        except Exception as e:
            self.get_logger().error(f'LLM call failed: {e}')
            self._publish(f'错误：大模型调用失败 - {e}')
        finally:
            with self._lock:
                self._busy = False

    def _call_chat_api(self):
        messages = [{'role': 'system', 'content': self.system_prompt}]
        with self._lock:
            messages.extend(self.chat_history[-20:])

        payload = {
            'model': self.model,
            'messages': messages,
            'max_tokens': self.max_tokens,
            'temperature': self.temperature,
        }

        url = f'{self.server_url}/v1/chat/completions'
        data = json.dumps(payload).encode('utf-8')

        req = urllib.request.Request(url, data=data, headers={
            'Content-Type': 'application/json',
        })

        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            result = json.loads(resp.read().decode('utf-8'))

        reply = result['choices'][0]['message']['content']
        with self._lock:
            self.chat_history.append({'role': 'assistant', 'content': reply})
        self.get_logger().info(f'Reply: {reply[:100]}...')
        return reply

    def _publish(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = LLMBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
