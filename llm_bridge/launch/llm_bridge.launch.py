"""Launch llama-server and llm_bridge_node together."""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    model_path = os.environ.get('LLM_MODEL_PATH',
                                os.path.expanduser('~/llama.cpp/models/qwen2.5-7b-q4_k_m.gguf'))
    server_bin = os.environ.get('LLM_SERVER_BIN',
                                os.path.expanduser('~/llama.cpp/build/bin/llama-server'))

    llama_server = ExecuteProcess(
        cmd=[server_bin, '-m', model_path,
             '--host', '0.0.0.0', '--port', '8080',
             '-c', '4096'],
        name='llama_server',
        output='screen',
    )

    llm_bridge = Node(
        package='llm_bridge',
        executable='llm_bridge_node',
        name='llm_bridge_node',
        output='screen',
        parameters=[{
            'server_url': 'http://localhost:8080',
        }],
    )

    return LaunchDescription([
        LogInfo(msg=f'Starting llama-server with model: {model_path}'),
        llama_server,
        RegisterEventHandler(
            OnProcessStart(
                target_action=llama_server,
                on_start=[
                    LogInfo(msg='llama-server started, waiting 3s before starting bridge...'),
                ],
            ),
        ),
        llm_bridge,
    ])
