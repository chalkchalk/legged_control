#!/bin/bash

# 定义 rosservice call 命令
rosservice_command='rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/legged_controller']                   
stop_controllers: ['']
strictness: 0
start_asap: false
timeout: 0.0" '

# 执行 rosservice call 命令
eval "$rosservice_command"