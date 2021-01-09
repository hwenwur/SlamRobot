#!/bin/bash
# 串口嗅探工具，输出进程对串口文件读写的数据
# hwenwur 2021/01/09

if [ $# -ne 1 ]; then
    echo "usage: $0 /dev/ttyUSBx"
    exit 1
fi

readonly serial_port="$1"
readonly pid=$(lsof "$serial_port" 2>/dev/null | sed '1d' | awk '{print $2}')

if [ -z "$pid" ]; then
    echo 'No process write to ' "$serial_port"
    exit 0
fi

echo "Attach to process($pid)..."
strace -x -p "$pid" -P"$serial_port" -e'write,read'

