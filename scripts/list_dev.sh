#!/bin/bash

# hwenwur 2020/12/31
# 输出已插入且在 dev_list 的设备和 /dev/ttyUSB* 的关系

dev_list=$(cat <<-'EOF'
10c4:ea60 Delta2A 雷达
1a86:7523 ESP8266
EOF
)

if ! ls -1 /dev | grep ttyUSB >/dev/null; then
    echo 'not plug any devices.' >&2
    exit 1
fi

dev_files=$(ls -1 /dev/tty* | grep --color=never -E 'USB|ACM')

for dev_name in $dev_files
do
    info=$(udevadm info --query property --name "$dev_name")
    vid=$(echo "$info" | grep 'ID_VENDOR_ID' | awk -F= '{ print $2 }')
    pid=$(echo "$info" |  grep 'ID_MODEL_ID' | awk -F= '{ print $2 }')
    dev=$(echo "$dev_list" | grep "$vid:$pid" | sed -E 's/^\w{4}:\w{4} //g')
    echo "$dev_name $vid:$pid $dev"
done

