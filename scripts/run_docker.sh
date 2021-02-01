#!/bin/bash

function docker_run() {
	docker run -it \
		--rm \
		--network=host \
		--device=/dev/stm32 \
		--device=/dev/lidar \
		-v $PWD:/root/SlamRobot \
		--cap-add=SYS_PTRACE \
		hwenwur/ros \
		bash -c 'cd /root/SlamRobot && bash'
}

function docker_exec() {
	local containerId="$1"
	docker exec -it "$containerId" bash -c 'cd /root/SlamRobot && /ros_entrypoint.sh bash'
}

function main() {
	local containerId=$(docker ps | grep hwenwur/ros | awk '{print $1}')
	if [ -n "$containerId" ]
	then
		docker_exec "$containerId"
	else
		docker_run
	fi
}

set -x
main
