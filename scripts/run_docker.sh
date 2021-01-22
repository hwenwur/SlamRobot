#!/bin/bash

docker run -it \
	--rm \
	--network=host \
	--device=/dev/stm32 \
	--device=/dev/lidar \
	-v $PWD:/root/SlamRobot \
	--cap-add=SYS_PTRACE \
	hwenwur/ros

