FROM ros:melodic

RUN sed -i 's/packages.ros.org/mirrors.sjtug.sjtu.edu.cn/g' /etc/apt/sources.list.d/ros1-latest.list && \
  sed -E -i 's/(archive|security).ubuntu.com/mirrors.sjtug.sjtu.edu.cn/g' /etc/apt/sources.list && \
  sed -E -i 's/ports.ubuntu.com/mirror.sjtu.edu.cn/g' /etc/apt/sources.list && \
  apt-get update && \
  apt-get install -y ros-melodic-cartographer-ros ros-melodic-navigation ros-melodic-teleop-twist-keyboard && \
  apt-get install -y strace && \
  rm -rf /var/lib/apt/lists/*

CMD ["bash"]

