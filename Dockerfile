FROM ros:melodic

RUN sed -i 's/packages.ros.org/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list.d/ros1-latest.list && \
  sed -E -i 's/(archive|security).ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
  apt-get update && \
  apt-get install -y ros-melodic-cartographer-ros ros-melodic-navigation ros-melodic-teleop-twist-keyboard && \
  rm -rf /var/lib/apt/lists/*

CMD ["bash"]
