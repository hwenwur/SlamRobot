.PHONY: all

all: read write

%: %.cpp
	g++ -std=c++11 -I../../catkin_ws/src/robot_driver/include $^ -o $@

clean:
	rm -rf ./read ./write

