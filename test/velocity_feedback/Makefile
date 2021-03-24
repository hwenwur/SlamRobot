.PHONY: cp rm 

main: main.o VelocityReader.o
	g++ main.o VelocityReader.o -o main -pthread

%.o: %.cpp
	g++ -std=c++11 -g -c $^ -o $@

cp: 
	cp ../../catkin_ws/src/robot_driver/include/*.h ./
	cp ../../catkin_ws/src/robot_driver/src/VelocityReader.cpp ./

clean:
	rm -rf *.o ./main
