.PHONY: build deploy build-docker clean

build:
	cd catkin_ws && catkin_make

deploy:
	rsync -rt --progress --delete --exclude="devel" --exclude="build" --exclude="bot_simulation" ./. tx2:~/.hb/SlamRobot2

build-docker:
	docker build -t hwenwur/ros -f ./Dockerfile ./docker-context

clean:
	rm -rf catkin_ws/devel catkin_ws/build

