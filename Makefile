.PHONY: deploy build-docker

deploy:
	rsync -rt --progress --delete --exclude="devel" --exclude="build" --exclude="bot_simulation" ./. tx2:~/.hb/SlamRobot2

build-docker:
	docker build -t hwenwur/ros -f ./Dockerfile ./docker-context

