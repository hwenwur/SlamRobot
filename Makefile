.PHONY: deploy

deploy:
	rsync -r --progress --exclude="devel" --exclude="build" ./. tx2:~/.hb/SlamRobot2
