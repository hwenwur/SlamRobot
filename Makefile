.PHONY: deploy

deploy:
	rsync -r --progress --delete --exclude="devel" --exclude="build" ./. tx2:~/.hb/SlamRobot2
