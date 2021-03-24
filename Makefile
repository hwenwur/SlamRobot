.PHONY: deploy

deploy:
	rsync -rt --progress --delete --exclude="devel" --exclude="build" --exclude="bot_simulation" ./. tx2:~/.hb/SlamRobot2
