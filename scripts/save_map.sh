#!/bin/bash
# http://wiki.ros.org/map_server#map_server-1

mkdir -p map

rosrun map_server map_saver -f ./map/map

# delete 'map/' from map.yaml 'image:'
sed 's/map\///g' -i map/map.yaml

