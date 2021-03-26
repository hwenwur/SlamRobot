#!/bin/bash

if [ $# -ne 1 ]; then
  echo 'You must specify filename'
  exit 1 
fi

rosservice call /finish_trajectory 0

rosservice call /write_state "{filename: '${HOME}/srm/data/$1.pbstream'}"

