#!/bin/bash

if [ $# -ne 1 ]; then
  echo "Usage: $0 <effort> "
else

    rosservice call /gazebo/apply_joint_effort "{joint_name: 'escenarioprueba::door_joint',
    effort: $1, 
    start_time: {secs: 0, nsecs: 0},
    duration: {secs: 3, nsecs: 0}}"

fi