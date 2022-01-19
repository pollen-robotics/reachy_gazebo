#!/bin/bash

for node in $(ros2 node list);
do
    echo "$node"
    ros2 param set "$node" $1 $2
done
