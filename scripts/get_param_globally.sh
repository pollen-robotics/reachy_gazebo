#!/bin/bash


get_param(){

    res=`ros2 param get $1 $2`
    echo $1 $res
}

for node in $(ros2 node list);
do
    # get_param $node $1 &
    echo $node
    ros2 param get $node $1
done
