#!bin/bash

# launch publisher and subscriber node with cleanup handler

cleanup(){
    echo "Restarting ROS 2 demon to cleanup before shutting down all the processes..."
    ros2 daemon stop
    sleep 1
    ros2 daemon start
    echo "Terminating all ROS 2-related processes..."
    kill 0
    exit
}

trap 'cleanup' SIGINT

#launch the publisher node
ros2 run ros2_fundamentals_examples py_minimal_publisher.py &

sleep 2

#launch the subscriber node
ros2 run ros2_fundamentals_examples py_minimal_subscriber.py
