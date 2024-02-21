#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch publisher
roslaunch my_package pubsub.launch

# wait for app to end
dt-launchfile-join