# rtt-gazebo-clock-plugin
Gazebo system plugin to sync the simulation time between RTT and Gazebo. Based on https://github.com/ahoarau/rtt_gazebo/tree/master/rtt_gazebo_system

## Install

`mkdir build`

`cd build`

`cmake -Dgazebo_DIR=$insert-prefix-here/lib/cmake/gazebo -DOROCOS-RTT_DIR=$insert-prefix-here/lib/cmake/orocos-rtt -Dignition-math2_DIR=$insert-prefix-here/lib/cmake/ignition-math2 -DSDFormat_DIR=$insert-prefix-here/lib/cmake/sdformat ..`

`make -j 4`
