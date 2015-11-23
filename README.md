# rtt-gazebo-clock-plugin
Gazebo system plugin to sync the simulation time between RTT and Gazebo. Based on https://github.com/ahoarau/rtt_gazebo/tree/master/rtt_gazebo_system

## Install

`mkdir build`

`cd build`

`cmake -Dgazebo_DIR=$insert-prefix-here/lib/cmake/gazebo -DOROCOS-RTT_DIR=$insert-prefix-here/lib/cmake/orocos-rtt -Dignition-math2_DIR=$insert-prefix-here/lib/cmake/ignition-math2 -DSDFormat_DIR=$insert-prefix-here/lib/cmake/sdformat ..`

`make -j 4`

## TODO

- Exclude Eigen folder

### Build Orocos Toolchain

1. `git clone --recursive https://github.com/orocos-toolchain/orocos_toolchain.git`
2. set `COR_ORO_PREFIX` to the install prefix
3. `export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$COR_ORO_PREFIX`
4. build package by package using: `cmake -DCMAKE_INSTALL_PREFIX=${COR_ORO_PREFIX} -DCMAKE_PREFIX_PATH=${COR_ORO_PREFIX} ..` except for `RTT` and `OCL`
5. build `RTT` with `cmake -DCMAKE_INSTALL_PREFIX=${COR_ORO_PREFIX} -DCMAKE_PREFIX_PATH=${COR_ORO_PREFIX} -DENABLE_CORBA=ON -DOROCOS_TARGET=gnulinux -DCORBA_IMPLEMENTATION=OMNIORB ..`
6. build `OCL` with `cmake -DCMAKE_INSTALL_PREFIX=${COR_ORO_PREFIX} -DCMAKE_PREFIX_PATH=${COR_ORO_PREFIX} -DBUILD_TASKBROWSER=ON ..`
