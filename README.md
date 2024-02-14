# F1by10

## Create a workspace

    mkdir f1tenth_ws
    cd f1tenth_ws
    mkdir src
    cd src

## Clone the f1tenth_simulator repository for the simulator

    git clone https://github.com/f1tenth/f1tenth_simulator.git

* This is the repository build by someone else for simulating a car in the rviz environment.

* Read the instructions from the repository's README to download all the required packages.

## Then clone this repository

    git clone https://github.com/manjotsuri-20/F1by10.git

## Build the package by running

    cd ~/f1tenth_ws
    catkin_make
    source devel/setup.bash

## Run

    roslaunch f1_pkg f1.launch

### Press 'K' to turn on keyboard controls with WASD controls.

### Press 'B' to turn on the safety node.

### Press 'N' to turn on automatic left wall following.
