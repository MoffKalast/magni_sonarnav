# Sonar "Nav" packages

This repo contains two packages:
- magni_aislemode that will attempt to navigate along a corridor using sonars
- magni_randomwalk that tries to randomly explore a room without collisions

### Installation instructions:

    cd ~/catkin_ws/src
    git clone https://github.com/MoffKalast/magni_sonarnav.git
    cd ..
    catkin_make

### Launch instructions

For random exploration:

    roslaunch magni_randomwalk randomwalk.launch

For corridor navigation:

    roslaunch magni_randomwalk aisle.launch
