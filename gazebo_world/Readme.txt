This project has 3 directories:
1. model 
2. script
3. world

"model" directory has two models:
1. robot - This model has 4-wheel robot (It has 4 wheels and camera)
2. simplefloor - This model has single floot structure 
3. iris_with_standoffs - This model is cloned from gazebo online.
   Cloned GIT REPO: https://github.com/osrf/gazebo_models.git 

"script" directory has plugin C++ code

"world" directory has Gazebo world

To build plugin:
1. mkdir "build"
2. cd ./build
3. cmake ../
4. make
5. export GAZEBO_PLUGIN_PATH=`pwd`

To run gazebo:
1. cd  to TOP directory(gazebo_world)
2. run "gazebo world/RobotWorld --verbose"


