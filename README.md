# SAE
Sistema de amortiguación electromagnético (SAE)

## important commands
### ros2 environment
- init environment (everytime you first connected to your rasp after turning it on):

source /opt/ros/jazzy/setup.bash

cd ~/sae_wsv2

source install/setup.bash

source /opt/ros/jazzy/setup.bash

source ~/sae_wsv2/install/setup.bash

source ~/.bashrc

- list avalaible nodes in each package:

ros2 pkg executables main_package

ros2 pkg executables service_package

- run a node:

ros2 run main_package distanceSensor_node --ros-args -p adc_channel:=2


- launch a launch file:

ros2 launch main_package sensors.launch.py

- compile workspace:

cd ~/sae_wsv2

colcon build --symlink-install

source install/setup.bash

- see active nodes:

ros2 node list

- see node conections:

ros2 node info <node_name>

- see topics:

ros2 topic list

- see graph nodes (UBUNTU server required on computer, install UBUNTU on WSL to do so):

1) install ro2jazzy on your computer
2) connect to the same wifi
3) config ROS_DOMAIN_ID and ROS discovery
4) execute "ros2 run rqt_graph rqt_graph"

### GIT

- see what files have changed: 

git status

- pull changes (important to do so before push)

cd ~/sae_wsv2
git pull

- add changes (inside ros2 project) + commit + push:

git pull --rebase origin main

git add .

git commit -m "Description"

git push

### DEBUG
- listen topics

ros2 topic echo <nombre_del_topico>

- detect i2c connections

sudo i2cdetect -y 1
