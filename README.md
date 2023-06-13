# ros2_hover_diffbot 

With this package we hope to bring new life in the original ROS Melodic version of the hoverboard-hack.

How to build it in a new workspace
cd your_new_ws
git clone https://github.com/BartVanderHaagen/ros2_hover_diffbot.git
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

How to use it in an existing workspace
cd
git clone https://github.com/BartVanderHaagen/ros2_hover_diffbot.git
sudo cp -r /src/hoverboard_driver ~/your_existing_ws/src/
cd ~/your_exising_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

How to run it
