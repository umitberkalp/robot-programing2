i am using native one so firstly please go to terminal and copy paste it


sudo apt-get install -y --no-install-recommends build-essential cmake git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libglvnd0 libglx0 libegl1 libxext6 libx11-6 libusb-1.0* udev apt-transport-https ca-certificates swig python3-pip ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp ros-humble-rviz2* ros-humble-cartographer* ros-humble-nav* ros-humble-teleop-twist-keyboard ros-humble-joint-state-publisher* ros-humble-robot-state-publisher* ros-humble-xacro ros-humble-imu-tools libgflags-dev nlohmann-json3-dev ros-humble-image-* python3-colcon-common-extensions

git clone https://github.com/YDLIDAR/YDLidar-SDK.git && \
    mkdir -p YDLidar-SDK/build && cd YDLidar-SDK/build &&\
    cmake .. && make -j4 && sudo make install &&\
    cd .. && pip install . && cd .. && rm -rf YDLidar-SDK 

wget -c https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz -O glog-0.6.0.tar.gz &&\
    tar -xzvf glog-0.6.0.tar.gz && cd glog-0.6.0 &&\
    mkdir build && cd build && cmake .. && make -j4 && sudo make install &&\
    sudo ldconfig && cd ../.. && rm -rf glog-*

wget -c https://github.com/Neargye/magic_enum/archive/refs/tags/v0.8.0.tar.gz -O magic_enum-0.8.0.tar.gz &&\
    tar -xzvf magic_enum-0.8.0.tar.gz && cd magic_enum-0.8.0 &&\
    mkdir build && cd build && cmake .. && make -j4 && sudo make install &&\
    sudo ldconfig && cd ../.. && rm -rf magic_enum*

git clone https://github.com/libuvc/libuvc.git &&\
    cd libuvc &&\ 
    mkdir build && cd build && cmake .. && make -j4 && sudo make install &&\
    sudo ldconfig  && cd ../.. && rm -rf libuvc*

    then Setup the ROS2 environment

    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
. ~/.bashrc

install the robot simulation softwaresource install/setup.bash
ros2 launch limo_gazebosim limo_gazebo_diff.launch.py

then use the codes that visual stuido and please click and run it

git clone https://github.com/LCAS/limo_ros2
cd limo_ros2 && ./build.sh

Run the simulator and check that everything is running as expected

source install/setup.bash
ros2 launch limo_gazebosim limo_gazebo_diff.launch.py
then you can click on screen 

The robot in my system first travels around our entire map autonomously. The code is designed to turn right when it sees an obstacle.
In this way, it can travel the entire map without hitting obstacles.
In the second stage, pink colored potholes are detected.
After detecting the pink potholes, it shares the number with us.