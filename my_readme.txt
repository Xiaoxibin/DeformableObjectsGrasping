11111111111111111111111111111111111111111111111111111111111.机械臂启动节点

连网
cd ~/DeformableObjectsGrasping-master
catkin_make
source ./devel/setup.bash

真实：roslaunch dsr_launcher single_robot_gazebo.launch mode:=real host:=192.168.5.100 port:=12345
可以测试一下，机械臂是否运行，rosrun dsr_example_py single_robot_simple.py




2222222222222222222222222222222222222222222222222222222222222.灵巧手启动节点

连接到电脑，查看串口：ls -l /dev |grep ttyUSB
切换到/sys/bus/usb-serial，修改ttyUSB0和ttyUSB1中的latency_timer文件内容为1，
    	其中ttyUSB0为seed_robotics占用端口，ttyUSB1为sensor_pkg占用端口，需要修改对应的yaml文件
    	
sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB1
sudo gedit /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
sudo gedit /sys/bus/usb-serial/devices/ttyUSB1/latency_timer

进入工作空间：cd ~/DeformableObjectsGrasping-master
source ./devel/setup.bash
启动节点：roslaunch seed_robotics RH8D_R.launch           终端会报一行黄色和一行红色的错，表示节点已正确启动
可以测试一下，灵巧手是否运行：rosrun seed_robotics user_sample_2_set_speed_position.py


33333333333333333333333333333333333333333333333333333333333.运行测试代码

进虚拟环境(UniDexGrasp2)  : conda activate UniDexGrasp2 	
(或在 vscode 中添加解释器：Ctrl + Shit + p)
进入路劲： /home/xxb/DeformableObjectsGrasping-master/src/seed_robotics/user_samples
运行：  python3 a_sample_grasp.py 

########################################################灵巧手关节显示代码在 JoystickControlPosition.py中










