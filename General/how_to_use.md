# This file describes the main steps to use the car

## Connection to the car and start of the ROS Nodes

### 1.Powering the car
1. Plug in the battery
2. Press the ON/OFF red button
3. Press the "start" red button 

**All the system is now powered**

### 2.Connection to the Raspberry PI
By default, the raspberry pi board connects to the IoT network at startup. It is also possible to connect the board to any other network by modifying the file "/etc/netplan/50-cloud-init.yaml".

1. Connect your pc to the same network as the raspberry (by default the IoT network)
2. In a terminal, run : "ssh pi@10.105.1.XX" (adapt the IP address according to the car and the network used)
3. Enter the password : "geicar"

**You are now in the raspberry environment**

### 3.Starting the ROS nodes (in the Raspberry PI board)
Run **"ros2 launch geicar_start geicar.launch.py"** to start all necessary nodes. 
You can see in the terminal the startup and the indications of the different nodes. 

**The nodes on the raspberry are now started. You can control the car with the XBOX controller**

### 4.Connection to the Jetson Nano
You can access the Jetson Nano card from the Raspberry. You have to establish a first ssh connection between the pc and the raspberry (step n°2), then establish the ssh connection between the raspberry and the jetson.

In another terminal :
1. Repeat step n°2 to connect to the raspberry again (you can check that you are in a Raspberry Pi terminal with the prompt, which indicates "pi@geicar")
2. Run "ssh jetson@192.168.1.10" to connect to the jetson nano card 
3. Enter the password : "geicar" 

**The prompt is now "jetson@geicar"**

### 5.Starting the ROS nodes (in the Jetson Nano board)
Once you are connected to the Jetson Nano (prompt jetson@geicar) :

1. Start and go into the docker container ros-humble : "sudo docker start -ai ros-humble", password : "geicar"

**The prompt is now "root@geicar"**

2. Run **"ros2 launch geicar_start_jetson geicar.jetson.launch.py"** to start the nodes that are in the jetson board (ie LIDAR and CAMERA)

**You can now see that the LIDAR is running and the camera is on**

### 6.Stop the nodes and turn OFF the car
1. Stop the nodes by pressing "Ctrl-C" in the launch terminals (in the Raspberry and in the Jetson)
2. Exit ssh connections with "exit"
3. Turn OFF the car by pressing the ON/OFF red button on the car


## Drive the car in manual mode (with the XBOX Controller)
Once you have started the nodes in the Raspberry PI (step n°3), you can drive the car with the joystick in Manual Mode **(make sure the car can move safely)** :

1. Power ON the joystick
2. _**Optional, but recommanded on first use** You can start the steering calibration by pressing "DPAD Bottom" + "START". Then follow the instructions given in the launch terminal (ros2 launch) on the Raspberry._ 
3. Press "start" (you can see that the car is started in the launch terminal (ros2 launch) on the Raspberry
4. You can select the mode : button "Y" for Manual (by default at startup), button "A" for Autonomous
5. Control the car with LT, RT, and the left joystick
6. Press "B" to stop the car

## See all messages exchanged by ROS nodes (including sensors data)
Once you have started the nodes (step n°3 and/or step n°5), you can see the different topics and messages published :

In a Raspberry Pi terminal or in a Jetson Nano terminal :
1. Run "ros2 topic list" to see all the topics (sometimes you need to run this command twice)
2. Run "ros2 topic echo NAME\_OF\_THE\_TOPIC" to display the messages published in the topic (for example : ros2 topic echo /us_data)

