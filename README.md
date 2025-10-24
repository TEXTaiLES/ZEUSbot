# ROS2 for Beginners 2
First, we’ll see how to create a world in Gazebo, build our own robot car, and view it in RViz and Gazebo .

## Gazebo Create World
### Install Gazebo 
1. Open the terminal and type:
     ```shell
     sudo apt install ros-humble-gazebo-*
     ```
  Close and reopen the terminal
   
### Gazibo world

2. To launch it, type gazebo.
In Insert, there’s Add Path.
At http://models.gazebosim.org/
 you can find ready-made Gazebo models.

3. To create walls: Edit → Building Editor.
Draw the walls, and to save: File → Save As (Ctrl+Shift+S) → enter the name and where to save → Save → Exit (in the next window).

4. To save the world: File → Save World As → enter the name (name.world) and location → Save.

To open the world you created, you need to be in the folder where you saved it
(hercules@hercules:~/gazebo_examples$)
and type in the terminal:
 ```shell
   gazebo (name.world)
```
 <div style="text-align:center;">
    <img src="image/4.png" alt="4" width="800">
</div>

## Create My Robot URDF
### Setup the URDF File
1. I create a folder named my_robot.

Inside the folder, I type:

 ```shell
   touch my_robot.urdf
   code my_robot.urdf
```

2. Open it in Visual Studio Code and write the code (it’s in the `my_robot.urdf` file inside the my_robot folder).

In the terminal, I run:

```shell
 ros2 launch urdf_tutorial display.launch.py model:=/home/hercules/my_robot/my_robot.urdf 
```

******Notes

For connecting the wheel, we use the joint type ``type="continuous"``.

For parts we don’t want to be movable and just want to group, we use `type="fixed"`.

 <div style="text-align:center;">
    <img src="image/3.png" alt="3" width="800">
</div>

##  URDF+Xacro creation ,Visualize in RVIZ2 and Gazebo +Teleoperation

1. I create a folder learn_ros2_ws and a src subfolder.
 Then I go into src and type:
```shell
  ros2 pkg create --build-type ament_cmake --node-name my_node my_first_pkg (ονομα αρχειου το my_first_pkg)
```

****If I get ERRORS****
```shell
  nano /home/hercules/learn_ros2_ws/src/my_first_pkg/package.xml
```
and I fill in
Find the label:
```shell
<license>TODO: License declaration</license>
```
Replace it with one of the recommended licenses supported by ament.

For example, if you’re using Apache 2.0:

```shell
<license>Apache-2.0</license>
cd ..
```

For completeness, create a LICENSE file in your package folder:

```shell
touch /home/hercules/learn_ros2_ws/src/my_first_pkg/LICENSE
```

Step 4: Re-running the build

Save the changes and rerun the build:

```shell
colcon build 
```

And to run the URDF file, I need to be outside the folder and run the command:
```shell
ros2 launch urdf_tutorial display.launch.py model:=/home/hercules/learn_ros2_ws/src/my_first_pkg/urdf/my_robot1.urdf 
```

Then I create three folders—``include``, ``launch``, and ``rviz``—inside`` my_first_pkg``.
Then I rerun the ``.urdf`` file and save it in the ``rviz`` folder.
Next, in launch I create two .py files (``display.launch.py`` and ``gazebo.launch.py``), and I’ll run these files from now on.

I also go to the ``CMakeLists.txt`` file and add this:
```shell
#install launch files
install(DIRECTORY
 launch
 urdf
 rviz
 DESTINATION share/${PROJECT_NAME}/
)
```

Next, I go to the terminal, run ``cd`` until I’m in the ``learn_ros2_ws`` folder, run ``ls``, and then run again:
```shell
 colcon build
 ```
after
```shell 
colcon build --symlink-install
source install/local_setup.bash
```

And then I do the following:
```shell
gedit ~/.bashrc  (Ανοιγει το αρχειο bashrc)
```

And I add this: ``source ~/learn_ros2_ws/install/setup.bash``
and in the terminal I type:
```shell
source .bashrc
```
and it should look like this:
```shell
hercules@hercules:~$ source .bashrc
hercules@hercules:~$ 
```
Finally, to run it: 

```shell
ros2 launch my_first_pkg display.launch.py
```
 <div style="text-align:center;">
    <img src="image/2.png" alt="2" width="800">
</div>

Or, to run it in Gazebo, I type:

```shell
ros2 launch my_first_pkg gazebo.launch.py
```
 <div style="text-align:center;">
    <img src="image/1.png" alt="1" width="800">
</div>


## Keyboard control from the PC
In a separate terminal window
```shell
 ros2 topic list 
 ```
 and it should display the following:
```
/clock
/cmd_vel
/joint_states
/odom
/parameter_events
/performance_metrics
/robot_description
/rosout
/tf
/tf_static
```
and after
```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

and it should display the following:
```
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit

currently:	speed 0.5	turn 1.0 
```

### If there’s an issue re-running

In the terminal, I type:
```shell
ps aux | grep gazebo
```
I check the running processes
and kill the first one with the command `kill -9 <pid>`.
Then, in the terminal, I type again:
```shell
ps aux | grep gazebo
```
and it should show only one (process), not others; then I run the code again.

### Robot colors
http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials

### Defining the robot’s geometry
http://wiki.ros.org/urdf/XML/link

### Assembling robot components 
http://wiki.ros.org/urdf/XML/joint

## Create Robot Arm in RVIZ (file .Xacro)

 <div style="text-align:center;">
    <img src="image/5.png" alt="5" width="800">
</div>

1. I create a folder named ``braxionas`` and, inside it, a ``src`` folder.
```shell
colcon build
```
1.  Inside the ``src`` folder, I run the command:
 ```shell
ros2 pkg create braxionas
```
I delete the ``include`` and ``src`` folders and create ``launch``, ``urdf``, ``meshes``, and ``rviz``.
I configure the ``CMakeLists.txt`` file appropriately, and then…

1. I open the .bashrc file.
```shell
gedit ~/.bashrc
```
```shell
source ~/braxionas/install/setup.bash
```
and then in the braxionas folder (``log``, ``install``, ``src``, ``build`` ).

```shell
colcon build
colcon build --symlink-install
```
after   ```source install/setup.bash```

```shell
ros2 launch braxionas display.launch.xml
```
or
```shell
ros2 launch braxionas display.launch.py
```

and the result is:
 <div style="text-align:center;">
    <img src="image/6.png" alt="6" width="800">
</div>

### NOTES
The ``.stl`` files also exist in the [Repositories](https://github.com/HerGousis/Robotic_ARM)

## Robot in the  my world (Gazebo) 

1. I create a folder named ``my_robot_description`` and, inside it, a ``src`` folder.
```shell
colcon build
```
  Then we add the files that are uploaded to the ``my_robot_description/src`` folder.
Next, in the`` my_robot_description`` folder (``log``, ``install``, ``src``, ``build`` ), I type:

```shell
colcon build
colcon build --symlink-install
```
after  
```shell 
source install/setup.bash
```
and finally:
```shell
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```

in a separate terminal 
```shell
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5},angular:{z: 0}}" 
```
and we’ll see it move along the X-axis within the world we’ve created.

or 
```shell
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5},angular:{z: 0.5}}" 
```
we’ll observe circular motion

 <div style="text-align:center;">
    <img src="image/7.png" alt="7" width="800">
</div>

## Add Camera in Gazebo 

1. Inside the ``urdf`` folder (``my_robot_description/src/my_robot_description/urdf``), add one more file,``camera.xacro``, and include it in the ``my_robot.urdf.xacro`` file.
Then, in the ``my_robot_description`` folder (``log``, ``install``, ``src``, ``build`` ), I type:

```shell
colcon build
colcon build --symlink-install
```

after
```shell 
source install/setup.bash
```
and finally:
```shell
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```

 <div style="text-align:center;">
    <img src="image/8.png" alt="8" width="800">
</div>

* In RViz, to add the camera, click ``Add``, choose ``Image``, and then under `` Topic `` select your camera.

## Add lidar

We create a new file ``lidar.xacro`` and add it to ```my_robot.urdf.xacro```

```shell
colcon build
colcon build --symlink-install
```

after   
```shell 
source install/setup.bash
```
and finally:
```shell
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```

 <div style="text-align:center;">
    <img src="image/9.png" alt="9" width="800">
</div>

* Regarding ROS 2 in ``lidar.xacro`` )            
 ```shell
 <plugin name="gazebo_ros_ray_sensor" filename="libgazebo_ros_ray_sensor.so">
     <topic_name>/scan</topic_name>
     <frame_name>lidar_link</frame_name>
     <output_type>sensor_msgs/LaserScan</output_type>
 </plugin> 
```

 <div style="text-align:center;">
    <img src="image/10.png" alt="10" width="800">
</div>

 <div style="text-align:center;">
    <img src="image/11.png" alt="11" width="800">
</div>

* Note: If you mean LiDAR, pick ``LaserScan`` and select the LiDAR topic (e.g., ``/scan`` ). For a camera feed, choose ``Image`` instead of ``LaserScan``



## Install OpenCV

In the terminal, I type:
```shell
pip install opencv-python
```
and after
```shell
sudo apt install ros-humble-cv-bridge
```
## Create Python Script 

I write in the folder. ``` my_robot_description/src ```

 ```shell
 ros2 pkg create robot_controller --build-type ament_python --dependencies rclpy
```

and the folder is created. ```robot_controller``` 

* in the file ```package.xml```

i write:

```shell
  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>robot_state_publisher</depend>
  <depend>my_robot_description</depend>
```

* in the file ```setup.py```
i write:

```shell
     entry_points={
        'console_scripts': [
            "go=robot_controller.go:main",
            "go_with_laser=robot_controller.go_with_laser:main"
        ],
    },
```
* and I write the code in a ``.py`` file where the```__init__.py``` is  


Finally, I execute it in the folder ``` my_robot_description/```
```shell
colcon build
colcon build --symlink-install
```
I open two terminal windows in that folder ```my_robot_description``` 

* in one terminal, I type: 
```shell 
source install/setup.bash
```
and after  
```shell
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```

* in the other one, I type:

```shell 
source install/setup.bash
```
and after 
```shell
ros2 run robot_controller go_with_laser 
```
or

```shell
ros2 run robot_controller go
```
or

```shell
ros2 run robot_controller go_with_lidar 
```

The ``go_with_lidar.py`` file calls other ``.py`` files from the ``utils``  folder and is essentially the same Python script as ``go_with_laser``.
### NOTES
The ``go_with_laser.py`` or ``go_with_lidar.py`` file makes the robot move around an object while keeping a constant distance of 1.0.
It also saves the LiDAR readings recorded during the simulation in the ``laser_data`` folder.
Finally, using OpenCV, it saves a photo every 10 seconds in the image_data folder.
When I run ``go_with_laser.py`` or ``go_with_lidar.py`` again, it deletes the old photos, adds new ones, and updates the `` laser_data.txt `` file.

https://github.com/user-attachments/assets/7c2a3cc9-6bfc-4115-971e-3b9456bf6529

## SLAM spatial mapping

Open three terminals.
In the first one: ```hercules@hercules:~/my_robot_description$```
  ```shell
 ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```

in the second one: ```hercules@hercules:~/my_robot_description$```
  ```shell
ros2 launch slam_toolbox online_async_launch.py
```
in the third one: ```hercules@hercules:~/my_robot_description$```
  ```shell
ros2 run robot_controller go_with_lidar 
```

![Image](https://github.com/user-attachments/assets/6f019e28-02d1-4efb-adf9-2cd26e59a102)


## Add Qr and Object στο Gazebo
### Qr
from the Github `` https://github.com/mikaelarguedas/gazebo_models``

In the terminal, I run:
 ```shell
git clone https://github.com/mikaelarguedas/gazebo_models.git
cd gazebo_models/ar_tags/scripts/
python3 generate_markers_model.py -i /home/hercules/gazebo_models/ar_tags -s 1000 -w 500 -v
```

and I open Gazebo and search for the object.

### Object 

I add the object in Gazebo.
We need ``.obj``, ``.jpg``, and `` .mtl `` files.

I save the world and update the code wherever necessary.

 <div style="text-align:center;">
    <img src="image/14.png" alt="14" width="800">
</div>

## Add UR5 and robot in RVIZ and Gazebo (file .Xacro)
<div style="text-align:center;">
    <img src="image/15.png" alt="15" width="800">
</div>

Create a new file ``ur5.xacro`` and add it to ``my_robot.urdf.xacro`` by writing:
 ```shell
 <xacro:include filename="ur5.xacro" />
```
i write:
```shell
colcon build
colcon build --symlink-install
```

after  
```shell 
source install/setup.bash
```
and finally: 
```shell
ros2 launch my_robot_description display.launch.xml 
```
or
```shell
 ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```

 
 <div style="text-align:center;">
    <img src="image/21.png" alt="21" width="800">
</div>

## COLMAP

From the images saved in the `` image_data ``  folder, and using COLMAP
 <div style="text-align:center;">
    <img src="image/16.png" alt="16" width="800">
</div>
 <div style="text-align:center;">
    <img src="image/17.png" alt="17" width="800">
</div>

## RealityCapture
Using the saved images in `` RealityCapture ``, we can reconstruct and export the 3D object.”
 <div style="text-align:center;">
    <img src="image/22.jpg" alt="22" width="800">
</div>

With `` MeshLab ``, we can view the 3D model we created

 <div style="text-align:center;">
    <img src="image/22.png" alt="22" width="800">
</div>

## Marker Detection ROS2

Create a new file `` qr.py `` and then run:

```shell
colcon build
colcon build --symlink-install
```

after  
```shell 
source install/setup.bash
```
and finally: 
```shell
 ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```
in a separate terminal:)

```shell
ros2 run robot_controller qr
```

</div>
 <div style="text-align:center;">
    <img src="image/18.png" alt="18" width="800">
</div>

## Locating the point of the object with marker in rviz ROS2

</div>
 <div style="text-align:center;">
    <img src="image/20.png" alt="20" width="800">
</div>

Create a new file `` marker_kalo.py `` and then run:
```shell
colcon build
colcon build --symlink-install
```
after  
```shell 
source install/setup.bash
```
and finally:
```shell
 ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```
in a separate terminal:

```shell
ros2 run robot_controller marker_kalo
```



## RANSAC-3D and marker-based detection to guide the robot to the target object.

</div>
 <div style="text-align:center;">
    <img src="image/23.png" alt="23" width="800">
</div>

I created a file called ``ransac.py``and then run:
```shell
colcon build
colcon build --symlink-install
```
after  
```shell 
source install/setup.bash
```
and finally:
```shell
 ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```
in a separate terminal:

```shell
ros2 run robot_controller ransac
```

https://github.com/user-attachments/assets/2f50b36e-cfe0-41eb-b17f-04c3f6ffb506

## ROS2 Moveit_2_Control a Robotic Arm

Inside the `src` we put the folder `my_robot_moveit_config`

and then run:
```shell
colcon build
colcon build --symlink-install
```
after  
```shell 
source install/setup.bash
```
and finally:
```shell
 ros2 launch my_robot_moveit_config demo.launch.py 
```
![Image](https://github.com/user-attachments/assets/0442bae0-1f62-4464-8f9b-f21401021e6f)


## ROS2 Robotic Arm with Inverse Kinematics

Inside the `my_robot_description` 

and then run:
```shell
colcon build
colcon build --symlink-install
```

after  
```shell 
source install/setup.bash
```
and finally:
```shell
 ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```
in a separate terminal:

for test: 
```
ros2 topic pub  -1 /set_joint_trajectory trajectory_msgs/msg/JointTrajectory '{header:{frame_id: base_arm_link}, joint_names: [arm_base_forearm_joint, forearm_hand_joint], points: [ {positions: {0.0, 0.1}} ] }'
```
and with IK :

```shell
python3 arm_control.py
```


![Image](https://github.com/user-attachments/assets/5501c1ec-950c-4470-9387-5def4679facc)


## ROS2 UGV with 3 Dof Robotic_Arm

```shell
colcon build
colcon build --symlink-install
```
after  
```shell 
source install/setup.bash
```
and finally:
```shell
 ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 
```
in a separate terminal:

```shell
ros2 run robot_controller go_with_arm
```

![Image](https://github.com/user-attachments/assets/e2378e6b-73fe-4871-864f-b0aeccee0c73)

and the result 

</div>
 <div style="text-align:center;">
    <img src="image/result.png" alt="23" width="800">
</div>


and collection images is inside to the florder `data` the file `image_data_arm2`

## ROS2 UGV with 6 Dof Robotic_Arm
```shell
colcon build
colcon build --symlink-install
```
after  
```shell 
source install/setup.bash
```
and finally:
```shell
 ros2 launch my_robot_bringup my_robot_gazebo.launch.xml 

```
in a separate terminal:

```shell
python3 arm_control.py
```

![Image](https://github.com/user-attachments/assets/fbc66810-228a-4826-aad3-cc7919c49719)

Or

in a separate terminal:

```shell
ros2 run robot_controller go_with_arm
```

![Image](https://github.com/user-attachments/assets/f3a1d00b-75ff-486b-979e-7e5efe5551b1)

and the result 

</div>
 <div style="text-align:center;">
    <img src="image/result2.png" alt="23" width="800">
</div>


and collection images is inside to the florder `data` the file `image_data_arm3`
