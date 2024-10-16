# Checkpoint19_Robot_Arm_Kinematics

## Task 1

Terminal 1

```
cd ~/catkin_ws
rosrun antropomorphic_project generate_matrixes.py
```

## Task 2

Terminal 1
```
rosrun antropomorphic_project fk_antropomorphic_arm.py
```

## Task 3

Terminal 1

```
source ~/simulation_ws/devel/setup.bash

roslaunch antropomorphic_arm_gazebo main.launch
```

Terminal 2

```
rosrun antropomorphic_project ik_antropomorphic_arm.py
```


Terminal 3

```
roslaunch antropomorphic_arm_control antropomorphic_arm_sim_rqt.launch
```
and input values from Terminal 2.

Terminal 4: Check Result
```
rostopic echo /end_effector_real_pose
```

