# BlueROV2Heavy-Gazebo-with-IDMB-CBF-DOB

​	This repository is based on the model "**bluerov2-heavy configuration**" (with 8 thrusters), we have completed setting physics attributes(inertia, hydrogen damps, etc.). Thanks to [this repository](https://github.com/HKPolyU-UAV/bluerov2.git) for providing a reference for my model building.



Results of obstacle avoidance experiments (Exp5) are presented below.

|      | safe                                                    | collision & solve failed                                  |
| ---- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| scene1 | ![image](https://github.com/CLASS-Lab/BlueROV2Heavy-Gazebo-with-IDMB-CBF-DOB/blob/main/gifs/scene1.gif) | ![image](https://github.com/CLASS-Lab/BlueROV2Heavy-Gazebo-with-IDMB-CBF-DOB/blob/main/gifs/scene1_failed.gif) |
| scene2 | ![image](https://github.com/CLASS-Lab/BlueROV2Heavy-Gazebo-with-IDMB-CBF-DOB/blob/main/gifs/scene2.gif) | ![image](https://github.com/CLASS-Lab/BlueROV2Heavy-Gazebo-with-IDMB-CBF-DOB/blob/main/gifs/scene2_failed.gif) |


Part of MATLAB simulation results can be seen in [robin-bird-go/MATLAB-IDMB-CBF-DOB](https://github.com/robin-bird-go/MATLAB-IDMB-CBF-DOB)





## 0. first things first

1. cmake version: **`3.22.0`**(recommended!),using **higher** versions could cause compiling **errors**.

2. to prevent annoying "TF_REPEATED_DATA" warning in your terminal, I strongly suggest you find "[this_workspace_dir]/src/uuv_simulator/uuv_gazebo_plugins/uuv_gazebo_ros_plugins/src/UnderwaterObjectROSPlugin.cc" and **comment** 

   ````c++
   this->tfBroadcaster.sendTransform(this->nedTransform);
   ````

   **code in line 208**. Because it force gazebo to publish repeated tf from "base_link" to "base_link_ned"!!! Which has bothered me for a year T_T.

3. don't forget to **add the directory of our gazebo models** in your home **.~bashrc**  file!

   ```shell
   export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:[your workspace path]/src/bluerov2_description/worlds
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:[your workspace path]/src/bluerov2_description/models
   ```

   


## 1. build ws

clone **[(uuv_simulator)][https://github.com/uuvsimulator/uuv_simulator.git]** to current workspace package folder, the tree is:

── current_ws
│   ├── fish_1_plugin
│   ├── fish_2_plugin
│   ├── README.md
│   ├── run_build.sh
│   └── src
│       ├── bluerov2_control
│       ├── bluerov2_description
│       ├── bluerov2_gazebo
│       ├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
│       ├── dyn_mb_mpc_cbf_dob
│       ├── dyn_ob_tf_pub
│       ├── mb_mpc_cbf_dob
│       ├── new_mpc_cbf
│       ├── new_mpc_cbf_dob
│       └── **uuv_simulator** 

use `sh run_build.sh`, we also include **building of  2 fish plugins** here.

## 2. launch

all these available launch files are in the **`src/bluerov2_gazebo/launch`** dir

### 2.0.debug

we use [plotjuggler](https://github.com/facontidavide/PlotJuggler) to debug, it's really useful!

### 2.1. quick_start

launch basic env and bluerov2(only can see if it's correctly running)

### 2.2. start with thrust manager

based on 2.1, add thrust manager

and can control BR2 with topic "/bluerov2/thruster_manager/input"

### 2.3. start mpc demo

based on 2.1 and mpc pkg

- params that can be edited:

| all descriptions are <br />based on standard frame | data                                                        |
| -------------------------------------------------- | ----------------------------------------------------------- |
| mpc_go_to_target_pos.launch                        | this is a demo from a start point to <br />target point     |
| start mpc demo.lauch                               | arg **record** value(file stored in `bluerov2_gazebo/bags`) |
| `mpc_node.h`                                       | UUV target: [5,2.-10]                                       |
| `start_mpc_demo.launch`                            | UUV init pose:[-5,-2,-5]                                    |
| br2_mpc/config/mpc_model.yaml                      | Q/R/P<br />x0<br />init_state                               |

if set `record=true` , use plotjuggler to debug!

### 2.4. new_start_mpc_cbf_dob_demo

start a demo using mpc-cbf-dob controller

### 2.5. start_mb_mpc_cbf_dob_demo

start a demo using mpc-cbf-dob controller **with our move blocking method**

### 2.6. start_dyn_mb_mpc_cbf_dob_demo

this is the final demo that uses our whole methods: including our dynamic CBF \ MB, and you can check the performance by launch this.



## 3. model description

### 3.1. thruster configuration

​	based on [thesis_wu](https://flex.flinders.edu.au/file/27aa0064-9de2-441c-8a17-655405d5fc2e/1/ThesisWu2018.pdf) in pdf page 48, and in gazebo, the T(alpha) needs to be adapted:

1. the positive direction of x/y/z axis is the standard setting: front, **left and upper**, so the 2nd/ 3rd/ 5th/ 6th row of T need to time (-1).
2. the vertical thrust is default set as upper direction, so the 5-8 th columns also need to be changed.   

### 3.2. model files

1. dae file uses the one from [this repo](https://github.com/PX4/PX4-SITL_gazebo-classic/tree/f754540e714642fea897445e69a675245bc6306a/models/uuv_bluerov2_heavy/meshes) 
2. the pose of 8 thrusters refers to [this repo](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/f754540e714642fea897445e69a675245bc6306a/models/uuv_bluerov2_heavy/uuv_bluerov2_heavy.sdf)
3. stl file is from the [official website  ](https://grabcad.com/library/bluerobotics-bluerov2-heavy-1/details?folder_id=4856403)    **(which is too large that I don't include on this repo, you need to<font  color=Red> download by yourself!)</font>**
4. If you want to change the bluerov2's system parameters, checkout the 2 files:
   - src/bluerov2_description/urdf/gazebo.xacro
   - src/bluerov2_description/urdf/base.xacro

### 3.3. obstacles setting

If you want to change the positions of the obstacles, then you need to edit following files:

| file path                                                    | keywords<br />(use <font  color=Blue> search </font> to locate) | change                                                       |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| src/bluerov2_gazebo/launch/new_start_for_cbf.launch          | static_transform_publisher                                   | \<node pkg="tf" type="static_transform_publisher" name="world_to_ob_tf" args="x  y  z phi theta psi  world ob 100" /><br />give obstacle's pos in **std** frame |
| src/new_mpc_cbf/include/new_mpc_cbf/new_mpc_cbf_node.h<br />(Similarly for the rest) | obstacles_std\obstacles                                      | double obstacles_std\[N_OB][8] =<br />{**x  y  z phi theta psi**  }; <br />obstacles also |
| src/bluerov2_description/worlds/new_world.world              | include                                                      | \<include><br />\<uri>model://ob</uri><br />\<pose>**x  y  z phi theta psi**\</pose><br />... |
| src/bluerov2_description/urdf/all_obstacles.urdf             | link & joint                                                 | just define your obstacle a boundary                         |

and don't forget to `catkin_make`

### 3.4. initial position setting

If you want to change the positions of bluerov2, you need to edit following files:

| file path                                                | keywords<br />(use <font  color=Blue> search </font> to locate) | change                         |
| -------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------ |
| src/new_mpc_cbf/src/new_mpc_cbf_node.cpp                 | NewMpcCbfNode::NewMpcCbfNode                                 | bluerov2_states({x,y,z...      |
| src/bluerov2_gazebo/launch/new_start_mpc_cbf_demo.launch | new_start_for_cbf                                            | \<arg name="x" value="35.5"/\> |

and don't forget to `sh run_build.sh`



## 4. control params setting

to some convenience, we define some params in cpp files and others in launch files, you can refer to this table:

| file path                                                | param                                    | meaning                                                     |
| -------------------------------------------------------- | ---------------------------------------- | ----------------------------------------------------------- |
| src/bluerov2_gazebo/launch/new_start_mpc_cbf_demo.launch | horizon<br />K_alpha                     | 1.change the predict steps;<br />2.change the param for CBF |
| the same as above                                        | omega_1<br />v1<br />alpha<br />l_factor | 3.change params for DOB                                     |



## 5. dynamic fish

to build and use the animated fish plugins, you need to enter the dir of *_plugin/  folder, and 

```shell
mkdir build
cd build
cmake ../
make
```

then go to edit your **.~bashrc** file in your **home** directory, add this

```shell
export GAZEBO_PLUGIN_PATH= [your workspace path]/fish_1_plugin/build:$GAZEBO_PLUGIN_PATH
export GAZEBO_PLUGIN_PATH= [your workspace path]/fish_2_plugin/build:$GAZEBO_PLUGIN_PATH
```



