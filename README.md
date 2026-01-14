# ICRA BARN Navigation Challenge
ICRA BARN Navigation Challenge simulation worlds, testing scripts and baseline navigation stack (e.g. DWA, Eband). World Index 0 - 299 are static world, world index 300 - 359 are dynamic world (DynaBarn).

---

### ***Navigation Stack Test Result:*** 
upload your benchmark result [here](https://entuedu-my.sharepoint.com/:x:/r/personal/ongd0017_e_ntu_edu_sg/_layouts/15/Doc.aspx?sourcedoc=%7B6FAD4274-1E4D-4E65-AC25-B05687519509%7D&file=Nav%20Stack%20Test%20Result.xlsx&action=default&mobileredirect=true)

## Table of Contents
- [Installation](#installation)
- [Scripts](#scripts)
  - [run.py](#runpy)
  - [benchmark.sh](#benchmarksh)
  - [report_test.py](#report_testpy)
  - [playground.py](#playgroundpy)

## Requirements
> ⚠️ Please use ROS melodic as ROS noetic is not working for DynaBarn. Replace `<YOUR_ROS_VERSION>` below to `melodic`

If you run it on a local machine without containers:
* ROS Melodic
* Ubuntu-18.04

If you run it in Singularity containers:
* Go version at least 1.13
* Singularity version at least 3.6.3 and less than 4.02

## Installation
Follow the instructions below to run simulations on your local machines. (You can skip 1-6 if you only use Singularity container)
> ⚠️ Singularity container image is outdated please use local machine instead

1. Optionally, create a virtual environment (we show examples with python venv, you can use conda instead)
```
sudo apt -y update; sudo apt-get -y install python3-venv
python3 -m venv /<YOUR_HOME_DIR>/nav_challenge
export PATH="/<YOUR_HOME_DIR>/nav_challenge/bin:$PATH"
```

2. Install Python dependencies
```
pip3 install defusedxml rospkg netifaces numpy psutil pyyaml
```

3. Create ROS workspace
```
mkdir -p /<YOUR_HOME_DIR>/jackal_ws/src
cd /<YOUR_HOME_DIR>/jackal_ws/src
```

4. Clone this repo and required ros packages: (replace `<YOUR_ROS_VERSION>` with your own, e.g. melodic)
```
git clone https://github.com/Team-Robo/the-barn-challenge-robo.git
git clone https://github.com/jackal/jackal.git --branch <YOUR_ROS_VERSION>-devel
git clone https://github.com/jackal/jackal_simulator.git --branch <YOUR_ROS_VERSION>-devel
git clone https://github.com/jackal/jackal_desktop.git --branch <YOUR_ROS_VERSION>-devel
git clone https://github.com/utexas-bwi/eband_local_planner.git
```

5. Install ROS package dependencies: (replace `<YOUR_ROS_VERSION>` with your own, e.g. melodic)
```
cd ..
source /opt/ros/<YOUR_ROS_VERSION>/setup.bash
rosdep init; rosdep update --rosdistro <YOUR_ROS_VERSION>
rosdep install -y --from-paths . --ignore-src --rosdistro=<YOUR_ROS_VERSION>
```

6. Build the workspace (if `catkin_make` fails, try changing `-std=c++11` to `-std=c++17` in `jackal_helper/CMakeLists.txt` line 3)
```
catkin_make
source devel/setup.bash
```

Follow the instruction below to run simulations in Singularity containers.

1. Follow this instruction to install Singularity: https://sylabs.io/guides/3.0/user-guide/installation.html. Singularity version >= 3.6.3 and <= 4.02 is required to successfully build the image!

2. Clone this repo
```
git clone https://github.com/Daffan/the-barn-challenge.git
cd the-barn-challenge
```

3. Build Singularity image (sudo access required)
```
sudo singularity build --notest nav_competition_image.sif Singularityfile.def
```

## Run Simulations
Navigate to the folder of this repo. Below is the example to run move_base with DWA as local planner.

If you run it on your local machines: (the example below runs [move_base](http://wiki.ros.org/move_base) with DWA local planner in world 0)
```
source ../../devel/setup.sh
python3 run.py --world_idx 0
```

If you run it in a Singularity container:
```
./singularity_run.sh /path/to/image/file python3 run.py --world_idx 0
```

A successful run should print the episode status (collided/succeeded/timeout) and the time cost in second:
> \>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
>
> Navigation collided with time 27.2930 (s)

> \>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
>
> Navigation succeeded with time 29.4610 (s)


> \>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
>
>Navigation timeout with time 100.0000 (s)

## Test your own navigation stack
We currently don't provide a lot of instructions or a standard API for implementing the navigation stack, but we might add more in this section depending on people's feedback. If you are new to the ROS or mobile robot navigation, we suggest checking [move_base](http://wiki.ros.org/move_base) which provides basic interface to manipulate a robot.

The suggested work flow is to edit section 1 in `run.py` file (line 89-109) that initialize your own navigation stack. You should not edit other parts in this file. We provide a bash script `test.sh` to run your navigation stack on 50 uniformly sampled BARN worlds with 10 runs for each world. Once the tests finish, run `python report_test.py --out_path /path/to/out/file` to report the test. Below is an example of DWA:
```
python report_test.py --out_path res/dwa_out.txt
```
You should see the report as this:
>Avg Time: 33.4715, Avg Metric: 0.1693, Avg Success: 0.8800, Avg Collision: 0.0480, Avg Timeout: 0.0720

Except for `DWA`, we also provide three learning-based navigation stack as examples (see branch `LfH`, `applr` and `e2e`).

## Submission
Submit a link that downloads your customized repository to this [Google form](https://docs.google.com/forms/d/e/1FAIpQLSfZLMVluXE-HWnV9lNP00LuBi3e9HFOeLi30p9tsHUViWpqrA/viewform). Your navigation stack will be tested in the Singularity container on 50 hold-out BARN worlds sampled from the same distribution as the 300 BARN worlds. In the repository, make sure the `run.py` runs your navigation stack and `Singularityfile.def` installs all the dependencies of your repo. We suggest to actually build an image and test it with `./singularity_run.sh /path/to/image/file python3 run.py --world_idx 0`. You can also refer to branch `LfH`, `applr` and `e2e`, which are in the correct form for submissions.

--------------------------------------------------------
## Scripts

### run.py

Runs a single navigation test in Gazebo simulation for a specified world.

```
usage: run.py [-h] [-w WORLD_IDX] [-g] [-l LAUNCH] [-o OUT] [-r]
              [-rc RVIZ_CONFIG] [-m] [-mn MNODES [MNODES ...]] [-my MYAML]

test BARN navigation challenge

optional arguments:
  -h, --help            show this help message and exit
  -w WORLD_IDX, --world_idx WORLD_IDX
                        BARN world index to run navigation, default 0
  -g, --gui             Enable Gazebo GUI
  -l LAUNCH, --launch LAUNCH
                        Navigation stack launch file in <ros package>/launch,
                        default move_base_DWA.launch
  -o OUT, --out OUT     Path for output logs .txt file, default <--launch>.txt
  -r, --rviz            Launch RViz
  -rc RVIZ_CONFIG, --rviz_config RVIZ_CONFIG
                        RViz config file in <ros package>/configs to be
                        launched, default common.rviz
  -m, --monitor         Enable resource usage (CPU & Memory) monitoring
  -mn MNODES [MNODES ...], --mnodes MNODES [MNODES ...]
                        List of ROS node names to monitor, e.g. /move_base
                        /amcl /map_server
  -my MYAML, --myaml MYAML
                        Path to YAML file specifying ROS nodes to
                        include/exclude for monitoring
```

**Examples:**
```
# Run test in world 5 
python3 run.py --world_idx 5

# Run test with custom launch file, with GUI and RViz, with custom rviz configuration file
# Launch file and rviz configuration file can be located in any ros packages
# Launch file in <ros_package>/launch, rviz config in <ros_packages>/configs
# Source the workspace so that it can be found
python3 run.py --launch move_base_eband.launch --gui --rviz --rviz-config eband.rviz

# Monitor node specified in yaml, checkout `monitor_nodes.yaml` to see its template
python3 run.py --monitor --myaml monitor_nodes.yaml
```

---

### benchmark.sh

Runs batch tests across multiple worlds. Tests specified worlds at intervals with multiple repetitions.

**Arguments:**
- `--launch` (list, default: move_base_DWA.launch) - One or more launch files to test
- `--start_idx` (int, default: 0) - Starting world index
- `--spacing` (int, default: 6) - Interval between worlds
- `--repeat` (int, default: 10) - Repetitions per world
- `--monitor` (flag) - Enable resource monitoring, similar to run.py argument
- `--mnodes` (list) - ROS nodes to monitor, similar to run.py argument
- `--myaml` (str) - YAML file specifying nodes to monitor, similar to run.py argument

**Examples:**
```
# Test multiple planners with custom spacing and repititions and monitor resource usage of specified nodes
./benchmark.sh --launch move_base_DWA.launch move_base_eband.launch --spacing 10 --repeat 5 --monitor --mnodes /move_base /amcl

# General
./benchmark.sh --launch <nav_launch_file> --start_idx 0 --spacing 4 --repeat 2 --monitor [--mnodes <nodes to monitor> | --myaml monitor_nodes.yaml]
```
---

### report_test.py

Analyzes and aggregates test results from log files generated by `run.py`. Outputs statistics for static and dynamic environments.

**Arguments:**
- `--out_path` (str, required) - Path to output log file from run.py

**Examples:**
```
# Generate report
python3 report_test.py --out_path move_base_DWA.launch.txt
```

---

### playground.py

Launches an interactive Gazebo simulation with navigation stack for debugging and exploration (no automated goals or metrics).

You can easily set a goal pose in rviz using `2d goal pose` or do it via command line.

**Arguments:**
- `--world_idx` (int, default: 0) - BARN world index
- `--gui` (flag) - Enable Gazebo GUI
- `--launch` (str, default: move_base_DWA.launch) - Navigation launch file
- `--rviz_config` (str, default: common.rviz) - RViz config file

**Examples:**
```
# Interactive session with GUI
python3 playground.py --world_idx 0 --gui

# Test custom planner
python3 playground.py --world_idx 15 --gui --launch move_base_eband.launch
```
#### set goal using command line
Publish goal to topic
```
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
'header: {frame_id: "odom"} pose: {position: {x: 2.0, y: 3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'

```
Using `move_base` action server gui to send goal interactively
```
rosrun actionlib axclient.py /move_base
```

