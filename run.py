import time
import argparse
import subprocess
import os
from os.path import join
import psutil
import sys

import numpy as np
import rospy
import rospkg

from gazebo_simulation import GazeboSimulation

INIT_POSITION = [-2, 3, 1.57]  # in world frame
GOAL_POSITION = [0, 10]  # relative to the initial position

def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def path_coord_to_gazebo_coord(x, y):
        RADIUS = 0.075
        r_shift = -RADIUS - (30 * RADIUS * 2)
        c_shift = RADIUS + 5

        gazebo_x = x * (RADIUS * 2) + r_shift
        gazebo_y = y * (RADIUS * 2) + c_shift

        return (gazebo_x, gazebo_y)

def get_rosnode_pid(node_name):
    try:
        out = subprocess.check_output(
            ['rosnode', 'info', node_name],
            stderr=subprocess.DEVNULL
        ).decode()
        for line in out.splitlines():
            if "Pid:" in line:
                return int(line.split()[-1])
    except Exception:
        return None


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = 'test BARN navigation challenge')
    parser.add_argument('-w', '--world_idx', type=int, default=0, help="BARN world index to run navigation, default 0")
    parser.add_argument('-g', '--gui', action="store_true", help="Enable Gazebo GUI")
    parser.add_argument('-l', '--launch', type=str, default="move_base_DWA.launch", 
                        help="Navigation stack launch file in jackal_helper/launch, default move_base_DWA.launch")
    parser.add_argument('-o', '--out', type=str, default=None, 
                        help="Path for output logs .txt file, default <--launch>.txt")
    parser.add_argument('-r', '--rviz', action='store_true', help="Launch RViz")
    parser.add_argument('-rc', '--rviz_config', type=str, default="common.rviz", 
                        help="RViz config file in jackal_help/configs to be launched, default common.rviz")
    parser.add_argument('-m', '--monitor', action='store_true', help="Enable resource usage (CPU & Memory) monitoring")
    parser.add_argument('-mn', '--mnode', type=str, default="/move_base", 
                        help="ROS node name to monitor resource usage, default /move_base")

    args = parser.parse_args()

    if args.out is None:
        args.out = args.launch + ".txt"
    
    ##########################################################################################
    ## 0. Launch Gazebo Simulation
    ##########################################################################################
    
    os.environ["JACKAL_LASER"] = "1"
    os.environ["JACKAL_LASER_MODEL"] = "ust10"
    os.environ["JACKAL_LASER_OFFSET"] = "-0.065 0 0.01"
    
    if args.world_idx < 300:  # static environment from 0-299
        world_name = "BARN/world_%d.world" %(args.world_idx)
        INIT_POSITION = [-2.25, 3, 1.57]  # in world frame
        GOAL_POSITION = [0, 10]  # relative to the initial position
    elif args.world_idx < 360:  # Dynamic environment from 300-359
        world_name = "DynaBARN/world_%d.world" %(args.world_idx - 300)
        INIT_POSITION = [11, 0, 3.14]  # in world frame
        GOAL_POSITION = [-20, 0]  # relative to the initial position
    else:
        raise ValueError("World index %d does not exist" %args.world_idx)
    
    print(">>>>>>>>>>>>>>>>>> Loading Gazebo Simulation with %s <<<<<<<<<<<<<<<<<<" %(world_name))   
    rospack = rospkg.RosPack()
    base_path = rospack.get_path('jackal_helper')
    os.environ['GAZEBO_PLUGIN_PATH'] = os.path.join(base_path, "plugins")
    
    launch_file = join(base_path, 'launch', 'gazebo_launch.launch')
    world_name = join(base_path, "worlds", world_name)
    rviz_config = join(base_path, "configs", args.rviz_config)
    
    gazebo_process = subprocess.Popen([
        'roslaunch',
        launch_file,
        'world_name:=' + world_name,
        'gui:=' + ("true" if args.gui else "false"),
        'rviz:=' + ("true" if args.rviz else "false"),
        'rviz_config:=' + rviz_config
    ])
    time.sleep(5)  # sleep to wait until the gazebo being created
    
    rospy.init_node('gym', anonymous=True) #, log_level=rospy.FATAL)
    rospy.set_param('/use_sim_time', True)
    
    # GazeboSimulation provides useful interface to communicate with gazebo  
    gazebo_sim = GazeboSimulation(init_position=INIT_POSITION)
    
    init_coor = (INIT_POSITION[0], INIT_POSITION[1])
    goal_coor = (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1])
    
    pos = gazebo_sim.get_model_state().pose.position
    curr_coor = (pos.x, pos.y)
    collided = True
    
    # check whether the robot is reset, the collision is False
    while compute_distance(init_coor, curr_coor) > 0.1 or collided:
        gazebo_sim.reset() # Reset to the initial position
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        collided = gazebo_sim.get_hard_collision()
        time.sleep(1)




    ##########################################################################################
    ## 1. Launch your navigation stack
    ## (Customize this block to add your own navigation stack)
    ##########################################################################################
    
    launch_file = join(base_path, '..', 'jackal_helper/launch/', args.launch)
    nav_stack_process = subprocess.Popen([
        'roslaunch',
        launch_file,
    ])

    def terminate_ros_proc():
        gazebo_process.terminate()
        gazebo_process.wait()
        nav_stack_process.terminate()
        nav_stack_process.wait()

    if args.monitor:
        move_base_pid = None
        timeout_start_time = time.time()
        while move_base_pid is None and not rospy.is_shutdown() and time.time() - timeout_start_time < 5:
            move_base_pid = get_rosnode_pid(args.mnode)
            time.sleep(0.5)
        if move_base_pid is None:
            print(f"Failed to find the pid of node: {args.mnode}, fallback to no resource usage monitoring")
            args.monitor = False
        else:
            move_base_proc = psutil.Process(move_base_pid)
            move_base_proc.cpu_percent(interval=None)  # warm-up
            cpu_samples = []
            mem_samples = []
    
    # Make sure your navigation stack recives the correct goal position defined in GOAL_POSITION
    import actionlib
    from geometry_msgs.msg import Quaternion
    from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
    nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = 'odom'
    mb_goal.target_pose.pose.position.x = GOAL_POSITION[0]
    mb_goal.target_pose.pose.position.y = GOAL_POSITION[1]
    mb_goal.target_pose.pose.position.z = 0
    mb_goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

    nav_as.wait_for_server()
    nav_as.send_goal(mb_goal)




    ##########################################################################################
    ## 2. Start navigation
    ##########################################################################################
    
    curr_time = rospy.get_time()
    pos = gazebo_sim.get_model_state().pose.position
    curr_coor = (pos.x, pos.y)

    
    # check whether the robot started to move
    while compute_distance(init_coor, curr_coor) < 0.1:
        curr_time = rospy.get_time()
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        time.sleep(0.01)
    
    # start navigation, check position, time and collision
    start_time = curr_time
    start_time_cpu = time.time()
    collided = False
    if args.monitor:
        last_sample_time = rospy.get_time()
        cpu = mem = mem_percent = 0.0
    
    while compute_distance(goal_coor, curr_coor) > 1 and not collided and curr_time - start_time < 100:
        try:
            curr_time = rospy.get_time()
            pos = gazebo_sim.get_model_state().pose.position
            curr_coor = (pos.x, pos.y)
            if args.monitor: # resource monitoring in 2hz
                if rospy.get_time() - last_sample_time >= 0.5:
                    last_sample_time = rospy.get_time()
                    cpu = move_base_proc.cpu_percent(interval=None)
                    mem = move_base_proc.memory_info().rss / (1024 * 1024)
                    mem_percent = move_base_proc.memory_percent()
                    cpu_samples.append(cpu)
                    mem_samples.append(mem)
                print("Time: %.2f (s), x: %.2f (m), y: %.2f (m) | CPU: %.1f%% | MEM: %.1f MB (%.1f%%)" 
                    %(curr_time - start_time, curr_coor[0], curr_coor[1], cpu, mem, mem_percent), end="\r")
            else:
                print("Time: %.2f (s), x: %.2f (m), y: %.2f (m)" %(curr_time - start_time, *curr_coor), end="\r")

            collided = gazebo_sim.get_hard_collision()
            while rospy.get_time() - curr_time < 0.1:
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("Navigation Run Interrupted by User")
            terminate_ros_proc()
            sys.exit()
        except Exception as e:
            print(f"Error during navigation run: {e}")
            terminate_ros_proc()
            sys.exit()


    
    
    ##########################################################################################
    ## 3. Report metrics and generate log
    ##########################################################################################
    
    print("\n\n>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<")
    success = False
    if collided:
        status = "collided"
    elif curr_time - start_time >= 100:
        status = "timeout"
    else:
        status = "succeeded"
        success = True
    print("Navigation %s with time %.4f (s)" %(status, curr_time - start_time))
    
    if args.world_idx >= 300:  # DynaBARN environment which does not have a planned path
        path_length = GOAL_POSITION[0] - INIT_POSITION[0]
    else:
        path_file_name = join(base_path, "worlds/BARN/path_files", "path_%d.npy" %args.world_idx)
        path_array = np.load(path_file_name)
        path_array = [path_coord_to_gazebo_coord(*p) for p in path_array]
        path_array = np.insert(path_array, 0, (INIT_POSITION[0], INIT_POSITION[1]), axis=0)
        path_array = np.insert(path_array, len(path_array), (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1]), axis=0)
        path_length = 0
        for p1, p2 in zip(path_array[:-1], path_array[1:]):
            path_length += compute_distance(p1, p2)
    
    # Navigation metric: 1_success *  optimal_time / clip(actual_time, 2 * optimal_time, 8 * optimal_time)
    optimal_time = path_length / 2
    actual_time = curr_time - start_time
    nav_metric = int(success) * optimal_time / np.clip(actual_time, 2 * optimal_time, 8 * optimal_time)
    print("Navigation metric: %.4f" %(nav_metric))
    print()

    if args.monitor and cpu_samples:
        def stats(arr):
            return {
                "avg": float(np.mean(arr)),
                "max": float(np.max(arr)),
                "min": float(np.min(arr)),
                "std": float(np.std(arr)),
            }

        cpu_stats = stats(cpu_samples)
        mem_stats = stats(mem_samples)

        print("\n>>>>>>>>>>>> Resource Usage Statistics <<<<<<<<<<<<<")
        print(f"Node monitoring: {args.mnode}")
        print(f"Samples collected: {len(cpu_samples)}")
        print("CPU Usage (%):")
        print("  Avg: %.2f | Max: %.2f | Min: %.2f | Std: %.2f"
             % (cpu_stats["avg"], cpu_stats["max"],
                 cpu_stats["min"], cpu_stats["std"]))

        print("Memory Usage (MB):")
        print("  Avg: %.3f | Max: %.3f | Min: %.3f | Std: %.3f"
              % (mem_stats["avg"], mem_stats["max"],
                 mem_stats["min"], mem_stats["std"]))
        print()
        with open(args.out, "a") as f:
            f.write("%d %d %d %d %.4f %.4f %.2f %.2f %.2f %.2f %.3f %.3f %.3f %.3f\n" 
                    %(args.world_idx, success, collided, 
                    (curr_time - start_time)>=100, 
                    curr_time - start_time, nav_metric, 
                    cpu_stats["avg"], cpu_stats["max"], 
                    cpu_stats["min"], cpu_stats["std"], 
                    mem_stats["avg"], mem_stats["max"], 
                    mem_stats["min"], mem_stats["std"]))
    
    else:
        if args.monitor:
            print("No samples during resource monitoring, skip resource statistic logs")
    
        with open(args.out, "a") as f:
            f.write("%d %d %d %d %.4f %.4f\n" %(args.world_idx, success, collided, (curr_time - start_time)>=100, curr_time - start_time, nav_metric))
   
    terminate_ros_proc()
