import rospy
from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.node import Node, define_bt_node
from ros_bt_py.node_config import NodeConfig
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import subprocess

def execute_subprocess(command, log_context=""):
    """
    Executes a subprocess command with standardized logging and error handling.
    
    Args:
        command (list): The command to run as a list of strings.
        log_context (str): A context string for better log messages.
    
    Returns:
        tuple: (success (bool), output (str), error (str))
    """
    try:
        rospy.loginfo(f"{log_context}: Executing command: {' '.join(command)}")
        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True
        )
        rospy.loginfo(f"{log_context}: Command executed successfully:\n{result.stdout}")
        return True, result.stdout, ""
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"{log_context}: Command failed with error:\n{e.stderr}")
        return False, "", e.stderr
    except Exception as e:
        rospy.logerr(f"{log_context}: Unexpected error occurred:\n{e}")
        return False, "", str(e)


# This is a TEST NODE
@define_bt_node(NodeConfig(
    version="1.0.0",
    options={'robot_name': str},
    inputs={},
    outputs={'task': str, 'robot': str},
    max_children=4))
class TaskStartNode(Node):
    """Generates the initial task that the pipeline will execute."""
    def _do_setup(self):
        pass

    def _do_tick(self):
        # Set the initial task. This could be something like "success".
        if self.options['robot_name'] in ("", "foo") or self.options['robot_name'] is None:
            rospy.logerr(f"TaskStartNode: robot_name need to be renamed")
            return NodeMsg.FAILED
        
        self.outputs['robot'] = self.options['robot_name']
        self.outputs['task'] = "success"
        rospy.loginfo("TaskGeneratorNode: Starting pipeline with 'success' task")
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE



# This is a TEST NODE
@define_bt_node(NodeConfig(
    version="1.0.0",
    options={},
    inputs={'task': str},
    outputs={'task': str},
    max_children=0))
class PublishCmdVelNode(Node):
    """Publishes cmd_vel for 10 seconds upon receiving the 'success' task."""
    def _do_setup(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.start_time = None
        self.running = False

    def _do_tick(self):
        # previous_task = self.inputs['task']
        previous_task = "success"
        if previous_task == "success":
            if not self.running:
                # Start publishing cmd_vel now and run for 10 seconds
                self.running = True
                self.start_time = time.time()
                rospy.loginfo("PublishCmdVelNode: Received 'success', starting 10s publish")

            elapsed = time.time() - self.start_time
            if elapsed < 5.0:
                # Publish cmd_vel message
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.5
                cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel)
                return NodeMsg.RUNNING
            else:
                # Done publishing for 5 seconds
                self.running = False
                # Set the next task in the pipeline
                self.outputs['task'] = "success"
                rospy.loginfo("PublishCmdVelNode: Finished publishing, next task 'success'")
                return NodeMsg.SUCCEEDED
        else:
            # If we don't have the right input task, we're idle
            return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.running = False
        self.start_time = None
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE



@define_bt_node(NodeConfig(
    version="1.0.0",
    options={'joint_positions': list},  # Option for specifying joint positions
    inputs={'task': str, 'robot': str},
    outputs={'task': str, 'robot': str},
    max_children=0))
class JointsPositionNode(Node):
    """Handles joints_position commands with multiple options using subprocess."""
    def _do_setup(self):
        rospy.loginfo("JointsPositionNode: Setup complete")

    def _do_tick(self):
        previous_task = self.inputs['task']
        if previous_task != "success":
            rospy.logerr("JointsPositionNode: Previous task not successful")
            return NodeMsg.FAILED

        if self.inputs['robot'] in ("", "foo") or self.inputs['robot'] is None:
            rospy.logerr("JointsPositionNode: Invalid robot input")
            return NodeMsg.FAILED

        joint_positions = self.options.get('joint_positions', None)
        command = ['rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot'], 'joints_position']
        if joint_positions and len(joint_positions) == 6:
            command.extend(map(str, joint_positions))

        success, output, error = execute_subprocess(command, log_context="JointsPositionNode")
        if success:
            self.outputs['task'] = "success"
            self.outputs['robot'] = self.inputs['robot']
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE



@define_bt_node(NodeConfig(
    version="1.0.0",
    options={'end_target': str},  # Option for specifying the end coordinate
    inputs={'task': str, 'robot': str},
    outputs={'task': str, 'robot': str},
    max_children=0))
class EndCoordinateNode(Node):
    """Handles end_coordinate commands dynamically using subprocess."""
    def _do_setup(self):
        rospy.loginfo("EndCoordinateNode: Setup complete")


    def _do_tick(self):
        previous_task = self.inputs['task']
        if previous_task != "success":
            rospy.logerr("EndCoordinateNode: Previous task not successful")
            return NodeMsg.FAILED

        if self.inputs['robot'] in ("", "foo") or self.inputs['robot'] is None:
            rospy.logerr("EndCoordinateNode: Invalid robot input")
            return NodeMsg.FAILED

        end_target = self.options.get('end_target', None)
        if not end_target:
            rospy.logerr("EndCoordinateNode: No end_target specified")
            return NodeMsg.FAILED

        command = ['rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot'], 'end_coordinate', end_target]
        success, output, error = execute_subprocess(command, log_context="EndCoordinateNode")
        
        if success:
            self.outputs['task'] = "success"
            self.outputs['robot'] = self.inputs['robot']
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE



@define_bt_node(NodeConfig(
    version="1.0.0",
    options={
        'mode': str,
        'object_name': str,
        'reference_frame': str,
        'pose': list
    },
    inputs={'task': str, 'robot': str},
    outputs={'task': str, 'robot': str},
    max_children=0))
class ObjectManipulationNode(Node):
    """Handles spawn_object, attach_object, detach_object, and remove_object tasks."""
    def _do_setup(self):
        rospy.loginfo("ObjectManipulationNode: Setup complete")

    def _do_tick(self):
        previous_task = self.inputs['task']
        if previous_task != "success":
            rospy.logerr("ObjectManipulationNode: Previous task not successful")
            return NodeMsg.FAILED

        if self.inputs['robot'] in ("", "foo") or self.inputs['robot'] is None:
            rospy.logerr("ObjectManipulationNode: Invalid robot input")
            return NodeMsg.FAILED

        mode = self.options.get('mode', None)
        object_name = self.options.get('object_name', None)
        reference_frame = self.options.get('reference_frame', None)
        pose = self.options.get('pose', None)

        if mode == "spawn_object":
            return self._spawn_object(object_name, reference_frame, pose)
        elif mode == "attach_object":
            return self._attach_object(object_name, reference_frame)
        elif mode == "detach_object":
            return self._detach_object(object_name, reference_frame)
        elif mode == "remove_object":
            return self._remove_object(object_name)
        elif mode == "clear_scene":
            return self._clear_scene()
        else:
            rospy.logwarn(f"ObjectManipulationNode: Unknown task '{mode}'")
            return NodeMsg.FAILED

    def _spawn_object(self, object_name, reference_frame, pose):
        if not object_name or not pose or len(pose) < 3:
            rospy.logwarn("ObjectManipulationNode: Missing parameters for spawn_object task")
            return NodeMsg.FAILED

        command = [
            'rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot'],
            'spawn_object', object_name, str(pose[0]), str(pose[1]), str(pose[2])
        ]
        success, output, error = execute_subprocess(command, log_context="ObjectManipulationNode: spawn_object")
        
        if success:
            self.outputs['task'] = "success"
            self.outputs['robot'] = self.inputs['robot']
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _attach_object(self, object_name, reference_frame):
        if not object_name or not reference_frame:
            rospy.logwarn("ObjectManipulationNode: Missing parameters for attach_object task")
            return NodeMsg.FAILED

        command = [
            'rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot'],
            'attach_object', object_name, reference_frame
        ]
        success, output, error = execute_subprocess(command, log_context="ObjectManipulationNode: attach_object")
        
        if success:
            self.outputs['task'] = "success"
            self.outputs['robot'] = self.inputs['robot']
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _detach_object(self, object_name, reference_frame):
        if not object_name or not reference_frame:
            rospy.logwarn("ObjectManipulationNode: Missing parameters for detach_object task")
            return NodeMsg.FAILED

        command = [
            'rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot'],
            'detach_object', object_name, reference_frame
        ]
        success, output, error = execute_subprocess(command, log_context="ObjectManipulationNode: detach_object")
        
        if success:
            self.outputs['task'] = "success"
            self.outputs['robot'] = self.inputs['robot']
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _remove_object(self, object_name):
        if not object_name:
            rospy.logwarn("ObjectManipulationNode: Missing object name for remove_object task")
            return NodeMsg.FAILED

        command = [
            'rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot'], 'remove_object', object_name
        ]
        success, output, error = execute_subprocess(command, log_context="ObjectManipulationNode: remove_object")
        
        if success:
            self.outputs['task'] = "success"
            self.outputs['robot'] = self.inputs['robot']
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _remove_object(self):

        command = ['rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot'], 'clear_scene']
        success, output, error = execute_subprocess(command, log_context="ObjectManipulationNode: clear_scene")
        
        if success:
            self.outputs['task'] = "success"
            self.outputs['robot'] = self.inputs['robot']
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.FAILED

    def _do_shutdown(self):
        rospy.loginfo("ObjectManipulationNode: Shutting down")

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE



@define_bt_node(NodeConfig(
    version="1.0.0",
    options={'mode': str},
    inputs={'task': str, 'robot': str},
    outputs={'task': str, 'robot': str},
    max_children=0))
class GripperControlNode(Node):
    """Handles gripper_open and gripper_close tasks using subprocess."""
    def _do_setup(self):
        rospy.loginfo("GripperControlNode: Ready to receive tasks")

    def _do_tick(self):
        previous_task = self.inputs['task']
        if previous_task != "success":
            rospy.logerr("GripperControlNode: Previous task not successful")
            return NodeMsg.FAILED

        if self.inputs['robot'] in ("", "foo") or self.inputs['robot'] is None:
            rospy.logerr("GripperControlNode: Invalid robot input")
            return NodeMsg.FAILED

        mode = self.options.get('mode', None)
        
        if mode == "gripper_open":
            return self._execute_gripper_command('gripper_open')
        elif mode == "gripper_close":
            return self._execute_gripper_command('gripper_close')
        else:
            rospy.logwarn(f"GripperControlNode: Unknown task '{mode}'")
            return NodeMsg.FAILED

    def _execute_gripper_command(self, command):
        try:
            rospy.loginfo(f"GripperControlNode: Executing '{command}' command")
            success, output, error = execute_subprocess(
                ['rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot'], command],
                log_context=f"GripperControlNode: {command}"
            )
            if success:
                rospy.loginfo(f"GripperControlNode: Command '{command}' executed successfully:\n{output}")
                self.outputs['task'] = "success"
                self.outputs['robot'] = self.inputs['robot']
                return NodeMsg.SUCCEEDED
            else:
                rospy.logerr(f"GripperControlNode: Command '{command}' failed with error:\n{error}")
                return NodeMsg.FAILED
        except Exception as e:
            rospy.logerr(f"GripperControlNode: Command '{command}' failed with error:\n{e}")
            return NodeMsg.FAILED

    def _do_shutdown(self):
        rospy.loginfo("GripperControlNode: Shutting down")

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE



@define_bt_node(NodeConfig(
    version="1.0.0",
    options={
        'pipeline_name': str,  # Motion planning pipeline (e.g., OMPL, PILZ)
        'planner_name': str   # Specific planner (e.g., RRTConnect, LIN)
    },
    inputs={'task': str, 'robot': str},
    outputs={'task': str, 'robot': str},
    max_children=0))
class MotionPipelineNode(Node):
    """Handles choosing motion planning pipeline using subprocess."""
    def _do_setup(self):
        rospy.loginfo("MotionPipelineNode: Ready to configure motion planning pipeline")

    def _do_tick(self):
        previous_task = self.inputs['task']
        if previous_task != "success":
            rospy.logerr("MotionPipelineNode: Previous task not successful")
            return NodeMsg.FAILED

        if self.inputs['robot'] in ("", "foo") or self.inputs['robot'] is None:
            rospy.logerr("MotionPipelineNode: Invalid robot input")
            return NodeMsg.FAILED

        pipeline_name = self.options.get('pipeline_name', None)
        planner_name = self.options.get('planner_name', None)

        if not pipeline_name or not planner_name:
            rospy.logwarn("MotionPipelineNode: Missing pipeline or planner name for choose_pipeline task")
            return NodeMsg.FAILED

        try:
            rospy.loginfo(f"MotionPipelineNode: Configuring pipeline '{pipeline_name}' with planner '{planner_name}'")
            # Use the helper function to handle the subprocess
            success, output, error = execute_subprocess(
                ['rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot'], 'choose_pipeline', pipeline_name, planner_name],
                log_context="MotionPipelineNode: choose_pipeline"
            )

            if success:
                rospy.loginfo(f"MotionPipelineNode: Command executed successfully:\n{output}")
                self.outputs['task'] = "success"
                self.outputs['robot'] = self.inputs['robot']
                return NodeMsg.SUCCEEDED
            else:
                rospy.logerr(f"MotionPipelineNode: Command failed with error:\n{error}")
                return NodeMsg.FAILED
        except Exception as e:
            rospy.logerr(f"MotionPipelineNode: Error during pipeline configuration:\n{e}")
            return NodeMsg.FAILED


    def _do_shutdown(self):
        rospy.loginfo("MotionPipelineNode: Shutting down")

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE



@define_bt_node(NodeConfig(
    version="1.0.0",
    options={'mode': str, 'file_name': str},
    inputs={'task': str, 'robot': str},
    outputs={'task': str, 'robot': str},
    max_children=0))
class JsonFileManagerNode(Node):
    """Handles tasks for checking JSON files, deleting JSON simulation content, and deleting temporary JSON files."""
    def _do_setup(self):
        rospy.loginfo("JsonFileManagerNode: Ready to manage JSON files")

    def _do_tick(self):
        previous_task = self.inputs['task']
        if previous_task != "success":
            rospy.logerr("JsonFileManagerNode: Previous task not successful")
            return NodeMsg.FAILED

        if self.inputs['robot'] in ("", "foo") or self.inputs['robot'] is None:
            rospy.logerr("JsonFileManagerNode: Invalid robot input")
            return NodeMsg.FAILED

        file_name = self.options.get('file_name', None)
        mode = self.options.get('mode', None)

        if mode == "check_json_files":
            return self._execute_command(['check_json_files'], "JsonFileManagerNode: check_json_files")
        elif mode == "delete_json_sim_content":
            if file_name:
                return self._execute_command(['delete_json_sim_content', file_name], "JsonFileManagerNode: delete_json_sim_content")
            else:
                rospy.logwarn("JsonFileManagerNode: No file name provided for delete_json_sim_content task")
                return NodeMsg.FAILED
        elif mode == "delete_json_temp":
            return self._execute_command(['delete_json_temp'], "JsonFileManagerNode: delete_json_temp")
        else:
            rospy.logwarn(f"JsonFileManagerNode: Unknown mode '{mode}'")
            return NodeMsg.FAILED

    def _execute_command(self, args, log_context):
        command = ['rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot']] + args
        success, output, error = execute_subprocess(command, log_context=log_context)

        if success:
            self.outputs['task'] = "success"
            self.outputs['robot'] = self.inputs['robot']
            return NodeMsg.SUCCEEDED
        else:
            rospy.logerr(f"{log_context}: Command failed with error:\n{error}")
            return NodeMsg.FAILED

    def _do_shutdown(self):
        rospy.loginfo("JsonFileManagerNode: Shutting down")

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE