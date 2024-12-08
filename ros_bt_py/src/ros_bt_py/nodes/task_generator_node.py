import rospy
from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.node import Node, define_bt_node
from ros_bt_py.node_config import NodeConfig
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import subprocess



# This is a TEST NODE
@define_bt_node(NodeConfig(
    version="1.0.0",
    options={'robot_name': str},
    inputs={},
    outputs={'task': str, 'robot': str},
    max_children=4))
class GenerateSuccessNode(Node):
    """Generates the initial task that the pipeline will execute."""
    def _do_setup(self):
        pass

    def _do_tick(self):
        # Set the initial task. This could be something like "success".
        self.outputs['robot'] = self.options['robot_name'] if self.options['robot_name'] is not None else "None"
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
        previous_task = self.inputs['task'] if self.inputs['task'] is not None else None
        if previous_task != "success":
            return NodeMsg.FAILED
        
        joint_positions = self.options['joint_positions'] if self.options['joint_positions'] is not None else None
       
        rospy.loginfo("JointsPositionNode: Executing joints_position command using subprocess")
        try:
            # Base command
            command = ['rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot'], 'joints_position']
            
            # Add joint positions if specified
            if joint_positions and len(joint_positions) == 6:
                command.extend(map(str, joint_positions))
                rospy.loginfo(f"JointsPositionNode: Using joint positions: {command}")
            else:
                rospy.loginfo(f"JointsPositionNode: Using default joint positions: {command}")

            # Run the command
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True
            )
            rospy.loginfo(f"JointsPositionNode: Command executed successfully:\n{result.stdout}")
            self.outputs['task'] = "success"
            self.outputs['robot'] = self.inputs['robot']
            return NodeMsg.SUCCEEDED
        except Exception as e:
            rospy.logerr(f"JointsPositionNode: Command failed with error:\n{e}")
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

        previous_task = self.inputs['task'] if self.inputs['task'] is not None else None
        if previous_task != "success":
            return NodeMsg.FAILED
        
        end_target = self.options['end_target'] if self.options['end_target'] is not None else None
        
        try:
            command = ['rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot'], 'end_coordinate', end_target]
            rospy.loginfo(f"EndCoordinateNode: Executing end_coordinate: '{command}'")
            
            # Construct and run the rosrun command dynamically
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True
            )
            rospy.loginfo(f"EndCoordinateNode: Command executed successfully:\n{result.stdout}")
            self.outputs['task'] = "success"
            self.outputs['robot'] = self.inputs['robot']
            return NodeMsg.SUCCEEDED
        except Exception as e:
            rospy.logerr(f"EndCoordinateNode: Command failed with error:\n{e}")
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
        
        previous_task = self.inputs['task'] if self.inputs['task'] is not None else None
        if previous_task != "success":
            return NodeMsg.FAILED
        
        mode =            self.options['mode']              if self.options['mode'] is not None else None
        object_name =     self.options['object_name']       if self.options['object_name'] is not None else None # ""
        reference_frame = self.options['reference_frame']   if self.options['reference_frame'] is not None else None # 'world' 
        pose =            self.options['pose']              if self.options['pose'] is not None else None # [0, 0, 0]

        try:
            if mode == "spawn_object":
                return self._spawn_object(object_name, reference_frame, pose)
            elif mode == "attach_object":
                return self._attach_object(object_name, reference_frame)
            elif mode == "detach_object":
                return self._detach_object(object_name, reference_frame)
            elif mode == "remove_object":
                return self._remove_object(object_name)
            else:
                rospy.logwarn(f"ObjectManipulationNode: Unknown task '{mode}'")
                return NodeMsg.FAILED
        except Exception as e:
            rospy.logerr(f"ObjectManipulationNode: Error during task '{mode}': {e}")
            return NodeMsg.FAILED

    def _spawn_object(self, object_name, reference_frame, pose):
        if not object_name:
            rospy.logwarn("ObjectManipulationNode: No object name provided for spawn_object task")
            return NodeMsg.FAILED

        try:
            rospy.loginfo(f"Spawning object '{object_name}' at pose {pose} in frame '{reference_frame}'")
            command = [
                'rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot'],
                'spawn_object', object_name, str(pose[0]), str(pose[1]), str(pose[2])
            ]
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
            rospy.loginfo(f"ObjectManipulationNode: Spawn object command executed successfully:\n{result.stdout}")
            self.outputs['task'] = "success"
            self.outputs['robot'] = self.inputs['robot']
            return NodeMsg.SUCCEEDED
        except Exception as e:
            rospy.logerr(f"ObjectManipulationNode: Failed to spawn object:\n{e}")
            return NodeMsg.FAILED

    def _attach_object(self, object_name, reference_frame):
        if not object_name or not reference_frame:
            rospy.logwarn("ObjectManipulationNode: Missing object name or reference frame for attach_object task")
            return NodeMsg.FAILED

        try:
            rospy.loginfo(f"Attaching object '{object_name}' to frame '{reference_frame}'")
            command = [
                'rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot'],
                'attach_object', object_name, reference_frame
            ]
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
            rospy.loginfo(f"ObjectManipulationNode: Attach object command executed successfully:\n{result.stdout}")
            self.outputs['task'] = "success"
            self.outputs['robot'] = self.inputs['robot']
            return NodeMsg.SUCCEEDED
        except Exception as e:
            rospy.logerr(f"ObjectManipulationNode: Failed to attach object:\n{e}")
            return NodeMsg.FAILED

    def _detach_object(self, object_name, reference_frame):
        if not object_name or not reference_frame:
            rospy.logwarn("ObjectManipulationNode: Missing object name or reference frame for detach_object task")
            return NodeMsg.FAILED

        try:
            rospy.loginfo(f"Detaching object '{object_name}' from frame '{reference_frame}'")
            command = [
                'rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot'],
                'detach_object', object_name, reference_frame
            ]
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
            rospy.loginfo(f"ObjectManipulationNode: Detach object command executed successfully:\n{result.stdout}")
            self.outputs['task'] = "success"
            self.outputs['robot'] = self.inputs['robot']
            return NodeMsg.SUCCEEDED
        except Exception as e:
            rospy.logerr(f"ObjectManipulationNode: Failed to detach object:\n{e}")
            return NodeMsg.FAILED

    def _remove_object(self, object_name):
        if not object_name:
            rospy.logwarn("ObjectManipulationNode: No object name provided for remove_object task")
            return NodeMsg.FAILED

        try:
            rospy.loginfo(f"Removing object '{object_name}'")
            command = [
                'rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot'], 'remove_object', object_name
            ]
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
            rospy.loginfo(f"ObjectManipulationNode: Remove object command executed successfully:\n{result.stdout}")
            self.outputs['task'] = "success"
            self.outputs['robot'] = self.inputs['robot']
            return NodeMsg.SUCCEEDED
        except Exception as e:
            rospy.logerr(f"ObjectManipulationNode: Failed to remove object:\n{e}")
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
        
        previous_task = self.inputs['task'] if self.inputs['task'] is not None else None
        if previous_task != "success":
            return NodeMsg.FAILED
        
        mode = self.options['mode'] if self.options['mode'] is not None else None
        
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
            result = subprocess.run(
                ['rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot'], command],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True
            )
            rospy.loginfo(f"GripperControlNode: Command '{command}' executed successfully:\n{result.stdout}")
            self.outputs['task'] = "success"
            self.outputs['robot'] = self.inputs['robot']
            return NodeMsg.SUCCEEDED
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
        
        previous_task = self.inputs['task'] if self.inputs['task'] is not None else None
        if previous_task != "success":
            return NodeMsg.FAILED
        
        pipeline_name = self.options['pipeline_name'] if self.options['pipeline_name'] is not None else None
        planner_name = self.options['planner_name'] if self.options['planner_name'] is not None else None

        return self._execute_pipeline_command(pipeline_name, planner_name)


    def _execute_pipeline_command(self, pipeline_name, planner_name):
        if not pipeline_name or not planner_name:
            rospy.logwarn("MotionPipelineNode: Missing pipeline or planner name for choose_pipeline task")
            return NodeMsg.FAILED

        try:
            rospy.loginfo(f"MotionPipelineNode: Configuring pipeline '{pipeline_name}' with planner '{planner_name}'")
            # Construct and execute the subprocess command
            command = [
                'rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot'],
                'choose_pipeline', pipeline_name, planner_name
            ]
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
            rospy.loginfo(f"MotionPipelineNode: Command executed successfully:\n{result.stdout}")
            self.outputs['task'] = "success"
            self.outputs['robot'] = self.inputs['robot']
            return NodeMsg.SUCCEEDED
        except Exception as e:
            rospy.logerr(f"MotionPipelineNode: Failed to configure pipeline:\n{e}")
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
        
        previous_task = self.inputs['task'] if self.inputs['task'] is not None else None
        if previous_task != "success":
            return NodeMsg.FAILED
        
        file_name = self.options['file_name'] if self.options['file_name'] is not None else None
        mode = self.options['mode'] if self.options['mode'] is not None else None
        
        if mode == "check_json_files":
            return self._execute_command(['check_json_files'])
        elif mode == "delete_json_sim_content":
            if file_name:
                return self._execute_command(['delete_json_sim_content', file_name])
            else:
                rospy.logwarn("JsonFileManagerNode: No file name provided for delete_json_sim_content task")
                return NodeMsg.FAILED
        elif mode == "delete_json_temp":
            return self._execute_command(['delete_json_temp'])
        else:
            rospy.logwarn(f"JsonFileManagerNode: Unknown mode '{mode}'")
            return NodeMsg.FAILED

    def _execute_command(self, args):
        try:
            # Construct the rosrun command dynamically
            command = ['rosrun', 'moveit_python', 'task_generator.py', self.inputs['robot']] + args
            rospy.loginfo(f"JsonFileManagerNode: Executing command: {' '.join(command)}")
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True
            )
            rospy.loginfo(f"JsonFileManagerNode: Command executed successfully:\n{result.stdout}")
            self.outputs['task'] = "success"
            self.outputs['robot'] = self.inputs['robot']
            return NodeMsg.SUCCEEDED
        except Exception as e:
            rospy.logerr(f"JsonFileManagerNode: Command failed with error:\n{e}")
            return NodeMsg.FAILED

    def _do_shutdown(self):
        rospy.loginfo("JsonFileManagerNode: Shutting down")

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE