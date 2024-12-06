import rospy
from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.node import Node, define_bt_node
from ros_bt_py.node_config import NodeConfig
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

# This is a TEST NODE
@define_bt_node(NodeConfig(
    version="1.0.0",
    options={},
    inputs={},
    outputs={'task': str},
    max_children=4))
class GenerateSuccessNode(Node):
    """Generates the initial task that the pipeline will execute."""
    def _do_setup(self):
        pass

    def _do_tick(self):
        # Set the initial task. This could be something like "success".
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
    options={},
    inputs={'task': str},
    outputs={'task': str},
    max_children=0))
class JointsPositionNode(Node):
    """Handles joints_position commands."""
    def _do_setup(self):
        rospy.loginfo("JointsPositionNode: Setup complete")

    def _do_tick(self):
        task = self.inputs['task']
        if task == "joints_position":
            rospy.loginfo("JointsPositionNode: Executing joints_position command")
            # Example logic for joint positions
            joint_positions = [0, 0, 0, 0, 0, 0]  # Default example
            rospy.loginfo(f"Setting joint positions to: {joint_positions}")
            # Update task
            self.outputs['task'] = "success"
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version="1.0.0",
    options={},
    inputs={'task': str},
    outputs={'task': str},
    max_children=0))
class EndCoordinateNode(Node):
    """Handles end_coordinate commands."""
    def _do_setup(self):
        rospy.loginfo("EndCoordinateNode: Setup complete")

    def _do_tick(self):
        task = self.inputs['task']
        if task == "end_coordinate":
            rospy.loginfo("EndCoordinateNode: Executing end_coordinate command")
            # Example logic for end coordinates
            end_target = "tf_end"  # Example target
            rospy.loginfo(f"Setting end coordinates to: {end_target}")
            # Update task
            self.outputs['task'] = "success"
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

@define_bt_node(NodeConfig(
    version="1.0.0",
    options={},
    inputs={'task': str},
    outputs={'task': str},
    max_children=0))
class EndCoordinateNode(Node):
    """Handles end_coordinate commands."""
    def _do_setup(self):
        rospy.loginfo("EndCoordinateNode: Setup complete")

    def _do_tick(self):
        task = self.inputs['task']
        if task == "end_coordinate":
            rospy.loginfo("EndCoordinateNode: Executing end_coordinate command")
            # Example logic for end coordinates
            end_target = "tf_end"  # Example target
            rospy.loginfo(f"Setting end coordinates to: {end_target}")
            # Update task
            self.outputs['task'] = "success"
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
    
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

@define_bt_node(NodeConfig(
    version="1.0.0",
    options={},
    inputs={'task': str, 'object_name': str, 'reference_frame': str, 'pose': list},
    outputs={'task': str},
    max_children=0))
class ObjectManipulationNode(Node):
    """Handles spawn_object, attach_object, detach_object, remove_object, and clear_scene tasks."""
    def _do_setup(self):
        self.scene = PlanningSceneInterface()
        rospy.sleep(2)  # Allow some time for the planning scene to initialize

    def _do_tick(self):
        task = self.inputs['task']
        object_name = self.inputs.get('object_name', '')
        reference_frame = self.inputs.get('reference_frame', 'world')
        pose = self.inputs.get('pose', [0, 0, 0])

        if task == "spawn_object":
            if object_name:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = reference_frame
                pose_stamped.pose.position.x = pose[0]
                pose_stamped.pose.position.y = pose[1]
                pose_stamped.pose.position.z = pose[2]
                pose_stamped.pose.orientation.w = 1.0  # Default orientation
                self.scene.add_box(object_name, pose_stamped, size=(0.1, 0.1, 0.1))
                rospy.loginfo(f"Spawned object '{object_name}' at {pose}")
                self.outputs['task'] = "success"
                return NodeMsg.SUCCEEDED
            else:
                rospy.logwarn("Spawn object failed: No object name provided")
                return NodeMsg.IDLE

        elif task == "attach_object":
            if object_name and reference_frame:
                self.scene.attach_box(reference_frame, object_name)
                rospy.loginfo(f"Attached object '{object_name}' to '{reference_frame}'")
                self.outputs['task'] = "success"
                return NodeMsg.SUCCEEDED
            else:
                rospy.logwarn("Attach object failed: Missing object name or reference frame")
                return NodeMsg.IDLE

        elif task == "detach_object":
            if object_name and reference_frame:
                self.scene.detach_box(reference_frame, object_name)
                rospy.loginfo(f"Detached object '{object_name}' from '{reference_frame}'")
                self.outputs['task'] = "success"
                return NodeMsg.SUCCEEDED
            else:
                rospy.logwarn("Detach object failed: Missing object name or reference frame")
                return NodeMsg.IDLE

        elif task == "remove_object":
            if object_name:
                self.scene.remove_world_object(object_name)
                rospy.loginfo(f"Removed object '{object_name}' from the scene")
                self.outputs['task'] = "success"
                return NodeMsg.SUCCEEDED
            else:
                rospy.logwarn("Remove object failed: No object name provided")
                return NodeMsg.IDLE

        elif task == "clear_scene":
            self.scene.remove_world_object()
            rospy.loginfo("Cleared all objects from the planning scene")
            self.outputs['task'] = "success"
            return NodeMsg.SUCCEEDED

        else:
            rospy.logwarn(f"Task '{task}' not recognized in ObjectManipulationNode")
            return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(NodeConfig(
    version="1.0.0",
    options={},
    inputs={'task': str},
    outputs={'task': str},
    max_children=0))
class GripperControlNode(Node):
    """Handles gripper_open and gripper_close tasks."""
    def _do_setup(self):
        self.gripper_pub = rospy.Publisher('/gripper_cmd', String, queue_size=1)
        rospy.sleep(1)  # Allow time for the publisher to initialize
        rospy.loginfo("GripperControlNode: Ready to receive tasks")

    def _do_tick(self):
        task = self.inputs['task']
        if task == "gripper_open":
            self.gripper_pub.publish("open")
            rospy.loginfo("GripperControlNode: Gripper opened")
            self.outputs['task'] = "success"
            return NodeMsg.SUCCEEDED
        elif task == "gripper_close":
            self.gripper_pub.publish("close")
            rospy.loginfo("GripperControlNode: Gripper closed")
            self.outputs['task'] = "success"
            return NodeMsg.SUCCEEDED
        else:
            rospy.logwarn(f"GripperControlNode: Unknown task '{task}'")
            return NodeMsg.IDLE

    def _do_shutdown(self):
        rospy.loginfo("GripperControlNode: Shutting down")

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

@define_bt_node(NodeConfig(
    version="1.0.0",
    options={},
    inputs={'task': str, 'pipeline_name': str, 'planner_name': str},
    outputs={'task': str},
    max_children=0))
class MotionPipelineNode(Node):
    """Handles choosing motion planning pipeline and follow mode."""
    def _do_setup(self):
        # Assume a service or topic to configure motion planning pipeline
        self.pipeline_pub = rospy.Publisher('/motion_pipeline_config', String, queue_size=1)
        self.follow_mode_pub = rospy.Publisher('/follow_mode', String, queue_size=1)
        rospy.sleep(1)  # Allow time for the publishers to initialize
        rospy.loginfo("MotionPipelineNode: Ready to configure motion planning pipeline and follow mode")

    def _do_tick(self):
        task = self.inputs['task']
        pipeline_name = self.inputs.get('pipeline_name', '')
        planner_name = self.inputs.get('planner_name', '')

        if task == "choose_pipeline":
            if pipeline_name and planner_name:
                # Send pipeline and planner configuration
                config_message = f"Pipeline: {pipeline_name}, Planner: {planner_name}"
                self.pipeline_pub.publish(config_message)
                rospy.loginfo(f"MotionPipelineNode: Configured motion pipeline with {config_message}")
                self.outputs['task'] = "success"
                return NodeMsg.SUCCEEDED
            else:
                rospy.logwarn("MotionPipelineNode: Missing pipeline or planner name for choose_pipeline task")
                return NodeMsg.IDLE

        elif task == "choose_follow_mode":
            # Set the robot to follow mode
            self.follow_mode_pub.publish("follow")
            rospy.loginfo("MotionPipelineNode: Configured robot to follow mode")
            self.outputs['task'] = "success"
            return NodeMsg.SUCCEEDED

        else:
            rospy.logwarn(f"MotionPipelineNode: Unknown task '{task}'")
            return NodeMsg.IDLE

    def _do_shutdown(self):
        rospy.loginfo("MotionPipelineNode: Shutting down")

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


import os
import json

@define_bt_node(NodeConfig(
    version="1.0.0",
    options={},
    inputs={'task': str, 'file_name': str},
    outputs={'task': str},
    max_children=0))
class JsonFileManagerNode(Node):
    """Handles tasks for checking JSON files, deleting JSON simulation content, and deleting temporary JSON files."""
    def _do_setup(self):
        rospy.loginfo("JsonFileManagerNode: Ready to manage JSON files")

    def _do_tick(self):
        task = self.inputs['task']
        file_name = self.inputs.get('file_name', '')

        if task == "check_json_files":
            return self._check_json_files()
        elif task == "delete_json_sim_content":
            return self._delete_json_sim_content(file_name)
        elif task == "delete_json_temp":
            return self._delete_json_temp()
        else:
            rospy.logwarn(f"JsonFileManagerNode: Unknown task '{task}'")
            return NodeMsg.IDLE

    def _check_json_files(self):
        json_files = [f for f in os.listdir('.') if f.endswith('.json')]
        if json_files:
            rospy.loginfo(f"JsonFileManagerNode: JSON files found: {json_files}")
            self.outputs['task'] = "success"
            return NodeMsg.SUCCEEDED
        else:
            rospy.loginfo("JsonFileManagerNode: No JSON files found")
            self.outputs['task'] = "no_files"
            return NodeMsg.IDLE

    def _delete_json_sim_content(self, file_name):
        if not file_name or not os.path.isfile(file_name):
            rospy.logwarn(f"JsonFileManagerNode: File '{file_name}' does not exist")
            return NodeMsg.IDLE

        try:
            with open(file_name, 'r+') as f:
                data = json.load(f)
                if isinstance(data, dict) and "simulation_content" in data:
                    data["simulation_content"] = None
                    f.seek(0)
                    f.truncate()
                    json.dump(data, f, indent=4)
                    rospy.loginfo(f"JsonFileManagerNode: Cleared simulation content in '{file_name}'")
                    self.outputs['task'] = "success"
                    return NodeMsg.SUCCEEDED
                else:
                    rospy.loginfo(f"JsonFileManagerNode: No 'simulation_content' key in '{file_name}'")
                    return NodeMsg.IDLE
        except Exception as e:
            rospy.logerr(f"JsonFileManagerNode: Error while processing '{file_name}': {e}")
            return NodeMsg.IDLE

    def _delete_json_temp(self):
        temp_files = [f for f in os.listdir('.') if f.endswith('.json') and "temp" in f]
        if not temp_files:
            rospy.loginfo("JsonFileManagerNode: No temporary JSON files found")
            return NodeMsg.IDLE

        for temp_file in temp_files:
            try:
                os.remove(temp_file)
                rospy.loginfo(f"JsonFileManagerNode: Deleted temporary file '{temp_file}'")
            except Exception as e:
                rospy.logerr(f"JsonFileManagerNode: Failed to delete '{temp_file}': {e}")
        self.outputs['task'] = "success"
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        rospy.loginfo("JsonFileManagerNode: Shutting down")

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
