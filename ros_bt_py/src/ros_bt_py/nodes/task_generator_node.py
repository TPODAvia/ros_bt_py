import rospy
from ros_bt_py_msgs.msg import Node as NodeMsg
from ros_bt_py.node import Node, define_bt_node
from ros_bt_py.node_config import NodeConfig
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

@define_bt_node(NodeConfig(
    version="1.0.0",
    options={},
    inputs={},
    outputs={'task': str},
    max_children=4))
class TaskGeneratorNode(Node):
    """Generates the initial task that the pipeline will execute."""
    def _do_setup(self):
        pass

    def _do_tick(self):
        # Set the initial task. This could be something like "publish_cmd_vel".
        self.outputs['task'] = "publish_cmd_vel"
        rospy.loginfo("TaskGeneratorNode: Starting pipeline with 'publish_cmd_vel' task")
        return NodeMsg.SUCCEEDED

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
class PublishCmdVelNode(Node):
    """Publishes cmd_vel for 10 seconds upon receiving the 'publish_cmd_vel' task."""
    def _do_setup(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.start_time = None
        self.running = False

    def _do_tick(self):
        current_task = self.inputs['task']
        if current_task == "publish_cmd_vel":
            if not self.running:
                # Start publishing cmd_vel now and run for 10 seconds
                self.running = True
                self.start_time = time.time()
                rospy.loginfo("PublishCmdVelNode: Received 'publish_cmd_vel', starting 10s publish")

            elapsed = time.time() - self.start_time
            if elapsed < 10.0:
                # Publish cmd_vel message
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.5
                cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel)
                return NodeMsg.RUNNING
            else:
                # Done publishing for 10 seconds
                self.running = False
                # Set the next task in the pipeline
                self.outputs['task'] = "open_gripper"
                rospy.loginfo("PublishCmdVelNode: Finished publishing, next task 'open_gripper'")
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
class OpenGripperNode(Node):
    """Opens the gripper upon receiving 'open_gripper' and then sets the next task."""
    def _do_setup(self):
        self.gripper_pub = rospy.Publisher('/gripper_cmd', String, queue_size=1)

    def _do_tick(self):
        current_task = self.inputs['task']
        if current_task == "open_gripper":
            # Send open command
            self.gripper_pub.publish("open")
            rospy.loginfo("OpenGripperNode: Gripper opened, next task 'close_gripper'")
            # Set next task
            self.outputs['task'] = "close_gripper"
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
class CloseGripperNode(Node):
    """Closes the gripper upon receiving 'close_gripper' and then finishes."""
    def _do_setup(self):
        self.gripper_pub = rospy.Publisher('/gripper_cmd', String, queue_size=1)

    def _do_tick(self):
        current_task = self.inputs['task']
        if current_task == "close_gripper":
            # Send close command
            self.gripper_pub.publish("close")
            rospy.loginfo("CloseGripperNode: Gripper closed, pipeline complete")
            # No further tasks, so set empty or None
            self.outputs['task'] = ""
            return NodeMsg.SUCCEEDED
        else:
            return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
