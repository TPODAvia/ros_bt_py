from ros_bt_py.node import Leaf
from ros_bt_py.node_config import define_bt_node, NodeConfig
from ros_bt_py_msgs.msg import Node as NodeMsg
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
import rospy

@define_bt_node(
    NodeConfig(
        version='1.0.0',
        options={
            'task': str,  # The task to execute
            'parameters': list  # Parameters for the task
        },
        inputs={},
        outputs={
            'result': bool,  # Success or failure of the task
            'message': str   # Optional message or error
        },
        max_children=0
    )
)
class TaskGeneratorNode(Leaf):
    def __init__(self, options=None, debug=False):
        super(TaskGeneratorNode, self).__init__(options=options, debug=debug)
        self.move_group = None
        self.planning_scene = None
        self.robot = None

    def setup(self):
        # Initialize MoveIt interfaces
        self.move_group = MoveGroupInterface("manipulator", "base_link")
        self.planning_scene = PlanningSceneInterface("base_link")
        self.robot = self.move_group._robot

        self.outputs['result'] = False
        self.outputs['message'] = ''
        self.logger.info("TaskGeneratorNode setup complete")

    def tick(self):
        task = self.options['task']
        params = self.options.get('parameters', [])

        try:
            if task == 'get_robot_param':
                self.get_robot_param()
            elif task == 'joints_position':
                self.joints_position(params)
            elif task == 'end_coordinate':
                self.end_coordinate(params)
            elif task == 'spawn_object':
                self.spawn_object(params)
            elif task == 'attach_object':
                self.attach_object(params)
            elif task == 'detach_object':
                self.detach_object(params)
            elif task == 'remove_object':
                self.remove_object(params)
            elif task == 'clear_scene':
                self.clear_scene()
            elif task == 'gripper_open':
                self.gripper_open()
            elif task == 'gripper_close':
                self.gripper_close()
            else:
                self.outputs['message'] = f"Unknown task: {task}"
                self.logger.error(self.outputs['message'])
                return NodeMsg.FAILED

            self.outputs['result'] = True
            return NodeMsg.SUCCEEDED

        except Exception as e:
            self.outputs['result'] = False
            self.outputs['message'] = str(e)
            self.logger.error(f"Task failed: {e}")
            return NodeMsg.FAILED

    def shutdown(self):
        self.logger.info("Shutting down TaskGeneratorNode")

    # Task Implementations
    def get_robot_param(self):
        # Retrieve and log robot parameters
        params = self.robot.get_current_state()
        self.logger.info(f"Robot parameters: {params}")

    def joints_position(self, joint_values):
        # Move robot to specified joint positions
        joint_names = self.move_group.get_joints()
        if len(joint_values) != len(joint_names):
            raise ValueError("Joint values length does not match joint names length")

        self.move_group.moveToJointPosition(joint_names, joint_values, wait=True)
        result = self.move_group.get_move_action().get_result()
        if result.error_code.val != result.error_code.SUCCESS:
            raise Exception(f"Failed to move to joint positions: {result}")

    def end_coordinate(self, params):
        # Move robot end effector to specified coordinates
        if len(params) < 7:
            raise ValueError("End coordinate requires 7 parameters: x, y, z, qx, qy, qz, qw")

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = float(params[0])
        pose.pose.position.y = float(params[1])
        pose.pose.position.z = float(params[2])
        pose.pose.orientation.x = float(params[3])
        pose.pose.orientation.y = float(params[4])
        pose.pose.orientation.z = float(params[5])
        pose.pose.orientation.w = float(params[6])

        self.move_group.moveToPose(pose, "tool0")
        result = self.move_group.get_move_action().get_result()
        if result.error_code.val != result.error_code.SUCCESS:
            raise Exception(f"Failed to move to end coordinate: {result}")

    def spawn_object(self, params):
        # Spawn an object into the planning scene
        if len(params) < 4:
            raise ValueError("Spawn object requires at least 4 parameters: name, x, y, z")
        name = params[0]
        x = float(params[1])
        y = float(params[2])
        z = float(params[3])
        # Optionally accept orientation parameters
        if len(params) >= 7:
            qx = float(params[4])
            qy = float(params[5])
            qz = float(params[6])
            qw = float(params[7])
        else:
            qx = qy = qz = 0.0
            qw = 1.0

        self.planning_scene.addBox(name, 0.05, 0.05, 0.05, x, y, z, qx, qy, qz, qw)
        self.logger.info(f"Spawned object '{name}' at ({x}, {y}, {z})")

    def attach_object(self, params):
        # Attach an object to the robot
        if len(params) < 1:
            raise ValueError("Attach object requires at least 1 parameter: object name")
        name = params[0]
        self.move_group.attachObject(name, "tool0")
        self.logger.info(f"Attached object '{name}' to 'tool0'")

    def detach_object(self, params):
        # Detach an object from the robot
        if len(params) < 1:
            raise ValueError("Detach object requires at least 1 parameter: object name")
        name = params[0]
        self.move_group.detachObject(name)
        self.logger.info(f"Detached object '{name}' from 'tool0'")

    def remove_object(self, params):
        # Remove an object from the planning scene
        if len(params) < 1:
            raise ValueError("Remove object requires at least 1 parameter: object name")
        name = params[0]
        self.planning_scene.removeCollisionObject(name)
        self.logger.info(f"Removed object '{name}' from the planning scene")

    def clear_scene(self):
        # Clear the planning scene
        self.planning_scene.clear()
        self.logger.info("Cleared the planning scene")

    def gripper_open(self):
        # Open the gripper (implementation depends on your robot)
        self.logger.info("Opening gripper")
        # Add your gripper control code here

    def gripper_close(self):
        # Close the gripper (implementation depends on your robot)
        self.logger.info("Closing gripper")
        # Add your gripper control code here

    # Additional methods for other tasks can be implemented similarly
