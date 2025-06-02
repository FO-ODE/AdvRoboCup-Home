import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

class GraspExecutor:
    def __init__(self):
        rospy.init_node("item_grasp_py")
        moveit_commander.roscpp_initialize([])

        self.goal_pose = geometry_msgs.msg.PoseStamped()
        self.attachObject = geometry_msgs.msg.PoseStamped()
        self.liftObject = geometry_msgs.msg.PoseStamped()

        self.gripper_pub = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)
        rospy.Subscriber("/cluster_centroid", PointStamped, self.pose_callback)

        self.group = moveit_commander.MoveGroupCommander("arm_torso")
        self.group.set_planner_id("RRTConnectkConfigDefault")
        self.group.set_pose_reference_frame("base_footprint")
        self.group.set_end_effector_link("gripper_link")
        self.group.set_max_velocity_scaling_factor(1.0)
        self.group.set_planning_time(5.0)

    def pose_callback(self, msg: PointStamped):
        self.goal_pose.header.frame_id = msg.header.frame_id
        self.goal_pose.pose.position.x = msg.point.x - 0.3
        self.goal_pose.pose.position.y = msg.point.y
        self.goal_pose.pose.position.z = msg.point.z
        self.goal_pose.pose.orientation.x = 0.5
        self.goal_pose.pose.orientation.y = 0.5
        self.goal_pose.pose.orientation.z = -0.5
        self.goal_pose.pose.orientation.w = -0.5

        self.attachObject.header.frame_id = msg.header.frame_id
        self.attachObject.pose.position.x = msg.point.x - 0.1
        self.attachObject.pose.position.y = msg.point.y
        self.attachObject.pose.position.z = msg.point.z
        self.attachObject.pose.orientation = self.goal_pose.pose.orientation

        self.liftObject.header.frame_id = msg.header.frame_id
        self.liftObject.pose.position.x = msg.point.x
        self.liftObject.pose.position.y = msg.point.y
        self.liftObject.pose.position.z = msg.point.z + 0.3
        self.liftObject.pose.orientation = self.goal_pose.pose.orientation

    def create_gripper_trajectory(self, position):
        traj = JointTrajectory()
        traj.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start = rospy.Duration(1)
        traj.points.append(point)
        return traj

    def move_arm(self, target_pose):
        self.group.set_pose_target(target_pose)
        self.group.set_start_state_to_current_state()

        rospy.loginfo("Planning motion to target pose...")
        plan = self.group.plan()
        if not plan or not plan[0]:
            raise RuntimeError("No plan found")

        rospy.loginfo("Executing motion...")
        result = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        return result
    
    def tuck_arm(self):
        rospy.loginfo("Waiting for play_motion action server...")
        client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
        if not client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("play_motion action server not available!")
            return False

        rospy.loginfo("Waiting for joint_states to stabilize...")
        rospy.wait_for_message("joint_states", JointState)
        # rospy.sleep(1.0)

        goal = PlayMotionGoal()
        goal.motion_name = "home"
        goal.skip_planning = False

        client.send_goal(goal)
        finished = client.wait_for_result(rospy.Duration(10.0))

        if not finished:
            rospy.logwarn("Tuck arm action did not finish before timeout.")
            return False

        rospy.loginfo("Arm successfully tucked.")
        return True


    def execute(self):
        rospy.sleep(3.0) # waiting for initialization, must have
        open_gripper = self.create_gripper_trajectory(0.045)
        close_gripper = self.create_gripper_trajectory(0.0)

        rospy.loginfo("Opening gripper")
        self.gripper_pub.publish(open_gripper)
        rospy.sleep(1.0)

        rospy.loginfo("Preparing grasping position")
        self.move_arm(self.goal_pose)
        rospy.sleep(1.0)

        rospy.loginfo("Approaching object")
        self.move_arm(self.attachObject)
        rospy.sleep(1.0)

        rospy.loginfo(f"Grasp Pose: {self.goal_pose}")
        rospy.loginfo(f"Attach Pose: {self.attachObject}")

        rospy.loginfo("Closing gripper")
        self.gripper_pub.publish(close_gripper)
        rospy.sleep(1.0)

        rospy.loginfo("Lifting object")
        self.move_arm(self.liftObject)
        rospy.sleep(1.0)

        rospy.loginfo("Tuck arm")
        self.tuck_arm()
        rospy.sleep(1.0)

        rospy.loginfo("Grasp sequence finished")
        rospy.sleep(1.0)

if __name__ == "__main__":
    executor = GraspExecutor()
    executor.execute()
