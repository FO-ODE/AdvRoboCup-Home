#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

class ObjectInserter:
    """
    ObjectInserter is a ROS node that subscribes to a labeled object point cloud topic,
    computes the object's geometric properties (center, height, and radius), and publishes
    the object's center as a PointStamped message, and can add the object as a
    collision object (cylinder) to the MoveIt PlanningScene. 
    """
    def __init__(self):
        rospy.init_node("object_inserter")
        self.scene = PlanningSceneInterface(synchronous=True)
        rospy.sleep(2)
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.sub = rospy.Subscriber("/objects_point_cloud",
                                    PointCloud2, self.callback, queue_size=1)

        self.source_frame = "base_footprint"   # The coordinate frame of the point cloud
        self.target_frame = "odom"             # MoveIt's planning frame
        self.obj_id = "grasp_target"
        self.object_added = False
        print("C: Starting init")
        self.center_pub = rospy.Publisher("/object_center", PointStamped, queue_size=1)
        self.center_sent = False
        self.center_point = None
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        rospy.loginfo("Node initialized")


    def callback(self, msg):
        if self.center_sent:
            return
        # Extract points 
        pts = np.array(list(pc2.read_points(msg, skip_nans=True,
                                            field_names=("x", "y", "z"))))
        if pts.shape[0] < 5:
            rospy.logwarn("Not enough points in object point cloud.")
            return

        # Compute dimensions 
        min_xyz = pts.min(axis=0)
        max_xyz = pts.max(axis=0)
        height  = max_xyz[2] - min_xyz[2] + 0.03
        radius  = max(max_xyz[0] - min_xyz[0],
                      max_xyz[1] - min_xyz[1]) / 2.0 # 2.0
        center  = (min_xyz + max_xyz) / 2.0

        # First generate pose under base_footprint 
        pose_bf = PoseStamped()
        pose_bf.header.frame_id = self.source_frame
        pose_bf.header.stamp    = rospy.Time(0)          # Let TF use the latest data
        pose_bf.pose.position.x = float(center[0])
        pose_bf.pose.position.y = float(center[1])
        pose_bf.pose.position.z = float(center[2])
        pose_bf.pose.orientation.w = 1.0                # No rotation
        
        #
        self.center_point = PointStamped()
        self.center_point.header.frame_id = self.source_frame
        self.center_point.header.stamp = rospy.Time.now()
        self.center_point.point.x = float(center[0])
        self.center_point.point.y = float(center[1])
        self.center_point.point.z = float(center[2])

        self.center_sent = True
        rospy.loginfo("Center point calculated and fixed.")


        try:
            # to odom 
            T = self.tf_buffer.lookup_transform(
                    self.target_frame,                   # target frame
                    self.source_frame,                   # current frame
                    rospy.Time(0), rospy.Duration(0.5))  
            pose_odom = tf2_geometry_msgs.do_transform_pose(pose_bf, T)
            pose_odom.header.frame_id = self.target_frame
        except (tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            rospy.logwarn(f"TF transform failed: {e}")
            return
        
        # ================================================================================
        # to add the object in the PlanningScene, because of some execution problem, stop using
        
        if not self.object_added:
            self.scene.add_cylinder(self.obj_id,
                                    pose_odom,
                                    height=height,
                                    radius=radius)
            rospy.logwarn(f"Added grasp target '{self.obj_id}'"
                            f"  center=({pose_odom.pose.position.x:.3f},"
                            f" {pose_odom.pose.position.y:.3f},"
                            f" {pose_odom.pose.position.z:.3f})"
                            f"  frame={pose_odom.header.frame_id}")
            
            # Add box obstacle to the left of the object 
            box_pose = PoseStamped()
            box_pose.header.frame_id = pose_odom.header.frame_id
            box_pose.pose.orientation.w = 1.0
            box_pose.pose.position.x = pose_odom.pose.position.x
            box_pose.pose.position.y = pose_odom.pose.position.y + 0.5  # left side (Y axis)
            box_pose.pose.position.z = pose_odom.pose.position.z        # same height

            self.scene.add_box("left_obstacle", box_pose,
                            size=(1.0, 0.6, 0.7))  # 50cm x 50cm x 50cm
            rospy.logwarn("Added left obstacle 'left_obstacle'")
            
            # Add top obstacle
            top_box_pose = PoseStamped()
            top_box_pose.header.frame_id = pose_odom.header.frame_id
            top_box_pose.pose.orientation.w = 1.0
            top_box_pose.pose.position.x = pose_odom.pose.position.x
            top_box_pose.pose.position.y = pose_odom.pose.position.y
            top_box_pose.pose.position.z = pose_odom.pose.position.z + height / 2.0 + 0.27  # grasp obj顶面 + box一半 + 间距20cm

            self.scene.add_box("top_obstacle", top_box_pose, size=(0.1, 0.3, 0.5))
            rospy.logwarn("Added top obstacle 'top_obstacle'")
            
            
            self.object_added = True
        # ================================================================================
        
        
    def timer_callback(self, event):
        if self.center_point:
            self.center_point.header.stamp = rospy.Time.now()
            self.center_pub.publish(self.center_point)



def main():
    ObjectInserter()
    rospy.spin()

if __name__ == "__main__":
    main()
