#!/usr/bin/env python3
import math
import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import time
from datetime import datetime


# ==============================================================
# === Κόμβος: Λήψη φωτογραφιών κάθε 0.5s εις το άπειρον =========
# ==============================================================
class CameraCaptureNode(Node):
    def __init__(self):
        super().__init__('camera_capture_node')

        self.image_folder = '/home/hercules/data/image_data/'
        os.makedirs(self.image_folder, exist_ok=True)
        self.delete_old_images()
        self.get_logger().info("🧹 Old images deleted. Folder ready.")

        self.bridge = CvBridge()
        self.last_image = None
        self.create_subscription(Image, '/camera_sensor/image_raw', self.camera_callback, 10)

        # λήψη κάθε 0.5s
        self.create_timer(0.5, self.capture_photo)
        self.get_logger().info("📸 CameraCaptureNode started (photo every 0.5s).")

    def delete_old_images(self):
        for f in os.listdir(self.image_folder):
            if f.endswith((".jpg", ".png")):
                os.remove(os.path.join(self.image_folder, f))
                self.get_logger().info(f"🗑️ Deleted old image: {f}")

    def camera_callback(self, msg):
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")

    def capture_photo(self):
        try:
            if self.last_image is None:
                return
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename = os.path.join(self.image_folder, f"img_{timestamp}.jpg")
            cv2.imwrite(filename, self.last_image)
            self.get_logger().info(f"📷 Saved: {filename}")
        except Exception as e:
            self.get_logger().error(f"❌ Save failed: {e}")


# ==============================================================
# === Κόμβος: Έλεγχος 6DOF βραχίονα & ρομπότ με LIDAR ==========
# ==============================================================
class SmartArmFollower(Node):
    def __init__(self):
        super().__init__('smart_arm_follower')

        # === PARAMETERS ===
        self.safe_distance = 1.6
        self.arm_config_path = '/home/hercules/my_robot_description/src/robot_controller/robot_controller/position.yaml'

        self.paused_for_photo = False
        self.is_moving = True

        # === Publishers / Subscribers ===
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/gazebo_ros_ray_sensor/out', self.lidar_callback, 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)

        # === Load arm poses (με μοίρες) ===
        self.positions = self.load_positions(self.arm_config_path)

        # === Timers ===
        self.create_timer(2.0, self.control_cycle)

        self.get_logger().info("✅ SmartArmFollower started (6DOF Arm + LiDAR).")

    # -------------------------------------------------
    def load_positions(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f"❌ YAML file not found: {path}")
            return []
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        self.get_logger().info(f"📄 Loaded {len(data['poses'])} arm poses.")
        return data['poses']

    # -------------------------------------------------
    def control_cycle(self):
        """Εναλλάσσει φάσεις: Κίνηση ↔ Εκτέλεση πόζας"""
        if self.is_moving:
            self.get_logger().info("🛑 Stop & execute arm poses.")
            self.is_moving = False
            self.paused_for_photo = True
            self.stop_robot()
            self.execute_all_arm_poses()
        else:
            self.get_logger().info("🚶 Resume moving (LiDAR active).")
            self.is_moving = True
            self.paused_for_photo = False

    # -------------------------------------------------
    def lidar_callback(self, msg):
        if self.paused_for_photo:
            return
        valid_ranges = [d for d in msg.ranges if d > 0.01]
        if not valid_ranges:
            return
        avg_dist = sum(valid_ranges) / len(valid_ranges)

        twist = Twist()
        if avg_dist < self.safe_distance:
            self.get_logger().info(f"Too close ({avg_dist:.2f} m) → turn right")
            twist.linear.x = 0.2
            twist.angular.z = -0.25
        elif avg_dist > self.safe_distance:
            self.get_logger().info(f"Too far ({avg_dist:.2f} m) → turn left")
            twist.linear.x = 0.2
            twist.angular.z = 0.25
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    # -------------------------------------------------
    def execute_all_arm_poses(self):
        """Διατρέχει όλα τα poses από YAML και τα στέλνει στο βραχίονα"""
        for i, pose in enumerate(self.positions):
            try:
                # Οι γωνίες είναι σε μοίρες μέσα στο YAML
                t1 = math.radians(pose['yaw1'])
                t2 = math.radians(pose['pitch1'])
                t3 = math.radians(pose['pitch2'])
                t4 = math.radians(pose['yaw2'])
                t5 = math.radians(pose['pitch3'])
                t6 = math.radians(pose['roll1'])

                self.publish_trajectory([t1, t2, t3, t4, t5, t6])
                self.get_logger().info(
                    f"🤖 Pose {i+1}/{len(self.positions)} "
                    f"→ [θ1..θ6]={pose}"
                )
                time.sleep(1.5)

            except KeyError:
                self.get_logger().warn(f"⚠️ Invalid YAML format at pose {i}")

        self.get_logger().info("✅ Arm poses done — resume soon.")

    # -------------------------------------------------
    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    # -------------------------------------------------
    def publish_trajectory(self, joints):
        traj = JointTrajectory()
        traj.header = Header()
        traj.header.frame_id = 'base_footprint'
        traj.joint_names = [
            'arm_base_2_joint',
            'arm_base_forearm_joint',
            'forearm_hand_joint',
            'hand_2_joint',
            'base_camera_joint',
            'hand_3_joint'
        ]
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start.sec = 1
        traj.points.append(point)
        self.arm_pub.publish(traj)


# ==============================================================
# === MAIN: Εκτέλεση και των δύο κόμβων ========================
# ==============================================================
def main(args=None):
    rclpy.init(args=args)
    arm_node = SmartArmFollower()
    camera_node = CameraCaptureNode()

    executor = MultiThreadedExecutor()
    executor.add_node(arm_node)
    executor.add_node(camera_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        arm_node.get_logger().info("🛑 Shutting down.")
    finally:
        arm_node.destroy_node()
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
