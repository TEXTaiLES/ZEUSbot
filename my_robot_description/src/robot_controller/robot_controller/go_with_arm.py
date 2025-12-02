#!/usr/bin/env python3
import math
import os
import yaml
import json
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
import shutil # ΝΕΟ: Για πιο δυναμική διαγραφή

class CameraCaptureNode(Node):
    def __init__(self):
        super().__init__('camera_capture_node')

        # --- ΡΥΘΜΙΣΕΙΣ PATHS ---
        self.data_dir = '/home/hercules/data/'
        self.image_folder = os.path.join(self.data_dir, 'image_data') 
        self.json_file = os.path.join(self.data_dir, 'lidar_log.json') 

        # Δημιουργία φακέλων
        try:
            os.makedirs(self.image_folder, exist_ok=True)
        except PermissionError:
            self.get_logger().error(f"❌ DEN EXW DIKAIOMA EGGRAFIS STO {self.data_dir}. Kane chown h trexe me sudo.")

        # --- ΚΑΘΑΡΙΣΜΟΣ & ΑΡΧΙΚΟΠΟΙΗΣΗ ---
        self.init_json_file()

        self.bridge = CvBridge()
        self.last_image = None
        self.latest_lidar_avg = 0.0
        self.json_data_list = [] 

        # Subscribers
        self.create_subscription(Image, '/camera_sensor/image_raw', self.camera_callback, 10)
        self.create_subscription(LaserScan, '/gazebo_ros_ray_sensor3/out', self.lidar_callback, 10)

        # Timer
        self.create_timer(0.5, self.capture_photo_and_log)
        self.get_logger().info("📸 CameraCaptureNode started.")

    def init_json_file(self):
        """Καθαρίζει τα παλιά και φτιάχνει ένα άδειο JSON αρχείο"""
        # Καθαρισμός εικόνων
        if os.path.exists(self.image_folder):
            for f in os.listdir(self.image_folder):
                if f.endswith((".jpg", ".png")):
                    try:
                        os.remove(os.path.join(self.image_folder, f))
                    except Exception as e:
                        self.get_logger().warn(f"Cannot delete image: {e}")

        # Διαγραφή παλιού JSON αν υπάρχει
        if os.path.exists(self.json_file):
            try:
                os.remove(self.json_file)
            except OSError as e:
                self.get_logger().error(f"⚠️ Failed to delete old JSON (Permission Error?): {e}")

        # Δημιουργία ΚΕΝΟΥ αρχείου JSON τώρα, για να δούμε αν γράφεται
        try:
            with open(self.json_file, 'w') as f:
                json.dump([], f) # Γράφουμε μια άδεια λίστα
            self.get_logger().info(f"✅ JSON file initialized successfully at: {self.json_file}")
        except Exception as e:
            self.get_logger().fatal(f"🔥 CRITICAL: CANNOT CREATE JSON FILE! Error: {e}")

    def camera_callback(self, msg):
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")

    def lidar_callback(self, msg):
        valid_ranges = [d for d in msg.ranges if d > 0.01 and not math.isinf(d)]
        if valid_ranges:
            self.latest_lidar_avg = sum(valid_ranges) / len(valid_ranges)
        else:
            self.latest_lidar_avg = 0.0

    def capture_photo_and_log(self):
        if self.last_image is None:
            # Αν θες να μην γεμίζει logs η κονσόλα, σχολίασε την από κάτω γραμμή
            # self.get_logger().warn("Waiting for image...")
            return
        
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        img_filename = f"img_{timestamp_str}.jpg"
        
        # --- 1. ΑΠΟΘΗΚΕΥΣΗ ΕΙΚΟΝΑΣ ---
        try:
            full_img_path = os.path.join(self.image_folder, img_filename)
            cv2.imwrite(full_img_path, self.last_image)
        except Exception as e:
            self.get_logger().error(f"❌ Image Save Failed: {e}")
            return # Αν αποτύχει η εικόνα, σταματάμε

        # --- 2. ΕΓΓΡΑΦΗ JSON ---
        try:
            entry = {
                "timestamp": timestamp_str,
                "image_filename": img_filename,
                "lidar_average_distance": round(self.latest_lidar_avg, 3)
            }
            
            self.json_data_list.append(entry)
            
            with open(self.json_file, 'w') as f:
                json.dump(self.json_data_list, f, indent=4)
            
            self.get_logger().info(f"💾 OK: {img_filename} | JSON Updated")

        except Exception as e:
            # ΑΥΤΟ ΤΟ ΜΗΝΥΜΑ ΘΑ ΜΑΣ ΠΕΙ ΓΙΑΤΙ ΔΕΝ ΦΤΙΑΧΝΕΤΑΙ ΤΟ JSON
            self.get_logger().error(f"🔥 JSON WRITE FAILED: {e}")

# ==============================================================
# === Κόμβος: Smart Arm Follower (Απαράλλαχτος) ================
# ==============================================================
class SmartArmFollower(Node):
    def __init__(self):
        super().__init__('smart_arm_follower')
        self.safe_distance = 1.6
        self.arm_config_path = '/home/hercules/my_robot_description/src/robot_controller/robot_controller/position.yaml'
        self.paused_for_photo = False
        self.is_moving = True
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/gazebo_ros_ray_sensor/out', self.lidar_callback, 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)
        self.positions = self.load_positions(self.arm_config_path)
        self.create_timer(2.0, self.control_cycle)
        self.get_logger().info("✅ SmartArmFollower started.")

    def load_positions(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f"❌ YAML file not found: {path}")
            return []
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        return data['poses']

    def control_cycle(self):
        if self.is_moving:
            self.is_moving = False
            self.paused_for_photo = True
            self.stop_robot()
            self.execute_all_arm_poses()
        else:
            self.is_moving = True
            self.paused_for_photo = False

    def lidar_callback(self, msg):
        if self.paused_for_photo: return
        valid_ranges = [d for d in msg.ranges if d > 0.01]
        if not valid_ranges: return
        avg_dist = sum(valid_ranges) / len(valid_ranges)
        twist = Twist()
        if avg_dist < self.safe_distance:
            twist.linear.x = 0.2
            twist.angular.z = -0.25
        elif avg_dist > self.safe_distance:
            twist.linear.x = 0.2
            twist.angular.z = 0.25
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def execute_all_arm_poses(self):
        for i, pose in enumerate(self.positions):
            try:
                joints = [math.radians(pose[k]) for k in ['yaw1','pitch1','pitch2','yaw2','pitch3','roll1']]
                self.publish_trajectory(joints)
                time.sleep(1.5)
            except KeyError: pass

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def publish_trajectory(self, joints):
        traj = JointTrajectory()
        traj.header = Header()
        traj.header.frame_id = 'base_footprint'
        traj.joint_names = ['arm_base_2_joint', 'arm_base_forearm_joint', 'forearm_hand_joint', 'hand_2_joint', 'base_camera_joint', 'hand_3_joint']
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start.sec = 1
        traj.points.append(point)
        self.arm_pub.publish(traj)

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
        pass
    finally:
        arm_node.destroy_node()
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()