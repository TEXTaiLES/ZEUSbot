#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PoseArray, Pose
from rclpy.time import Time
import json
import os 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(PoseArray, 'robots_positions', 10)
        
        # Τρέχουμε τον έλεγχο συχνά (π.χ. 10Hz = 0.1s), αλλά αποθηκεύουμε ΜΟΝΟ αν κουνηθεί
        self.timer = self.create_timer(0.1, self.check_and_publish)

        # ---------------------------------------------------------
        # ΡΥΘΜΙΣΕΙΣ PATH, JSON ΚΑΙ FRAMES
        # ---------------------------------------------------------
        self.save_directory = "/home/hercules/data/" 
        self.filename = "ugv_path_data.json"
        self.full_file_path = os.path.join(self.save_directory, self.filename)
        
        self.target_frame_base = "base_footprint"  
        self.target_frame_2nd  = "camera_link_optical" 
        
        # ΝΕΟ: Όριο κίνησης (σε μέτρα). Αν κουνηθεί λιγότερο από 1cm, το αγνοούμε.
        self.movement_threshold = 0.01 

        # Μεταβλητές για να θυμόμαστε την ΤΕΛΕΥΤΑΙΑ ΚΑΤΑΓΕΓΡΑΜΜΕΝΗ θέση
        self.last_recorded_base = None # (x, y, z)
        self.last_recorded_sec  = None # (x, y, z)

        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory, exist_ok=True)

        if os.path.exists(self.full_file_path):
            os.remove(self.full_file_path)

        self.path_history = []  
        
        # Ιστορικό για Plots
        self.base_x, self.base_y = [], []
        self.sec_x, self.sec_y, self.sec_z = [], [], []
        self.sec_dx, self.sec_dy, self.sec_dz = [], [], []

        self.quiver = None 

        self.get_logger().info(f"Tracking Changes (> {self.movement_threshold}m): {self.target_frame_base} & {self.target_frame_2nd}")

        # ---------------------------------------------------------
        # Matplotlib Setup
        # ---------------------------------------------------------
        plt.ion()
        self.fig = plt.figure(figsize=(12, 6))
        
        # --- Subplot 1: 2D Base ---
        self.ax1 = self.fig.add_subplot(1, 2, 1)
        self.ax1.set_title(f"Base Path (2D)\nFrame: {self.target_frame_base}")
        self.ax1.set_xlabel("X [m]")
        self.ax1.set_ylabel("Y [m]")
        self.ax1.grid(True)
        self.ax1.set_xlim(-5, 5)
        self.ax1.set_ylim(-5, 5)
        
        self.line1, = self.ax1.plot([], [], 'b-', label='Path')
        self.point1, = self.ax1.plot([], [], 'ro', label='Base')
        self.ax1.legend(loc='upper right')

        # --- Subplot 2: 3D Second Point ---
        self.ax2 = self.fig.add_subplot(1, 2, 2, projection='3d')
        self.ax2.set_title(f"Z-Axis Orientation Cloud (3D)\nFrame: {self.target_frame_2nd}")
        self.ax2.set_xlabel("X [m]")
        self.ax2.set_ylabel("Y [m]")
        self.ax2.set_zlabel("Z [m]")
        
        self.point2, = self.ax2.plot([], [], [], 'mo', markersize=3, linestyle='None', label='History')
        self.ax2.legend(loc='upper right')

    def quaternion_to_z_axis(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        dir_x = 2 * (x*z + y*w)
        dir_y = 2 * (y*z - x*w)
        dir_z = 1 - 2 * (x**2 + y**2)
        return dir_x, dir_y, dir_z

    def calculate_distance(self, pos1, pos2):
        """Υπολογίζει την ευκλείδεια απόσταση μεταξύ δύο σημείων (x,y,z)"""
        return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 + (pos1[2]-pos2[2])**2)

    def check_and_publish(self):
        try:
            # 1. Base Transform
            tf_base = self.tf_buffer.lookup_transform('world', self.target_frame_base, Time())
            bx, by, bz = tf_base.transform.translation.x, tf_base.transform.translation.y, tf_base.transform.translation.z
            current_base_pos = (bx, by, bz)

            # 2. Second Point Transform
            has_sec = False
            current_sec_pos = (0.0, 0.0, 0.0)
            dx, dy, dz = 0.0, 0.0, 1.0

            try:
                tf_sec = self.tf_buffer.lookup_transform('world', self.target_frame_2nd, Time())
                sx, sy, sz = tf_sec.transform.translation.x, tf_sec.transform.translation.y, tf_sec.transform.translation.z
                dx, dy, dz = self.quaternion_to_z_axis(tf_sec.transform.rotation)
                current_sec_pos = (sx, sy, sz)
                has_sec = True
            except TransformException:
                pass

            # --- ΛΟΓΙΚΗ ΕΛΕΓΧΟΥ ΚΙΝΗΣΗΣ ---
            should_record = False

            # Αν είναι η πρώτη φορά που τρέχει, καταγράφουμε σίγουρα
            if self.last_recorded_base is None:
                should_record = True
            else:
                # Υπολογισμός απόστασης από την τελευταία ΚΑΤΑΓΕΓΡΑΜΜΕΝΗ θέση
                dist_base = self.calculate_distance(current_base_pos, self.last_recorded_base)
                
                dist_sec = 0.0
                if has_sec and self.last_recorded_sec:
                    dist_sec = self.calculate_distance(current_sec_pos, self.last_recorded_sec)

                # Αν κουνήθηκε η βάση OR το χέρι/κάμερα περισσότερο από το όριο
                if dist_base > self.movement_threshold or dist_sec > self.movement_threshold:
                    should_record = True

            if should_record:
                # Ενημερώνουμε τις "τελευταίες" θέσεις
                self.last_recorded_base = current_base_pos
                if has_sec:
                    self.last_recorded_sec = current_sec_pos

                # Publish Topic (για debugging/visualization στο rviz)
                pose_array = PoseArray()
                pose_array.header.stamp = self.get_clock().now().to_msg()
                pose_array.header.frame_id = 'world'
                p = Pose()
                p.position.x, p.position.y, p.position.z = bx, by, bz
                pose_array.poses.append(p)
                self.publisher.publish(pose_array)

                self.get_logger().info(f"Moved! Base: ({bx:.2f}, {by:.2f}) | 2nd: ({current_sec_pos[0]:.2f}...)")

                # Update Lists
                self.base_x.append(bx)
                self.base_y.append(by)
                
                if has_sec:
                    self.sec_x.append(current_sec_pos[0])
                    self.sec_y.append(current_sec_pos[1])
                    self.sec_z.append(current_sec_pos[2])
                    self.sec_dx.append(dx)
                    self.sec_dy.append(dy)
                    self.sec_dz.append(dz)

                # Update Plots & Save JSON
                self.update_plots(bx, by, has_sec)
                self.save_to_json(bx, by, bz, current_sec_pos[0], current_sec_pos[1], current_sec_pos[2])

        except TransformException as e:
            # self.get_logger().warn(f"TF Error: {str(e)}")
            pass

    def save_to_json(self, bx, by, bz, sx, sy, sz):
        data_point = {
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "base": {"x": bx, "y": by, "z": bz},
            "camera_point": {"x": sx, "y": sy, "z": sz}
        }
        self.path_history.append(data_point)

        try:
            with open(self.full_file_path, 'w') as f:
                json.dump(self.path_history, f, indent=4)
        except Exception as e:
            self.get_logger().error(f"Failed to write JSON: {e}")

    def update_plots(self, bx, by, has_sec):
        # --- Update 2D Plot ---
        self.line1.set_data(self.base_x, self.base_y)
        self.point1.set_data([bx], [by])
        
        if self.base_x:
            self.ax1.set_xlim(min(self.base_x)-1, max(self.base_x)+1)
            self.ax1.set_ylim(min(self.base_y)-1, max(self.base_y)+1)

        # --- Update 3D Plot ---
        if has_sec and self.sec_x:
            self.point2.set_data(self.sec_x, self.sec_y)
            self.point2.set_3d_properties(self.sec_z)

            if self.quiver:
                self.quiver.remove()
            
            self.quiver = self.ax2.quiver(
                self.sec_x, self.sec_y, self.sec_z,
                self.sec_dx, self.sec_dy, self.sec_dz,
                length=0.2, normalize=True, color='blue', linewidth=1, arrow_length_ratio=0.3
            )

            # Dynamic Scaling
            min_x, max_x = min(self.sec_x), max(self.sec_x)
            min_y, max_y = min(self.sec_y), max(self.sec_y)
            min_z, max_z = min(self.sec_z), max(self.sec_z)

            mid_x = (max_x + min_x) * 0.5
            mid_y = (max_y + min_y) * 0.5
            mid_z = (max_z + min_z) * 0.5

            max_range = max(max_x - min_x, max_y - min_y, max_z - min_z) / 2.0
            if max_range < 0.5: max_range = 0.5

            self.ax2.set_xlim(mid_x - max_range, mid_x + max_range)
            self.ax2.set_ylim(mid_y - max_range, mid_y + max_range)
            self.ax2.set_zlim(mid_z - max_range, mid_z + max_range)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()