#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import tkinter as tk


class Arm6DOFController(Node):
    def __init__(self):
        super().__init__('arm_6dof_gui')
        self.pub = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)

        # === GUI ===
        self.root = tk.Tk()
        self.root.title("6-DOF Arm Joint Control (Yaw/Pitch/Roll)")

        tk.Label(
            self.root,
            text="Έλεγχος αρθρώσεων (yaw₁, pitch₁, pitch₂, yaw₂, pitch₃, roll₁)"
        ).pack(pady=5)

        # Sliders
        self.j1 = self.make_slider("Yaw₁ (arm_base_2_joint, γύρω από Z)", 0, 360, 0)
        self.j2 = self.make_slider("Pitch₁ (arm_base_forearm_joint, γύρω από Y)", -90, 90, 0)
        self.j3 = self.make_slider("Pitch₂ (forearm_hand_joint, γύρω από Y)", -180, 180, 0)
        self.j4 = self.make_slider("Yaw₂ (hand_2_joint, γύρω από Z)", 0, 360, 0)
        self.j5 = self.make_slider("Pitch₃ (base_camera_joint, γύρω από Y)", 0, 360, 0)
        self.j6 = self.make_slider("Roll₁ (hand_3_joint, γύρω από X)", 0, 360, 0)

        self.status = tk.Label(self.root, text="", font=("Courier", 10))
        self.status.pack(pady=6)

        self.root.after(100, self.ros_spin)
        self.root.mainloop()

    # ---------- helpers ----------
    def make_slider(self, label, min_val, max_val, init):
        s = tk.Scale(
            self.root, from_=min_val, to=max_val, resolution=1,
            orient="horizontal", label=label, length=360,
            command=self.update_joints
        )
        s.set(init)
        s.pack(pady=4)
        return s

    def ros_spin(self):
        rclpy.spin_once(self, timeout_sec=0)
        self.root.after(50, self.ros_spin)

    # ---------- update ----------
    def update_joints(self, event=None):
        # deg → rad
        t1 = math.radians(self.j1.get())
        t2 = math.radians(self.j2.get())
        t3 = math.radians(self.j3.get())
        t4 = math.radians(self.j4.get())
        t5 = math.radians(self.j5.get())
        t6 = math.radians(self.j6.get())

        self.status.config(
            text=(
                f"θ1={math.degrees(t1):6.1f}° | θ2={math.degrees(t2):6.1f}° | "
                f"θ3={math.degrees(t3):6.1f}° | θ4={math.degrees(t4):6.1f}° | "
                f"θ5={math.degrees(t5):6.1f}° | θ6={math.degrees(t6):6.1f}°"
            ),
            fg="green"
        )

        self.publish_trajectory(t1, t2, t3, t4, t5, t6)

    # ---------- publisher ----------
    def publish_trajectory(self, t1, t2, t3, t4, t5, t6):
        traj = JointTrajectory()
        traj.header = Header()
        traj.header.frame_id = 'base_footprint'

        # ✅ Ακριβώς τα joints που υπάρχουν στο URDF
        traj.joint_names = [
            'arm_base_2_joint',         # Yaw₁
            'arm_base_forearm_joint',   # Pitch₁ (ώμος)
            'forearm_hand_joint',       # Pitch₂ (αγκώνας)
            'hand_2_joint',             # Yaw₂ (καρπός)
            'base_camera_joint',        # Pitch₃ (κάμερα)
            'hand_3_joint'              # Roll₁ (νέα άρθρωση γύρω από X)
        ]

        point = JointTrajectoryPoint()
        point.positions = [t1, t2, t3, t4, t5, t6]
        point.time_from_start.sec = 1
        traj.points.append(point)
        self.pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    Arm6DOFController()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
