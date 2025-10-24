#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import tkinter as tk
import numpy as np


class Arm6DOFController(Node):
    def __init__(self):
        super().__init__('arm_6dof_gui')
        self.pub = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)

        # === Μήκη συνδέσμων (προσαρμόστε αν έχετε διαφορετικά στο URDF) ===
        self.L1 = 0.20   # arm_base_2_joint -> shoulder
        self.L2 = 0.60   # shoulder -> elbow
        self.L3 = 0.60   # elbow -> wrist
        self.L4 = 0.20   # wrist -> camera
        self.L5 = 0.20   # camera -> roll
        self.L6 = 0.20   # τελικό κομμάτι

        # === GUI ===
        self.root = tk.Tk()
        self.root.title("6-DOF Arm Joint Control (Yaw/Pitch/Roll + Forward Kinematics)")

        tk.Label(
            self.root,
            text="Έλεγχος αρθρώσεων (yaw₁, pitch₁, pitch₂, yaw₂, pitch₃, roll₁)"
        ).pack(pady=5)

        self.j1 = self.make_slider("Yaw₁ (Z)", 0, 360, 0)
        self.j2 = self.make_slider("Pitch₁ (Y)", -90, 90, 0)
        self.j3 = self.make_slider("Pitch₂ (Y)", -90, 90, 0)
        self.j4 = self.make_slider("Yaw₂ (Z)", 0, 360, 0)
        self.j5 = self.make_slider("Pitch₃ (Y)", 0, 360, 0)
        self.j6 = self.make_slider("Roll₁ (X)", 0, 360, 0)

        self.status = tk.Label(self.root, text="", font=("Courier", 10))
        self.status.pack(pady=6)

        self.position_label = tk.Label(self.root, text="", font=("Courier", 12), fg="blue")
        self.position_label.pack(pady=6)

        self.root.after(100, self.ros_spin)
        self.root.mainloop()

    # ---------- GUI ----------
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

    # ---------- Update ----------
    def update_joints(self, event=None):
        t1 = math.radians(self.j1.get())
        t2 = math.radians(self.j2.get())
        t3 = math.radians(self.j3.get())
        t4 = math.radians(self.j4.get())
        t5 = math.radians(self.j5.get())
        t6 = math.radians(self.j6.get())

        # Ενημέρωση label γωνιών
        self.status.config(
            text=(
                f"θ1={self.j1.get():6.1f}° | θ2={self.j2.get():6.1f}° | "
                f"θ3={self.j3.get():6.1f}° | θ4={self.j4.get():6.1f}° | "
                f"θ5={self.j5.get():6.1f}° | θ6={self.j6.get():6.1f}°"
            ),
            fg="green"
        )

        # Υπολογισμός ευθύ κινηματικού (FK)
        x, y, z = self.forward_kinematics(t1, t2, t3, t4, t5, t6)
        self.position_label.config(text=f"Εnd-Effector: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m")

        # Δημοσίευση των γωνιών
        self.publish_trajectory(t1, t2, t3, t4, t5, t6)

    # ---------- Forward Kinematics ----------
    def dh_transform(self, a, alpha, d, theta):
        """Υπολογισμός πίνακα μετασχηματισμού Denavit–Hartenberg."""
        ca, sa = math.cos(alpha), math.sin(alpha)
        ct, st = math.cos(theta), math.sin(theta)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,       sa,       ca,      d],
            [0,        0,        0,      1]
        ])

    def forward_kinematics(self, t1, t2, t3, t4, t5, t6):
        """Υπολογισμός θέσης end-effector (καρπού) ως προς τη βάση."""

        # Ενδεικτικά DH παράμετροι (προσαρμόστε για το URDF σας)
        # a_i, α_i, d_i, θ_i
        A1 = self.dh_transform(0,     math.pi/2, self.L1, t1)
        A2 = self.dh_transform(self.L2, 0,       0,     t2)
        A3 = self.dh_transform(self.L3, 0,       0,     t3)
        A4 = self.dh_transform(0,     math.pi/2, self.L4, t4)
        A5 = self.dh_transform(0,    -math.pi/2, self.L5, t5)
        A6 = self.dh_transform(0,        0,      self.L6, t6)

        # Ολικός μετασχηματισμός
        T = A1 @ A2 @ A3 @ A4 @ A5 @ A6

        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        return x, y, z

    # ---------- Publisher ----------
    def publish_trajectory(self, t1, t2, t3, t4, t5, t6):
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

