#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Twist
import numpy as np


class TfblControllerNode(Node): 
    def __init__(self):
        super().__init__("tfbl_controller") 

        self.eta2_ref_ = 1
        self.k1_, self.k2_, self.k3_ = 1, 2, 0.5
        self.x1_, self.x2_, self.x3_, self.x4_ = 0.0, 0.0, float(np.pi/2), 1.0
        self.u1_, self.u2_ = 0.0 , 0.0


        self.cmd_vel_publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        # self.get_state_timer_ = self.create_timer(1/500.0, self.get_state_timer_callback)

        self.control_loop_timer = self.create_timer(1/500.0, self.control_loop)
        
        self.get_logger().info("The tfbl controller node is running.")


    def control_loop(self):

        cmd = Twist()
        cmd.linear.x = float(self.x4_)
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = float(self.u2_)

        self.cmd_vel_publisher_.publish(cmd)

        try:
            trans = self.tf_buffer_.lookup_transform("odom", 'base_link', rclpy.time.Time())
            
            self.x1_ = trans.transform.translation.x
            self.x2_ = trans.transform.translation.y

            q = trans.transform.rotation
            self.x3_ = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

            self.get_logger().info(str(self.x1_) + ", " + str(self.x2_) + ", " + str(self.x3_))


            xi1 = self.x1_**2 + self.x2_**2 - 1
            xi2 = 2 * self.x4_ * (self.x1_ * np.cos(self.x3_) + self.x2_ * np.sin(self.x3_))

            Lg1LfS1 = 2 * self.x1_ * np.cos(self.x3_) + 2 * self.x2_ * np.sin(self.x3_)
            Lg2LfS1 = 2 * self.x4_ * (self.x2_ * np.cos(self.x3_) - self.x1_ * np.sin(self.x3_))
            Lf2S1 = 2 * self.x4_**2

            eta1 = np.arctan2(self.x2_, self.x1_)
            eta2 = -(self.x4_ * (self.x2_ * np.cos(self.x3_) - self.x1_ * np.sin(self.x3_))) / (self.x1_**2 + self.x2_**2)

            Lg1LfS2 = -(self.x2_ * np.cos(self.x3_) - self.x1_ * np.sin(self.x3_)) / (self.x1_**2 + self.x2_**2)
            Lg2LfS2 = (self.x4_ * (self.x1_ * np.cos(self.x3_) + self.x2_ * np.sin(self.x3_))) / (self.x1_**2 + self.x2_**2)

            Lf2S2 = (
                np.cos(self.x3_) * self.x4_ * (
                    (np.sin(self.x3_) * self.x4_) / (self.x1_**2 + self.x2_**2) + 
                    (2 * self.x1_ * self.x4_ * (self.x2_ * np.cos(self.x3_) - self.x1_ * np.sin(self.x3_))) / (self.x1_**2 + self.x2_**2)**2
                ) - 
                np.sin(self.x3_) * self.x4_ * (
                    (np.cos(self.x3_) * self.x4_) / (self.x1_**2 + self.x2_**2) - 
                    (2 * self.x2_ * self.x4_ * (self.x2_ * np.cos(self.x3_) - self.x1_ * np.sin(self.x3_))) / (self.x1_**2 + self.x2_**2)**2
                )
            )

            D = np.array([[Lg1LfS1, Lg2LfS1], [Lg1LfS2, Lg2LfS2]])

            try:
                M = np.linalg.inv(D)
            except np.linalg.LinAlgError:
                self.get_logger().warn("Singular matrix detected, skipping this control cycle.")
                return

            v1_fbl = -Lf2S1 - self.k1_ * xi1 - self.k2_ * xi2
            v2_fbl = -Lf2S2 - self.k3_ * (eta2 - self.eta2_ref_)

            self.u1_, self.u2_ = M @ np.array([v1_fbl, v2_fbl])

            self.x4_ =  self.x4_  + (1/500 * self.u1_ )


        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

    




    def quaternion_to_yaw(self, x, y, z, w):
        # Convert quaternion (x, y, z, w) to yaw
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = TfblControllerNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()