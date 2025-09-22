# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from tf2_msgs.msg import TFMessage
# from tf2_ros import TransformListener, Buffer
# from geometry_msgs.msg import Twist
# import numpy as np
# from qpsolvers import solve_qp
# import scipy.sparse as sp
# import sys


# class QpControllerNode(Node): 
#     def __init__(self, desired_x=0.0, desired_y=5.0):
#         super().__init__("qp_controller") 

#         # Controller parameters
#         self.dt = 0.01  # 100 Hz control loop
        
#         # Robot state: [x, y, theta, v]
#         self.x1_ = 0.0  # x position
#         self.x2_ = 0.0  # y position  
#         self.x3_ = 0.0  # theta (orientation)
#         self.x4_ = 0.0  # v (linear velocity)
        
#         # Control inputs
#         self.u1_ = 0.0  # angular velocity control (theta_dot)
#         self.u2_ = 0.0  # linear acceleration control (v_dot)
        
#         # Desired trajectory (set from command line arguments)
#         self.yd_ = np.array([desired_x, desired_y])      # desired position
#         self.dyd_ = np.array([0.0, 0.0])     # desired velocity
#         self.ddyd_ = np.array([0.0, 0.0])    # desired acceleration
        
#         # QP solver matrices (will be updated in control loop)
#         self.P = np.diag([1, 1, 10, 10])  # Cost matrix for [u_1, u_2, delta1, delta2]
        
#         # ROS2 setup
#         self.cmd_vel_publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

#         self.tf_buffer_ = Buffer()
#         self.tf_listener_ = TransformListener(self.tf_buffer_, self)

#         # # Control loop at 50 Hz
#         # self.control_loop_timer = self.create_timer(self.dt, self.control_loop)
        
#         # self.get_logger().info("The QP controller node is running.")

#         # Add a small delay before starting the control loop to let TF buffer populate
#         self.startup_delay = 2.0  # seconds
#         self.control_started = False
        
#         # Start with a timer that waits for TF to be available
#         self.startup_timer = self.create_timer(0.1, self.check_tf_availability)
        
#         self.get_logger().info("The QP controller node is running. Waiting for TF data...")

#     def check_tf_availability(self):
#         """Check if TF frames are available before starting main control loop"""
#         try:
#             # Try to get a transform to see if TF is working
#             trans = self.tf_buffer_.lookup_transform(
#                 "odom", "base_link", 
#                 rclpy.time.Time(), 
#                 timeout=rclpy.duration.Duration(seconds=0.1)
#             )
            
#             # If we get here, TF is working
#             self.get_logger().info("TF frames are available. Starting control loop...")
            
#             # Stop the startup timer and start the control loop
#             self.startup_timer.cancel()
#             self.control_loop_timer = self.create_timer(self.dt, self.control_loop)
#             self.control_started = True
            
#         except Exception as e:
#             # TF not ready yet, keep waiting
#             self.get_logger().debug(f"Still waiting for TF: {e}")
#             pass

        

#     def control_loop(self):
#         try:
#             # Get current state from TF
#             trans = self.tf_buffer_.lookup_transform("odom", 'base_link', rclpy.time.Time())
            
#             self.x1_ = trans.transform.translation.x
#             self.x2_ = trans.transform.translation.y

#             q = trans.transform.rotation
#             self.x3_ = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

#             # Current output and derivatives
#             y_current = np.array([self.x1_, self.x2_])
#             dy_current = np.array([self.x4_ * np.cos(self.x3_), self.x4_ * np.sin(self.x3_)])
            
#             # Solve QP for optimal control
#             success = self.solve_qp_control(y_current, dy_current)
            
#             if success:
#                 # Apply control to update velocity (integration)
#                 self.x4_ += self.u2_ * self.dt  # v += v_dot * dt
                
#                 # Publish control commands
#                 self.publish_control_commands()
                
#                 # Log state information
#                 self.get_logger().info(
#                     f"State: x={self.x1_:.3f}, y={self.x2_:.3f}, theta={self.x3_:.3f}, v={self.x4_:.3f} | "
#                     f"Controls: u1={self.u1_:.3f}, u2={self.u2_:.3f}"
#                 )
#             else:
#                 self.get_logger().warn("QP solver failed, using previous control inputs")
#                 self.publish_control_commands()

#         except Exception as e:
#             self.get_logger().warn(f"Control loop failed: {e}")
#             # Publish zero commands in case of error
#             self.publish_zero_commands()

#     def solve_qp_control(self, y_current, dy_current):
#         """
#         Solve QP optimization for control inputs
#         Variables: z = [u_1, u_2, delta1, delta2]
#         where u_1 = theta_dot, u_2 = v_dot, delta1, delta2 are slack variables
#         """
#         try:
#             # Define QP cost function: minimize z^T * P * z + q^T * z
#             q = np.array([0, 0, 
#                          0.001*(self.yd_[0] - y_current[0]) + 0.001*(self.yd_[1] - y_current[1]), 
#                          0])
            
#             # RHS of constraints (feedback linearization)
#             rhs1 = (self.ddyd_[0] + 10 * (self.dyd_[0] - dy_current[0]) + 5 * (self.yd_[0] - y_current[0]))
#             rhs2 = (self.ddyd_[1] + 10 * (self.dyd_[1] - dy_current[1]) + 5 * (self.yd_[1] - y_current[1]))
            
#             # Equality constraint matrix A @ z = b
#             # From feedback linearization: A * [u1, u2, delta1, delta2]^T = [rhs1, rhs2]^T
#             A = np.array([
#                 [-self.x4_*np.sin(self.x3_),  np.cos(self.x3_), 1, 0],
#                 [ self.x4_*np.cos(self.x3_),  np.sin(self.x3_), 0, 1]
#             ])
#             b = np.array([rhs1, rhs2])
            
#             # Convert to sparse format for efficiency
#             P_sparse = sp.csc_matrix(self.P)
#             A_sparse = sp.csc_matrix(A)
            
#             # Solve QP using OSQP
#             z_val = solve_qp(P_sparse, q, A=A_sparse, b=b, solver='osqp', verbose=False)
            
#             if z_val is not None:
#                 # Extract control inputs
#                 self.u1_ = float(z_val[0])  # theta_dot
#                 self.u2_ = float(z_val[1])  # v_dot
#                 return True
#             else:
#                 return False
                
#         except Exception as e:
#             self.get_logger().error(f"QP solver error: {e}")
#             return False

#     def publish_control_commands(self):
#         """Publish control commands to cmd_vel topic"""
#         cmd = Twist()
        
#         # Linear velocity
#         cmd.linear.x = float(self.x4_)
#         cmd.linear.y = 0.0
#         cmd.linear.z = 0.0

#         # Angular velocity (this is the control input u1)
#         cmd.angular.x = 0.0
#         cmd.angular.y = 0.0
#         cmd.angular.z = float(self.u1_)  # theta_dot

#         self.cmd_vel_publisher_.publish(cmd)

#     def publish_zero_commands(self):
#         """Publish zero velocity commands in case of error"""
#         cmd = Twist()
#         cmd.linear.x = 0.0
#         cmd.linear.y = 0.0
#         cmd.linear.z = 0.0
#         cmd.angular.x = 0.0
#         cmd.angular.y = 0.0
#         cmd.angular.z = 0.0
#         self.cmd_vel_publisher_.publish(cmd)

#     def set_desired_trajectory(self, yd, dyd=None, ddyd=None):
#         """
#         Update desired trajectory (can be called externally or via service/topic)
#         """
#         self.yd_ = np.array(yd)
#         if dyd is not None:
#             self.dyd_ = np.array(dyd)
#         if ddyd is not None:
#             self.ddyd_ = np.array(ddyd)

#     def quaternion_to_yaw(self, x, y, z, w):
#         """Convert quaternion (x, y, z, w) to yaw angle"""
#         siny_cosp = 2.0 * (w * z + x * y)
#         cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
#         return np.arctan2(siny_cosp, cosy_cosp)


# def main(args=None):
#     rclpy.init(args=args)
#     # Parse command line arguments
#     desired_x = 0.0  # default value
#     desired_y = 5.0  # default value
    
#     # Get command line arguments (skip the first one which is the script name)
#     if len(sys.argv) >= 3:
#         try:
#             desired_x = float(sys.argv[1])
#             desired_y = float(sys.argv[2])
#             print(f"Using command line arguments: desired_x={desired_x}, desired_y={desired_y}")
#         except ValueError:
#             print("Error: Arguments must be valid float numbers. Using default values (0.0, 5.0)")
#     elif len(sys.argv) == 2:
#         print("Error: Please provide both x and y coordinates. Usage: ros2 run controllers fblqp_controller <x> <y>")
#         print("Using default values (0.0, 5.0)")
#     else:
#         print("No arguments provided. Using default desired position (0.0, 5.0)")
#         print("Usage: ros2 run controllers fblqp_controller <desired_x> <desired_y>")
#     node = QpControllerNode(desired_x, desired_y)
#     rclpy.spin(node)
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
import numpy as np
from qpsolvers import solve_qp
import scipy.sparse as sp
import sys
import math


class QpControllerNode(Node): 
    def __init__(self, desired_x=0.0, desired_y=5.0):
        super().__init__("qp_controller") 

        # Controller parameters
        self.dt = 0.01  # 100 Hz control loop
        
        # Robot state: [x, y, theta, v]
        self.x1_ = 0.0  # x position
        self.x2_ = 0.0  # y position  
        self.x3_ = 0.0  # theta (orientation)
        self.x4_ = 0.0  # v (linear velocity)
        
        # Control inputs
        self.u1_ = 0.0  # angular velocity control (theta_dot)
        self.u2_ = 0.0  # linear acceleration control (v_dot)
        
        # Desired trajectory (set from command line arguments)
        self.yd_ = np.array([2.0, 1.5])      # desired position
        self.dyd_ = np.array([0.0, 0.0])     # desired velocity
        self.ddyd_ = np.array([0.0, 0.0])    # desired acceleration
        
        # QP solver matrices (will be updated in control loop)
        self.P = np.diag([1, 1, 100, 100])  # Cost matrix for [u_1, u_2, delta1, delta2]
        
        # ROS2 setup
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        # Subscribe to robot pose from Gazebo bridge
        self.pose_subscription = self.create_subscription(
            PoseArray,
            '/robot_pose',
            self.pose_callback,
            10
        )
        
        # Store current robot pose
        self.current_robot_pose = None
        self.pose_received = False

        # Add a small delay before starting the control loop to let pose data arrive
        self.startup_delay = 2.0  # seconds
        self.control_started = False
        
        # Start with a timer that waits for pose to be available
        self.startup_timer = self.create_timer(0.1, self.check_pose_availability)
        
        self.get_logger().info("The QP controller node is running. Waiting for pose data...")

    def pose_callback(self, msg):
        """Callback to update robot pose from the first pose in the array"""
        if len(msg.poses) > 0:
            # Get the first pose (which is your robot's pose)
            robot_pose = msg.poses[0]
            
            self.current_robot_pose = robot_pose
            self.pose_received = True
            
            # Update robot state from pose
            self.x1_ = robot_pose.position.x
            self.x2_ = robot_pose.position.y
            
            # Convert quaternion to yaw
            self.x3_ = self.quaternion_to_yaw(
                robot_pose.orientation.x,
                robot_pose.orientation.y,
                robot_pose.orientation.z,
                robot_pose.orientation.w
            )
            
            # Log for debugging (at debug level to avoid spam)
            self.get_logger().debug(f'Robot position: x={self.x1_:.3f}, y={self.x2_:.3f}, yaw={self.x3_:.3f}')

    def check_pose_availability(self):
        """Check if pose data is available before starting main control loop"""
        if self.pose_received:
            # If we get here, pose data is working
            self.get_logger().info("Pose data is available. Starting control loop...")
            
            # Stop the startup timer and start the control loop
            self.startup_timer.cancel()
            self.control_loop_timer = self.create_timer(self.dt, self.control_loop)
            self.control_started = True
        else:
            # Pose not ready yet, keep waiting
            self.get_logger().debug("Still waiting for pose data...")

    def control_loop(self):
        try:
            # Check if we have valid pose data
            if not self.pose_received or self.current_robot_pose is None:
                self.get_logger().warn("No pose data available, skipping control loop")
                self.publish_zero_commands()
                return

            # Current output and derivatives
            y_current = np.array([self.x1_, self.x2_])
            dy_current = np.array([self.x4_ * np.cos(self.x3_), self.x4_ * np.sin(self.x3_)])
            
            # Solve QP for optimal control
            success = self.solve_qp_control(y_current, dy_current)
            
            if success:
                # Apply control to update velocity (integration)
                self.x4_ += self.u2_ * self.dt  # v += v_dot * dt
                
                # Publish control commands
                self.publish_control_commands()
                
                # Log state information
                self.get_logger().info(
                    f"State: x={self.x1_:.3f}, y={self.x2_:.3f}, theta={self.x3_:.3f}, v={self.x4_:.3f} | "
                    f"Controls: u1={self.u1_:.3f}, u2={self.u2_:.3f}"
                )
            else:
                self.get_logger().warn("QP solver failed, using previous control inputs")
                self.publish_control_commands()

        except Exception as e:
            self.get_logger().warn(f"Control loop failed: {e}")
            # Publish zero commands in case of error
            self.publish_zero_commands()

    def solve_qp_control(self, y_current, dy_current):
        """
        Solve QP optimization for control inputs
        Variables: z = [u_1, u_2, delta1, delta2]
        where u_1 = theta_dot, u_2 = v_dot, delta1, delta2 are slack variables
        """
        try:
            if abs(self.yd_[0] - y_current[0])<1e-2 and abs(self.yd_[1] - y_current[1])<1e-2:
                self.yd_ = np.array([0.0, 3.0]) 
            # Define QP cost function: minimize z^T * P * z + q^T * z
            q = np.array([0, 0, 
                         0.01*(self.yd_[0] - y_current[0]) + 0.01*(self.yd_[1] - y_current[1]), 
                         0])
            
            # RHS of constraints (feedback linearization)
            rhs1 = (self.ddyd_[0] + 12 * (self.dyd_[0] - dy_current[0]) + 5 * (self.yd_[0] - y_current[0]))
            rhs2 = (self.ddyd_[1] + 12 * (self.dyd_[1] - dy_current[1]) + 12 * (self.yd_[1] - y_current[1]))
            
            # Equality constraint matrix A @ z = b
            # From feedback linearization: A * [u1, u2, delta1, delta2]^T = [rhs1, rhs2]^T
            A = np.array([
                [-self.x4_*np.sin(self.x3_),  np.cos(self.x3_), 1, 0],
                [ self.x4_*np.cos(self.x3_),  np.sin(self.x3_), 0, 1]
            ])
            b = np.array([rhs1, rhs2])
            
            # Convert to sparse format for efficiency
            P_sparse = sp.csc_matrix(self.P)
            A_sparse = sp.csc_matrix(A)
            
            # Solve QP using OSQP
            z_val = solve_qp(P_sparse, q, A=A_sparse, b=b, solver='osqp', verbose=False)
            
            if z_val is not None:
                # Extract control inputs
                self.u1_ = float(z_val[0])  # theta_dot
                self.u2_ = float(z_val[1])  # v_dot
                return True
            else:
                return False
                
        except Exception as e:
            self.get_logger().error(f"QP solver error: {e}")
            return False

    def publish_control_commands(self):
        """Publish control commands to cmd_vel topic"""
        cmd = Twist()
        
        # Linear velocity
        cmd.linear.x = float(self.x4_)
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        # Angular velocity (this is the control input u1)
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = float(self.u1_)  # theta_dot

        self.cmd_vel_publisher_.publish(cmd)

    def publish_zero_commands(self):
        """Publish zero velocity commands in case of error"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher_.publish(cmd)

    def set_desired_trajectory(self, yd, dyd=None, ddyd=None):
        """
        Update desired trajectory (can be called externally or via service/topic)
        """
        self.yd_ = np.array(yd)
        if dyd is not None:
            self.dyd_ = np.array(dyd)
        if ddyd is not None:
            self.ddyd_ = np.array(ddyd)

    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion (x, y, z, w) to yaw angle"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    # Parse command line arguments
    desired_x = 0.0  # default value
    desired_y = 3.0  # default value
    
    # Get command line arguments (skip the first one which is the script name)
    if len(sys.argv) >= 3:
        try:
            desired_x = float(sys.argv[1])
            desired_y = float(sys.argv[2])
            print(f"Using command line arguments: desired_x={desired_x}, desired_y={desired_y}")
        except ValueError:
            print("Error: Arguments must be valid float numbers. Using default values (0.0, 5.0)")
    elif len(sys.argv) == 2:
        print("Error: Please provide both x and y coordinates. Usage: ros2 run controllers fblqp_controller <x> <y>")
        print("Using default values (0.0, 5.0)")
    else:
        print("No arguments provided. Using default desired position (0.0, 5.0)")
        print("Usage: ros2 run controllers fblqp_controller <desired_x> <desired_y>")
    node = QpControllerNode(desired_x, desired_y)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()