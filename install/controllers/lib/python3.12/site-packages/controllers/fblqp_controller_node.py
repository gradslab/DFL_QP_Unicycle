#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float64MultiArray
import numpy as np
from qpsolvers import solve_qp
import scipy.sparse as sp
import sys

class QpControllerNode(Node): 
    def __init__(self, desired_x=0.0, desired_y=0.0, desired_theta=0.0):
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
        self.yd_ = np.array([desired_x, desired_y])      # desired position
        self.dyd_ = np.array([0.0, 0.0])     # desired velocity
        self.ddyd_ = np.array([0.0, 0.0])    # desired acceleration
        # 💡 Wrap the desired theta on initialization
        self.desired_theta_ = self.wrap_to_pi(desired_theta)
        
        # QP solver matrices
        self.P = np.diag([10.0, 10.0, 100000.0, 100000.0])  # Cost matrix for [u_1, u_2, delta1, delta2]
        
        # ROS2 setup
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.pose_subscription = self.create_subscription(
            PoseArray, '/robot_pose', self.pose_callback, 10
        )
        
        # Data logging publishers
        self.robot_state_publisher_ = self.create_publisher(
            Float64MultiArray, '/robot_state', 10)
        self.control_inputs_publisher_ = self.create_publisher(
            Float64MultiArray, '/control_inputs', 10)
        self.desired_trajectory_publisher_ = self.create_publisher(
            Float64MultiArray, '/desired_trajectory', 10)
        self.tracking_error_publisher_ = self.create_publisher(
            Float64MultiArray, '/tracking_error', 10)
        self.slack_variables_publisher_ = self.create_publisher(
            Float64MultiArray, '/slack_variables', 10)
        self.cost_publisher_ = self.create_publisher(
            Float64MultiArray, '/cost_function', 10)
        
        self.current_robot_pose = None
        self.pose_received = False
        
        # Logging variables
        self.delta1_val = 0.0
        self.delta2_val = 0.0
        self.cost_value = 0.0

        self.control_started = False
        self.startup_timer = self.create_timer(0.1, self.check_pose_availability)
        
        self.get_logger().info(f"QP controller running. Target: ({desired_x}, {desired_y}), Theta: {self.desired_theta_:.2f} rad. Waiting for pose...")

    def wrap_to_pi(self, x):
        """Wrap an angle in radians to the range [-pi, pi]."""
        return (x + np.pi) % (2 * np.pi) - np.pi

    def pose_callback(self, msg):
        """Callback to update robot pose from the first pose in the array"""
        if len(msg.poses) > 0:
            robot_pose = msg.poses[0]
            self.current_robot_pose = robot_pose
            self.pose_received = True
            
            self.x1_ = robot_pose.position.x
            self.x2_ = robot_pose.position.y
            
            # Convert quaternion to yaw and immediately wrap it
            yaw = self.quaternion_to_yaw(
                robot_pose.orientation.x, robot_pose.orientation.y,
                robot_pose.orientation.z, robot_pose.orientation.w
            )
            # 💡 Apply wrap_to_pi to the current orientation state
            self.x3_ = self.wrap_to_pi(yaw)

    def check_pose_availability(self):
        """Check if pose data is available before starting main control loop"""
        if self.pose_received:
            self.get_logger().info("Pose data is available. Starting control loop...")
            self.startup_timer.cancel()
            self.control_loop_timer = self.create_timer(self.dt, self.control_loop)
            self.control_started = True
        else:
            self.get_logger().debug("Still waiting for pose data...")

    def control_loop(self):
        try:
            if not self.pose_received or self.current_robot_pose is None:
                self.get_logger().warn("No pose data available, skipping control loop")
                self.publish_zero_commands()
                return

            y_current = np.array([self.x1_, self.x2_])
            dy_current = np.array([self.x4_ * np.cos(self.x3_), self.x4_ * np.sin(self.x3_)])
            
            success = self.solve_qp_control(y_current, dy_current)
              
            if success:
                self.x4_ += self.u2_ * self.dt  # v += v_dot * dt
                self.x4_ = np.clip(self.x4_, -0.26, 0.26)
                self.publish_control_commands()
                self.publish_logging_data(y_current)
                
                if not hasattr(self, '_log_counter'): self._log_counter = 0
                self._log_counter += 1
                
                if self._log_counter % 100 == 0:
                    self.get_logger().info(
                        f"State: x={self.x1_:.3f}, y={self.x2_:.3f}, θ={self.x3_:.3f}, v={self.x4_:.3f} | "
                        f"Controls: u1={self.u1_:.3f}, u2={self.u2_:.3f} | Error: {np.linalg.norm(self.yd_ - y_current):.3f}"
                    )
            else:
                self.get_logger().warn("QP solver failed, using previous control inputs")
                self.publish_control_commands()
                self.publish_logging_data(y_current)

        except Exception as e:
            self.get_logger().error(f"Control loop failed: {e}")
            self.publish_zero_commands()

    def solve_qp_control(self, y_current, dy_current):
        """Solve QP optimization for control inputs."""
        try:
            # 💡 Calculate orientation error using wrapped angles
            theta_error = self.wrap_to_pi(self.x3_ - self.desired_theta_)
            
            # Define QP cost function: minimize z^T * P * z + q^T * z
            q = np.array([
                10 * theta_error, 
                0.5 * (self.yd_[0] - y_current[0]) + 0.5 * (self.yd_[1] - y_current[1]), 
                0.0, 
                0.0
            ])
            
            # RHS of constraints (feedback linearization)
            rhs1 = (self.ddyd_[0] + 3 * (self.dyd_[0] - dy_current[0]) + 2 * (self.yd_[0] - y_current[0]))
            rhs2 = (self.ddyd_[1] + 7 * (self.dyd_[1] - dy_current[1]) + 12 * (self.yd_[1] - y_current[1]))
            
            # Equality constraint matrix A @ z = b
            A = np.array([
                [-self.x4_ * np.sin(self.x3_), np.cos(self.x3_), -1.0, 0.0],
                [ self.x4_ * np.cos(self.x3_), np.sin(self.x3_), 0.0, -1.0]
            ])
            b = np.array([rhs1, rhs2])
            
            # Convert to sparse format for efficiency
            P_sparse = sp.csc_matrix(self.P)
            A_sparse = sp.csc_matrix(A)
            
            z_val = solve_qp(P_sparse, q, A=A_sparse, b=b, solver='osqp', verbose=False)
            
            if z_val is not None:
                self.u1_ = np.clip(float(z_val[0]), -1.82, 1.82)
                self.u2_ = np.clip(float(z_val[1]), -1.0, 1.0)
                self.delta1_val = float(z_val[2])
                self.delta2_val = float(z_val[3])
                self.cost_value = 0.5 * z_val.T @ self.P @ z_val + q.T @ z_val
                return True
            else:
                return False
                
        except Exception as e:
            self.get_logger().error(f"QP solver error: {e}")
            return False

    def publish_logging_data(self, y_current):
        """Publish all data needed for later plotting"""
        # Robot state
        self.robot_state_publisher_.publish(Float64MultiArray(data=[self.x1_, self.x2_, self.x3_, self.x4_]))
        # Control inputs
        self.control_inputs_publisher_.publish(Float64MultiArray(data=[self.u1_, self.u2_]))
        # Desired trajectory
        self.desired_trajectory_publisher_.publish(Float64MultiArray(data=[self.yd_[0], self.yd_[1], self.dyd_[0], self.dyd_[1], self.ddyd_[0], self.ddyd_[1]]))
        # Tracking error
        error = self.yd_ - y_current
        self.tracking_error_publisher_.publish(Float64MultiArray(data=[error[0], error[1], np.linalg.norm(error)]))
        # Slack variables
        self.slack_variables_publisher_.publish(Float64MultiArray(data=[self.delta1_val, self.delta2_val]))
        # Cost function value
        self.cost_publisher_.publish(Float64MultiArray(data=[self.cost_value]))

    def publish_control_commands(self):
        """Publish control commands to cmd_vel topic"""
        cmd = Twist()
        cmd.linear.x = self.x4_
        cmd.angular.z = self.u1_
        self.cmd_vel_publisher_.publish(cmd)

    def publish_zero_commands(self):
        """Publish zero velocity commands in case of error"""
        self.cmd_vel_publisher_.publish(Twist())

    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion (x, y, z, w) to yaw angle"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    desired_x, desired_y, desired_theta = 0.0, 0.0, 0.0
    
    if len(sys.argv) >= 4:
        try:
            desired_x = float(sys.argv[1])
            desired_y = float(sys.argv[2])
            desired_theta = float(sys.argv[3])
            print(f"Using arguments: x={desired_x}, y={desired_y}, theta={desired_theta} rad")
        except ValueError:
            print("Error: Arguments must be valid numbers. Using defaults (0, 0, 0).")
    else:
        print("Usage: ros2 run <pkg> <node> <x> <y> <theta_rad>")
        print("Using default target (0.0, 0.0, 0.0)")

    node = QpControllerNode(desired_x, desired_y, desired_theta)
    
    print("="*60)
    print("ROS2 QP CONTROLLER WITH WRAP-TO-PI")
    print("="*60)
    print("Topics published for ros2 bag recording:")
    print("  /robot_state, /control_inputs, /desired_trajectory,")
    print("  /tracking_error, /slack_variables, /cost_function")
    print("\nTo record data, run in another terminal:")
    print("ros2 bag record -a")
    print("="*60)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down controller...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()