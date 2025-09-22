#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float64MultiArray
import numpy as np
import sys

class TrajectoryTrackingPseudoInverseController(Node):
    def __init__(self, total_duration=50.0, trajectory_period=25.0):
        super().__init__("trajectory_tracking_pinv_controller")

        # Controller parameters
        self.dt = 0.01  # 100 Hz control loop
        self.T = total_duration      # Total experiment duration
        self.t_s = trajectory_period # Time to complete one trajectory cycle

        # Controller Gains and Parameters
        self.Kp = np.diag([32.0/4, 32.0/4])
        self.Kd = np.diag([67.2/4, 67.2/4])
        self.lambda_reg = 0.05  # Regularization factor for pseudo-inverse

        # Robot state: [x, y, theta, v]
        self.x1_ = 0.0  # x position
        self.x2_ = 0.0  # y position
        self.x3_ = 0.0  # theta (orientation)
        self.x4_ = 0.0  # v (linear velocity)

        # Control inputs
        self.u1_ = 0.0  # angular velocity control (omega)
        self.u2_ = 0.0  # linear acceleration control (a)

        # Control limits
        self.omega_max = 1.82
        self.v_max = 0.26
        self.v_dot_max = 1.0
        self.u1_min, self.u1_max = -self.omega_max, self.omega_max
        self.u2_min, self.u2_max = -self.v_dot_max, self.v_dot_max
        
        # Singularity avoidance parameters
        self.v_min = 0.02
        self.v_reset = 0.06
        self.v_reset_smooth_tau = 0.05

        # Experiment timing
        self.start_time = None
        self.experiment_running = False
        self.experiment_complete = False

        # ROS2 setup
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.pose_subscription = self.create_subscription(
            PoseArray, '/robot_pose', self.pose_callback, 10
        )

        # Data logging publishers for ros2 bag
        self.robot_state_publisher_ = self.create_publisher(
            Float64MultiArray, '/robot_state', 10)
        self.control_inputs_publisher_ = self.create_publisher(
            Float64MultiArray, '/control_inputs', 10)
        self.desired_trajectory_publisher_ = self.create_publisher(
            Float64MultiArray, '/desired_trajectory', 10)
        self.tracking_error_publisher_ = self.create_publisher(
            Float64MultiArray, '/tracking_error', 10)
        self.experiment_info_publisher_ = self.create_publisher(
            Float64MultiArray, '/experiment_info', 10)

        # Store current robot pose
        self.current_robot_pose = None
        self.pose_received = False
        self.experiment_time = 0.0

        # Wait for pose data before starting
        self.startup_timer = self.create_timer(0.1, self.check_pose_availability)

        self.get_logger().info("Pseudo-Inverse DFL Controller initialized.")
        self.get_logger().info(f"Experiment duration: {self.T}s, Trajectory period: {self.t_s}s")
        self.get_logger().info("Waiting for pose data to start experiment...")

    def get_lemniscate_trajectory(self, t):
        """
        Generates a figure-eight (Lemniscate of Gerono) trajectory.
        """
        k = (2.0 * np.pi) / self.t_s
        tau = k * (t % self.t_s)
        s_tau, c_tau = np.sin(tau), np.cos(tau)

        # Position (y_d)
        pos_d = np.array([s_tau**2, (s_tau**2) * c_tau])
        # Velocity (dy_d/dt)
        vx_dtau = 2.0 * s_tau * c_tau
        vy_dtau = 2.0 * s_tau * c_tau**2 - s_tau**3
        vel_d = np.array([vx_dtau * k, vy_dtau * k])
        # Acceleration (d^2y_d/dt^2)
        ax_dtau = 2.0 * (c_tau**2 - s_tau**2)
        ay_dtau = 2.0 * c_tau**3 - 7.0 * (s_tau**2) * c_tau
        accel_d = np.array([ax_dtau * (k**2), ay_dtau * (k**2)])

        return pos_d, vel_d, accel_d

    def pose_callback(self, msg):
        """Callback to update robot pose from the first pose in the array"""
        if len(msg.poses) > 0:
            robot_pose = msg.poses[0]
            if not self.pose_received:
                self.get_logger().info("First robot pose received.")
            self.current_robot_pose = robot_pose
            self.pose_received = True

            self.x1_ = robot_pose.position.x
            self.x2_ = robot_pose.position.y
            self.x3_ = self.quaternion_to_yaw(
                robot_pose.orientation.x, robot_pose.orientation.y,
                robot_pose.orientation.z, robot_pose.orientation.w
            )

    def check_pose_availability(self):
        """Check if pose data is available and initialize the experiment."""
        if self.pose_received:
            self.get_logger().info("Pose data available. Starting experiment...")
            self.startup_timer.cancel()
            
            # Initialize velocity to avoid singularity at t=0
            _, dyd0, _ = self.get_lemniscate_trajectory(0.0)
            dx0, dy0 = dyd0
            v_d0 = np.sqrt(dx0**2 + dy0**2)
            if v_d0 < 1e-6:
                v_init = np.sign(dx0) * self.v_min if np.abs(dx0) > 1e-6 else self.v_min
            else:
                v_init = np.sign(v_d0) * max(self.v_min, v_d0)
            self.x4_ = v_init
            self.get_logger().info(f"Initial velocity set to {self.x4_:.3f} m/s to avoid singularity.")

            self.start_time = self.get_clock().now()
            self.experiment_running = True
            self.control_loop_timer = self.create_timer(self.dt, self.control_loop)
        else:
            self.get_logger().debug("Still waiting for initial pose data...")

    def control_loop(self):
        try:
            if not self.experiment_running or self.experiment_complete:
                return

            current_time = self.get_clock().now()
            self.experiment_time = (current_time - self.start_time).nanoseconds * 1e-9

            if self.experiment_time >= self.T:
                if not self.experiment_complete:
                    self.get_logger().info("Experiment completed! Stopping robot...")
                    self.experiment_complete = True
                    self.publish_zero_commands()
                return

            # Get desired trajectory
            yd, dyd, ddyd = self.get_lemniscate_trajectory(self.experiment_time)
            y_current = np.array([self.x1_, self.x2_])
            dy_current = np.array([self.x4_ * np.cos(self.x3_), self.x4_ * np.sin(self.x3_)])

            # Calculate control inputs
            self.u1_, self.u2_ = self.calculate_pseudo_inverse_control(
                y_current, dy_current, yd, dyd, ddyd
            )
            
            # Integrate dynamics (update velocity)
            self.x4_ += self.u2_ * self.dt  # v += a * dt
            
            # Singularity safeguard
            if np.abs(self.x4_) < self.v_min:
                _, dyd_next, _ = self.get_lemniscate_trajectory(self.experiment_time + self.dt)
                v_d_next = np.linalg.norm(dyd_next)
                reset_sign = np.sign(v_d_next) if v_d_next > 1e-6 else np.sign(self.x4_) if self.x4_ != 0 else 1.0
                target_v = reset_sign * self.v_reset
                # Smoothly transition to the reset velocity
                alpha = self.dt / (self.v_reset_smooth_tau + self.dt)
                self.x4_ = (1 - alpha) * self.x4_ + alpha * target_v

            # Enforce velocity limits
            self.x4_ = np.clip(self.x4_, -self.v_max, self.v_max)
            
            # Publish commands and logs
            self.publish_control_commands()
            self.publish_logging_data(y_current, yd, dyd, ddyd)

            # Log progress periodically
            if not hasattr(self, '_log_counter'): self._log_counter = 0
            self._log_counter += 1
            if self._log_counter % 100 == 0:
                error_norm = np.linalg.norm(yd - y_current)
                progress = (self.experiment_time / self.T) * 100
                self.get_logger().info(f"Progress: {progress:.1f}% | State: [x={self.x1_:.2f}, y={self.x2_:.2f}, θ={self.x3_:.2f}, v={self.x4_:.2f}] | Error: {error_norm:.3f}m")

        except Exception as e:
            self.get_logger().error(f"Control loop failed: {e}")
            self.publish_zero_commands()
            
    def calculate_pseudo_inverse_control(self, y_current, dy_current, yd, dyd, ddyd):
        """Calculates control inputs using regularized pseudo-inverse DFL."""
        # Virtual control (PD + feedforward)
        w = ddyd + self.Kd @ (dyd - dy_current) + self.Kp @ (yd - y_current)
        
        # Jacobian Matrix
        v = self.x4_
        theta = self.x3_
        J = np.array([
            [-v * np.sin(theta), np.cos(theta)],
            [ v * np.cos(theta), np.sin(theta)]
        ])
        
        # Regularized pseudo-inverse
        JTJ = J.T @ J
        reg = (self.lambda_reg**2) * np.eye(2)
        # Add a small identity matrix if condition number is too high, for numerical stability
        extra = 1e-6 * np.eye(2) if np.linalg.cond(JTJ + reg) > 1e12 else np.zeros((2,2))
        
        J_pinv = np.linalg.inv(JTJ + reg + extra) @ J.T
        u_cmd = J_pinv @ w  # [omega, v_dot]
        
        # Apply control limits
        u1 = np.clip(u_cmd[0], self.u1_min, self.u1_max) # omega
        u2 = np.clip(u_cmd[1], -self.v_dot_max, self.v_dot_max) # acceleration
        
        return u1, u2

    def publish_logging_data(self, y_current, yd, dyd, ddyd):
        """Publish all data needed for later plotting."""
        # Robot state
        self.robot_state_publisher_.publish(Float64MultiArray(data=[self.x1_, self.x2_, self.x3_, self.x4_]))
        # Control inputs
        self.control_inputs_publisher_.publish(Float64MultiArray(data=[self.u1_, self.u2_]))
        # Desired trajectory
        self.desired_trajectory_publisher_.publish(Float64MultiArray(data=[yd[0], yd[1], dyd[0], dyd[1], ddyd[0], ddyd[1]]))
        # Tracking error
        error = yd - y_current
        self.tracking_error_publisher_.publish(Float64MultiArray(data=[error[0], error[1], np.linalg.norm(error)]))
        # Experiment info
        progress = (self.experiment_time / self.T) * 100.0
        self.experiment_info_publisher_.publish(Float64MultiArray(data=[self.experiment_time, progress, 1.0 if self.experiment_complete else 0.0]))

    def publish_control_commands(self):
        cmd = Twist()
        cmd.linear.x = self.x4_
        cmd.angular.z = self.u1_
        self.cmd_vel_publisher_.publish(cmd)

    def publish_zero_commands(self):
        self.cmd_vel_publisher_.publish(Twist())

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    duration = 50.0
    trajectory_period = 25.0

    if len(sys.argv) >= 2:
        try: duration = float(sys.argv[1])
        except ValueError: print(f"Error: Duration must be a float. Using default {duration}s")
    if len(sys.argv) >= 3:
        try: trajectory_period = float(sys.argv[2])
        except ValueError: print(f"Error: Trajectory period must be a float. Using default {trajectory_period}s")

    node = TrajectoryTrackingPseudoInverseController(duration, trajectory_period)

    print("=" * 70)
    print("ROS2 PSEUDO-INVERSE DFL CONTROLLER (FIGURE-EIGHT)")
    print("=" * 70)
    print(f"Experiment Duration (T): {duration}s")
    print(f"Trajectory Period (t_s): {trajectory_period}s (controls speed)")
    print("\nThis controller uses a regularized pseudo-inverse of the Jacobian")
    print("and includes singularity handling for near-zero velocities.")
    print("\nTo run with custom parameters:")
    print("ros2 run <your_package> <your_node_name> <duration> <trajectory_period>")
    print("\nTopics published for ros2 bag recording:")
    print("  /robot_state        - [x, y, theta, v]")
    print("  /control_inputs     - [u1_omega, u2_accel]")
    print("  /desired_trajectory - [xd, yd, d_xd, d_yd, dd_xd, dd_yd]")
    print("  /tracking_error     - [x_err, y_err, norm_err]")
    print("  /experiment_info    - [time, progress_%, complete_flag]")
    print("\nTo record data, run in another terminal:")
    print("ros2 bag record /robot_state /control_inputs /desired_trajectory /tracking_error /experiment_info")
    print("=" * 70)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nExperiment interrupted by user...")
    finally:
        node.get_logger().info("Shutting down controller...")
        node.publish_zero_commands()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()