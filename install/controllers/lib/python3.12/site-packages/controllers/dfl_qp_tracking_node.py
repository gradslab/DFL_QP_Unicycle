#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float64MultiArray
import numpy as np
from qpsolvers import solve_qp
import scipy.sparse as sp
import sys

class TrajectoryTrackingQpController(Node):
    def __init__(self, total_duration=50.0, trajectory_period=25.0):
        super().__init__("trajectory_tracking_qp_controller")

        # Controller parameters
        self.dt = 0.01  # 100 Hz control loop
        self.T = total_duration      # Total experiment duration
        self.t_s = trajectory_period # Time to complete one trajectory cycle (controls speed)

        # Robot state: [x, y, theta, v]
        self.x1_ = 0.0  # x position
        self.x2_ = 0.0  # y position
        self.x3_ = 0.0  # theta (orientation)
        self.x4_ = 0.0  # v (linear velocity)

        # Control inputs
        self.u1_ = 0.0  # angular velocity control (omega)
        self.u2_ = 0.0  # linear acceleration control (a)

        # QP solver matrices (from new implementation)
        # Weights for [u_omega, u_a, delta1, delta2]
        self.P = np.diag([10.0, 1.0, 10000.0, 10000.0])

        # Control limits (from new implementation)
        self.omega_max = 1.82
        self.v_max = 0.26
        self.v_dot_max = 1.0
        self.u1_min, self.u1_max = -self.omega_max, self.omega_max
        self.u2_min, self.u2_max = -self.v_dot_max, self.v_dot_max

        # Experiment timing
        self.start_time = None
        self.experiment_running = False
        self.experiment_complete = False

        # ROS2 setup
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        # Subscribe to robot pose from Gazebo bridge
        self.pose_subscription = self.create_subscription(
            PoseArray,
            '/robot_pose',
            self.pose_callback,
            10
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
        self.slack_variables_publisher_ = self.create_publisher(
            Float64MultiArray, '/slack_variables', 10)
        self.experiment_info_publisher_ = self.create_publisher(
            Float64MultiArray, '/experiment_info', 10)

        # Store current robot pose
        self.current_robot_pose = None
        self.pose_received = False

        # Logging variables
        self.delta1_val = 0.0
        self.delta2_val = 0.0
        self.experiment_time = 0.0

        # Wait for pose data before starting
        self.startup_timer = self.create_timer(0.1, self.check_pose_availability)

        self.get_logger().info("Trajectory Tracking QP Controller initialized.")
        self.get_logger().info(f"Experiment duration: {self.T}s, Trajectory period: {self.t_s}s")
        self.get_logger().info("Waiting for pose data to start experiment...")

    def get_lemniscate_trajectory(self, t):
        """
        Generates a figure-eight (Lemniscate of Gerono) trajectory.
        The trajectory speed is controlled by self.t_s.
        """
        # Scale simulation time `t` to trajectory parameter `tau`
        k = (2.0 * np.pi) / self.t_s
        tau = k * (t % self.t_s)  # Use modulo to handle multiple cycles

        # Pre-calculate sin and cos for efficiency
        s_tau = np.sin(tau)
        c_tau = np.cos(tau)

        # Position (y_d)
        xd = s_tau**2
        yd = (s_tau**2) * c_tau
        pos_d = np.array([xd, yd])

        # Velocity (dy_d/dt) - using the chain rule: d/dt = d/dtau * k
        vx_dtau = 2.0 * s_tau * c_tau
        vy_dtau = 2.0 * s_tau * c_tau**2 - s_tau**3
        vel_d = np.array([vx_dtau * k, vy_dtau * k])

        # Acceleration (d^2y_d/dt^2) - using the chain rule: d^2/dt^2 = d^2/dtau^2 * k^2
        ax_dtau = 2.0 * (c_tau**2 - s_tau**2)  # Equivalent to 2*cos(2*tau)
        ay_dtau = 2.0 * c_tau**3 - 7.0 * (s_tau**2) * c_tau
        accel_d = np.array([ax_dtau * (k**2), ay_dtau * (k**2)])

        return pos_d, vel_d, accel_d

    def pose_callback(self, msg):
        """Callback to update robot pose from the first pose in the array"""
        if len(msg.poses) > 0:
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

    def check_pose_availability(self):
        """Check if pose data is available before starting experiment"""
        if self.pose_received:
            self.get_logger().info("Pose data available. Starting trajectory tracking experiment...")

            # Stop the startup timer and start the control loop
            self.startup_timer.cancel()

            # Initialize experiment
            self.start_time = self.get_clock().now()
            self.experiment_running = True

            # Set initial velocity to zero
            self.x4_ = 0.0

            self.control_loop_timer = self.create_timer(self.dt, self.control_loop)
        else:
            self.get_logger().debug("Still waiting for pose data...")

    def control_loop(self):
        try:
            if not self.experiment_running or self.experiment_complete:
                return

            if not self.pose_received or self.current_robot_pose is None:
                self.get_logger().warn("No pose data available, skipping control loop")
                self.publish_zero_commands()
                return

            # Calculate experiment time
            current_time = self.get_clock().now()
            time_diff = current_time - self.start_time
            self.experiment_time = time_diff.nanoseconds * 1e-9

            # Check if experiment is complete
            if self.experiment_time >= self.T:
                if not self.experiment_complete:
                    self.get_logger().info("Experiment completed! Stopping robot...")
                    self.experiment_complete = True
                    self.publish_zero_commands()
                return

            # Get desired trajectory at current time
            yd, dyd, ddyd = self.get_lemniscate_trajectory(self.experiment_time)

            # Current output and derivatives
            y_current = np.array([self.x1_, self.x2_])
            dy_current = np.array([self.x4_ * np.cos(self.x3_), self.x4_ * np.sin(self.x3_)])

            # Solve QP for optimal control
            success = self.solve_qp_control(y_current, dy_current, yd, dyd, ddyd)

            if success:
                # Apply control to update velocity (integration)
                self.x4_ += self.u2_ * self.dt  # v += a * dt
                self.x4_ = np.clip(self.x4_, -self.v_max, self.v_max)

                # Publish control commands
                self.publish_control_commands()

                # Publish data for logging
                self.publish_logging_data(y_current, yd, dyd, ddyd)

                # Log state information (reduced frequency)
                if not hasattr(self, '_log_counter'):
                    self._log_counter = 0
                self._log_counter += 1

                if self._log_counter % 100 == 0:  # Log every 1 second at 100Hz
                    error_norm = np.linalg.norm(yd - y_current)
                    progress = (self.experiment_time / self.T) * 100
                    self.get_logger().info(
                        f"Progress: {progress:.1f}% | "
                        f"State: x={self.x1_:.3f}, y={self.x2_:.3f}, θ={self.x3_:.3f}, v={self.x4_:.3f} | "
                        f"Error: {error_norm:.4f}m"
                    )
            else:
                self.get_logger().warn("QP solver failed, using previous control inputs")
                self.publish_control_commands() # Publish old commands to be safe
                self.publish_logging_data(y_current, yd, dyd, ddyd)

        except Exception as e:
            self.get_logger().error(f"Control loop failed: {e}")
            self.publish_zero_commands()

    def solve_qp_control(self, y_current, dy_current, yd, dyd, ddyd):
        """
        Solve QP optimization for control inputs
        Variables: z = [u_omega, u_a, delta1, delta2]
        """
        try:
            # Linear term in objective (currently zero as per simulation code)
            epsilon = 0.0 * (y_current[0] - yd[0]) + 0.0 * (y_current[1] - yd[1])
            q = np.array([0.0, epsilon, 0.0, 0.0])

            # Equality constraints: A_eq * z = b_eq (feedback linearization with slack)
            # Use updated gains
            Kp_val = 32.0/8
            Kd_val = 67.2/8
            
            rhs1 = (ddyd[0] + Kd_val * (dyd[0] - dy_current[0]) +
                    Kp_val * (yd[0] - y_current[0]))
            rhs2 = (ddyd[1] + Kd_val * (dyd[1] - dy_current[1]) +
                    Kp_val * (yd[1] - y_current[1]))

            # Constraint matrix for z = [u_omega, u_a, delta1, delta2]
            A_eq = np.array([
                [-self.x4_ * np.sin(self.x3_), np.cos(self.x3_), -1, 0],
                [ self.x4_ * np.cos(self.x3_), np.sin(self.x3_), 0, -1]
            ])
            b_eq = np.array([rhs1, rhs2])

            # Convert to sparse format for efficiency
            P_sparse = sp.csc_matrix(self.P)
            A_sparse = sp.csc_matrix(A_eq)

            # Solve QP using OSQP
            z_val = solve_qp(P_sparse, q, A=A_sparse, b=b_eq, solver='osqp', verbose=False)

            if z_val is not None:
                # Extract control inputs
                self.u1_ = float(z_val[0])  # omega
                self.u2_ = float(z_val[1])  # acceleration
                self.delta1_val = float(z_val[2])
                self.delta2_val = float(z_val[3])

                # Apply control limits
                self.u1_ = np.clip(self.u1_, self.u1_min, self.u1_max)
                self.u2_ = np.clip(self.u2_, self.u2_min, self.u2_max)

                return True
            else:
                return False

        except Exception as e:
            self.get_logger().error(f"QP solver error: {e}")
            return False

    def publish_logging_data(self, y_current, yd, dyd, ddyd):
        """Publish all data needed for later plotting"""
        # Robot state: [x, y, theta, v]
        robot_state_msg = Float64MultiArray()
        robot_state_msg.data = [float(self.x1_), float(self.x2_), float(self.x3_), float(self.x4_)]
        self.robot_state_publisher_.publish(robot_state_msg)

        # Control inputs: [u1, u2]
        control_msg = Float64MultiArray()
        control_msg.data = [float(self.u1_), float(self.u2_)]
        self.control_inputs_publisher_.publish(control_msg)

        # Desired trajectory: [x_d, y_d, dx_d, dy_d, ddx_d, ddy_d]
        desired_msg = Float64MultiArray()
        desired_msg.data = [float(yd[0]), float(yd[1]),
                            float(dyd[0]), float(dyd[1]),
                            float(ddyd[0]), float(ddyd[1])]
        self.desired_trajectory_publisher_.publish(desired_msg)

        # Tracking error: [x_error, y_error, norm_error]
        x_error = yd[0] - y_current[0]
        y_error = yd[1] - y_current[1]
        norm_error = np.linalg.norm([x_error, y_error])
        error_msg = Float64MultiArray()
        error_msg.data = [float(x_error), float(y_error), float(norm_error)]
        self.tracking_error_publisher_.publish(error_msg)

        # Slack variables: [delta1, delta2]
        slack_msg = Float64MultiArray()
        slack_msg.data = [float(self.delta1_val), float(self.delta2_val)]
        self.slack_variables_publisher_.publish(slack_msg)

        # Experiment info: [current_time, progress_percent, experiment_complete]
        progress = (self.experiment_time / self.T) * 100.0
        experiment_info_msg = Float64MultiArray()
        experiment_info_msg.data = [float(self.experiment_time), float(progress),
                                      float(1.0 if self.experiment_complete else 0.0)]
        self.experiment_info_publisher_.publish(experiment_info_msg)

    def publish_control_commands(self):
        """Publish control commands to cmd_vel topic"""
        cmd = Twist()
        cmd.linear.x = float(self.x4_)
        cmd.angular.z = float(self.u1_)
        self.cmd_vel_publisher_.publish(cmd)

    def publish_zero_commands(self):
        """Publish zero velocity commands"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher_.publish(cmd)

    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion (x, y, z, w) to yaw angle"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)

    # Default parameters from the simulation script
    duration = 50.0
    trajectory_period = 25.0

    # Parse command line arguments: <duration> <trajectory_period>
    if len(sys.argv) >= 2:
        try:
            duration = float(sys.argv[1])
        except ValueError:
            print(f"Error: Duration must be a float. Using default {duration}s")
    
    if len(sys.argv) >= 3:
        try:
            trajectory_period = float(sys.argv[2])
        except ValueError:
            print(f"Error: Trajectory period must be a float. Using default {trajectory_period}s")

    node = TrajectoryTrackingQpController(duration, trajectory_period)

    print("=" * 70)
    print("ROS2 FIGURE-EIGHT TRAJECTORY TRACKING QP CONTROLLER")
    print("=" * 70)
    print(f"Experiment Duration (T): {duration}s")
    print(f"Trajectory Period (t_s): {trajectory_period}s (controls speed)")
    print("")
    print("Trajectory: Lemniscate of Gerono (Figure-Eight)")
    print("  x_d(t) = sin²(τ)")
    print("  y_d(t) = sin²(τ) * cos(τ), where τ = (2π/t_s) * t")
    print("")
    print("To run with custom parameters:")
    print("ros2 run <your_package> <your_node_name> <duration> <trajectory_period>")
    print("Example: ros2 run my_controllers dfl_qp_tracking 60 30")
    print("")
    print("Topics published for ros2 bag recording:")
    print("  /robot_state        - [x, y, theta, v]")
    print("  /control_inputs     - [u1_omega, u2_accel]")
    print("  /desired_trajectory - [xd, yd, d_xd, d_yd, dd_xd, dd_yd]")
    print("  /tracking_error     - [x_err, y_err, norm_err]")
    print("  /slack_variables    - [delta1, delta2]")
    print("  /experiment_info    - [time, progress_%, complete_flag]")
    print("")
    print("To record data, run in another terminal:")
    print("ros2 bag record /robot_state /control_inputs /desired_trajectory /tracking_error /slack_variables /experiment_info")
    print("=" * 70)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nExperiment interrupted by user...")
    finally:
        node.get_logger().info("Shutting down trajectory tracking controller...")
        node.publish_zero_commands()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()