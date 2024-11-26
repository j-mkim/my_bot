import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
import serial
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import quaternion_from_euler
from math import sin, cos
import threading

class ArduinoOdometryNode(Node):
    def __init__(self):
        super().__init__('arduino_odometry_node')

        # Parameters (adjust these to match your robot)
        self.counts_per_rev = 576.0      # Encoder counts per wheel revolution
        self.wheel_diameter = 0.0635     # Wheel diameter in meters
        self.wheel_base = 0.2368         # w, Distance between left and right wheels

        # Initialize serial communication
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info('Serial port opened successfully.')
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open serial port: {e}')
            rclpy.shutdown()
            return

        # ROS publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Initialize variables
        self.encoder_counts = [0, 0, 0, 0]
        self.prev_encoder_counts = [0, 0, 0, 0]
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # Start the serial reading thread
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def read_serial(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith('<') and line.endswith('>'):
                    data = line[1:-1]
                    values = data.split(',')
                    if len(values) == 4:
                        counts = [int(v) for v in values]
                        self.encoder_counts = counts
                        self.calculate_odometry()
                    else:
                        self.get_logger().warn('Invalid data length from serial: ' + line)
                else:
                    if line != '':
                        self.get_logger().warn('Invalid data format from serial: ' + line)
            except Exception as e:
                self.get_logger().error(f'Error reading serial data: {e}')

    def calculate_odometry(self):
        # Compute delta counts
        delta_counts = [self.encoder_counts[i] - self.prev_encoder_counts[i] for i in range(4)]
        self.prev_encoder_counts = self.encoder_counts.copy()

        # Convert delta counts to distance
        wheel_circumference = self.wheel_diameter * 3.1415926
        distance_per_count = wheel_circumference / self.counts_per_rev

        # Distances moved by each wheel
        distances = [delta_counts[i] * distance_per_count for i in range(4)]

        # Compute average distances for left and right wheels
        left_distance = (distances[1] + distances[2]) / 2.0  # RL and FL
        right_distance = (distances[0] + distances[3]) / 2.0 # RR and FR

        # Compute total movement
        distance = (left_distance + right_distance) / 2.0
        delta_th = (right_distance - left_distance) / self.wheel_base

        # Update position
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        self.last_time = current_time

        delta_x = distance * cos(self.th + delta_th / 2.0)
        delta_y = distance * sin(self.th + delta_th / 2.0)

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Prepare Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Set orientation
        q = quaternion_from_euler(0, 0, self.th)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Set velocities
        if dt > 0:
            odom.twist.twist.linear.x = distance / dt
            odom.twist.twist.angular.z = delta_th / dt
        else:
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.angular.z = 0.0

        # Publish odometry
        self.odom_pub.publish(odom)

    def cmd_vel_callback(self, msg):
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Compute wheel velocities
        v_left = linear_x - (angular_z * self.wheel_base / 2.0)
        v_right = linear_x + (angular_z * self.wheel_base / 2.0)

        # Convert to motor speeds (adjust scaling as needed)
        max_motor_speed = 255  # Max PWM value
        max_robot_speed = 1.0  # Max robot speed in m/s

        motor_speed_left = int((v_left / max_robot_speed) * max_motor_speed)
        motor_speed_right = int((v_right / max_robot_speed) * max_motor_speed)

        # Ensure motor speeds are within -255 to 255
        motor_speed_left = max(min(motor_speed_left, 255), -255)
        motor_speed_right = max(min(motor_speed_right, 255), -255)

        # Map to the four motors
        # 0 = RR, 1 = RL, 2 = FL, 3 = FR
        motor_speeds = [motor_speed_right, motor_speed_left, motor_speed_left, motor_speed_right]

        # Prepare command string
        command = '<{},{},{},{}>\n'.format(motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3])

        # Send command over serial
        try:
            self.ser.write(command.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Error writing to serial port: {e}')

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Replace these with actual readings from your robot
        joint_state_msg.name = [
            'front_left_wheel_joint', 
            'front_right_wheel_joint', 
            'rear_left_wheel_joint', 
            'rear_right_wheel_joint', ]
        joint_state_msg.position = [motor_speeds[2], motor_speeds[3], motor_speeds[1], motor_speeds[0]]
        #joint_state_msg.velocity = [left_wheel_velocity, right_wheel_velocity]  # Optional
        #joint_state_msg.effort = []  # Optional

        self.publisher.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
