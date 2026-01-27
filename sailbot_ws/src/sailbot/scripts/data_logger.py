#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import os
import time
from datetime import datetime

from std_msgs.msg import Float32, Bool, UInt8

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        # Create logs directory if it doesn't exist
        self.log_dir = os.path.expanduser('~/sailbot_logs')
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        # Create new log file with timestamp
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.log_file_path = os.path.join(self.log_dir, f'log_{timestamp}.csv')
        self.csv_file = open(self.log_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Define columns
        self.headers = [
            'timestamp',
            'armed',
            'heading_mag_deg',
            'roll_deg',
            'yaw_rate_deg_s',
            'wind_apparent_deg',
            'rudder_cmd_deg',
            'sheet_cmd_pct'
        ]
        self.csv_writer.writerow(self.headers)
        self.csv_file.flush()

        # State storage
        self.current_data = {
            'armed': False,
            'heading_mag_deg': 0.0,
            'roll_deg': 0.0,
            'yaw_rate_deg_s': 0.0,
            'wind_apparent_deg': 0.0,
            'rudder_cmd_deg': 0.0,
            'sheet_cmd_pct': 0.0
        }

        # Subscriptions
        # IMU
        self.create_subscription(Float32, '/imu/heading_mag_deg', self.heading_cb, 10)
        self.create_subscription(Float32, '/imu/roll_deg', self.roll_cb, 10)
        self.create_subscription(Float32, '/imu/yaw_rate_deg_s', self.yaw_rate_cb, 10)
        
        # Wind
        self.create_subscription(Float32, '/state/wind_apparent', self.wind_cb, 10)
        
        # Actuators
        self.create_subscription(Float32, '/actuators/rudder_cmd_deg', self.rudder_cb, 10)
        self.create_subscription(Float32, '/actuators/sheet_cmd_pct', self.sheet_cb, 10)
        
        # System
        self.create_subscription(Bool, '/system/armed', self.armed_cb, 10)
        
        # Timer for writing to file (e.g. 10 Hz)
        self.timer = self.create_timer(0.1, self.timer_cb)
        
        self.get_logger().info(f"Data Logger started. Logging to {self.log_file_path}")

    def heading_cb(self, msg): self.current_data['heading_mag_deg'] = msg.data
    def roll_cb(self, msg): self.current_data['roll_deg'] = msg.data
    def yaw_rate_cb(self, msg): self.current_data['yaw_rate_deg_s'] = msg.data
    def wind_cb(self, msg): self.current_data['wind_apparent_deg'] = msg.data
    def rudder_cb(self, msg): self.current_data['rudder_cmd_deg'] = msg.data
    def sheet_cb(self, msg): self.current_data['sheet_cmd_pct'] = msg.data
    def armed_cb(self, msg): self.current_data['armed'] = msg.data

    def timer_cb(self):
        # Write current state to CSV
        row = [
            time.time(),
            self.current_data['armed'],
            self.current_data['heading_mag_deg'],
            self.current_data['roll_deg'],
            self.current_data['yaw_rate_deg_s'],
            self.current_data['wind_apparent_deg'],
            self.current_data['rudder_cmd_deg'],
            self.current_data['sheet_cmd_pct']
        ]
        self.csv_writer.writerow(row)
        self.csv_file.flush()

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
