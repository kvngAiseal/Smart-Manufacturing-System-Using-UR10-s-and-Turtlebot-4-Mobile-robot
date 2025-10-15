#!/usr/bin/env python3
"""
4-PHASE TURTLEBOT-UR10 COORDINATION (Extended)
1. Position 1: Pick from UR10_1
2. Position 2: Deliver to UR10_2
3. Position 3: Pick from UR10_2 
4. Position 4: Final delivery
"""

import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from std_msgs.msg import String
import time

class TurtlebotUR10Coordinator:
    def __init__(self):
        rclpy.init()
        self.navigator = TurtleBot4Navigator()
        
        # Communication setup
        self.ur10_1_pub = self.navigator.create_publisher(String, '/turtlebot_status', 10)
        self.ur10_2_pub = self.navigator.create_publisher(String, '/turtlebot_status_2', 10)
        self.ur10_sub = self.navigator.create_subscription(
            String, '/robot_status', self.ur10_callback, 10)
        self.ur10_sub_2 = self.navigator.create_subscription(
            String, '/robot_status_2', self.ur10_2_callback, 10)
        
        # State machine
        self.phase = 0  # 0=init, 1=going to P1, 2=going to P2, 3=going to P3, 4=going to P4
        self.navigator.get_logger().info("System Initialized - Waiting for UR10_1")

    def publish_repeatedly(self, publisher, message, count=5, delay=0.5):
        """Publishes multiple copies of a message"""
        for i in range(count):
            publisher.publish(String(data=message))
            self.navigator.get_logger().info(f"Published to {publisher.topic_name} ({i+1}/{count}): '{message}'")
            time.sleep(delay)

    def ur10_callback(self, msg):
        """Handles UR10_1 messages"""
        cmd = msg.data.strip()
        if self.phase == 0 and cmd == "UR10_1 is ready to start.":
            self.phase = 1
            self.navigator.get_logger().info("PHASE 1: UR10_1 start confirmed")
        elif self.phase == 1 and cmd == "UR10_1: Finished loading the package.":
            self.phase = 2
            self.navigator.get_logger().info("PHASE 2: UR10_1 loading confirmed")

    def ur10_2_callback(self, msg):
        """Handles UR10_2 messages"""
        cmd = msg.data.strip()
        if self.phase == 2 and cmd == "UR10_2 is ready to collect final device.":
            self.phase = 3
            self.navigator.get_logger().info("PHASE 3: UR10_2 ready for second pickup")
        elif self.phase == 3 and cmd == "UR10_2: Finished loading second object.":
            self.phase = 4
            self.navigator.get_logger().info("PHASE 4: UR10_2 loaded final package")

    def navigate_to_position(self, position, label):
        """Navigation to target position"""
        goal = self.navigator.getPoseStamped(position, TurtleBot4Directions.NORTH)
        self.navigator.goToPose(goal)
        
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.navigator, timeout_sec=0.1)
        
        return self.navigator.getResult()


    def run(self):
        try:
            
            # --- PHASE 1: Pick from UR10_1 ---
            self.navigator.get_logger().info("🕒 Waiting for UR10_1 start signal...")
            while self.phase < 1:
                rclpy.spin_once(self.navigator, timeout_sec=0.1)

            # --- PHASE 0: Initialization ---
            initial_pose = self.navigator.getPoseStamped([1.668, 3.319], TurtleBot4Directions.NORTH)
            self.navigator.setInitialPose(initial_pose)
            self.navigator.waitUntilNav2Active()

            if self.navigate_to_position([2.922, 5.375], "Position 1"):
                self.publish_repeatedly(self.ur10_1_pub, "Turtlebot reached picking place.")

            # --- PHASE 2: Deliver to UR10_2 ---
            self.navigator.get_logger().info("🕒 Waiting for UR10_1 completion...")
            while self.phase < 2:
                rclpy.spin_once(self.navigator, timeout_sec=0.1)

            if self.navigate_to_position([2.816, 2.832], "Position 2"):
                self.publish_repeatedly(self.ur10_2_pub, "Turtlebot reached delivery location.")

            # --- PHASE 3: Pick from UR10_2 ---
            self.navigator.get_logger().info("🕒 Waiting for UR10_2 pickup signal...")
            while self.phase < 3:
                rclpy.spin_once(self.navigator, timeout_sec=0.1)

            if self.navigate_to_position([3.460, 2.411], "Position 3"):
                self.publish_repeatedly(self.ur10_2_pub, "Turtlebot 2 reached picking place.")

            # --- PHASE 4: Final Delivery ---
            self.navigator.get_logger().info("🕒 Waiting for UR10_2 completion...")
            while self.phase < 4:
                rclpy.spin_once(self.navigator, timeout_sec=0.1)

            if self.navigate_to_position([1.176, 6.874], "Position 4"):
                self.publish_repeatedly(self.ur10_2_pub, "Turtlebot 2 reached delivery location.")
                self.navigator.get_logger().info("✅ Mission complete!")

        except Exception as e:
            self.navigator.get_logger().error(f"Error: {str(e)}")
        finally:
            self.navigator.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    coordinator = TurtlebotUR10Coordinator()
    coordinator.run()