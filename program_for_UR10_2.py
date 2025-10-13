import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        try:
            # Publishers
            self.script_publisher_2_: Publisher = self.create_publisher(
                String, 'ur10_2/urscript_interface/script_command', 10
            )
            self.status_publisher_2_: Publisher = self.create_publisher(
                String, '/robot_status_2', 10
            )
             # Subscribers
            self.turtlebot_subscriber = self.create_subscription(
                String, '/turtlebot_status', self.turtlebot_callback, 10
            )
            self.turtlebot_subscriber_2 = self.create_subscription(
                String, '/turtlebot_status_2', self.turtlebot_callback_2, 10
            )
            
            # Robot poses (poses 5-12 in the main program)
            self.target_poses = [
                "[0, -1.57, 0, -1.57, 0, 0]",
                "[0.68, -0.72, 1.51, -2.36, -1.52, 2.46]", 
                "[0.68, -0.93, 1.45, -2.09, -1.52, 2.46]",
                "[3.07, -0.718, 1.04, -1.99, -1.57, 1.61]",
                "[3.07, -0.76, 0.96, -1.85, -1.57, 1.61]",
                "[2.9, -0.76, 0.95, -1.84, -1.57, 1.34]",
                "[2.9, -0.71, 1.03, -1.97, -1.57, 1.34]",
                "[1.54, -1.15, 1.76, -2.24, -1.53, 1.57]",
                "[1.54, -0.86, 1.83, -2.61, -1.53, 1.57]",
            ]
            
            self.home_pose = "[0, -1.57, 0, -1.57, 0, 0]"

            self.timer = self.create_timer(0.5, self.timer_callback)
            self.has_moved_home_2 = False
            self.robot_ready = False
            self.waiting_for_turtlebot = True
            self.object_loaded_2 = False
            self.turtlebot_msg_received_2 = False
            self.turtlebot_msg_received_final = False  # New flag for final trigger
            self.sequence_2_finished = False  # Flag to track the second sequence
            self.time_to_publish_finished_loading_2 = 0.0
            self.delay_2 = 5.0  #delay for UR10_2
            self.ur10_2_ready_published = False  # Flag to indicate if the message has been published
            self.ur10_2_ready_start_time = 0.0  # Time when the message was first published
            self.ur10_2_ready_duration = 10.0  # Minimum duration to publish the message
            self.publish_finished_loading_2_flag = False
            
            time.sleep(1)
            self.publish_ready_message_2()
        except Exception as e:
            self.get_logger().error(f"__init__ error: {e}")
            
    def publish_ready_message_2(self):
        try:
            status_msg = String()
            status_msg.data = "UR10_2 is ready to start."
            self.status_publisher_2_.publish(status_msg)
            self.get_logger().info("Published to /robot_status_2: UR10_2 is ready to start.")
        except Exception as e:
            self.get_logger().error(f"publish_ready_message_2 error: {e}")
            
    def turtlebot_callback_2(self, msg: String):
        try:
            self.get_logger().info(f"Received from TurtleBot 2: {msg.data}")
            if "Turtlebot reached delivery location." in msg.data and not self.turtlebot_msg_received_2:
                self.get_logger().info("UR10_2: Turtlebot reached delivery location.")
                status_msg = String()
                status_msg.data = "UR10_2: picking delivered object."
                self.status_publisher_2_.publish(status_msg)
                self.get_logger().info("Published to /robot_status_2: UR10_2 is picking the object")
                self.move_robot(self.target_poses[6], robot_number=2)
                time.sleep(15)
                self.move_robot(self.target_poses[5], robot_number=2)
                time.sleep(5)
                self.move_robot(self.target_poses[6], robot_number=2)
                time.sleep(5)
                self.move_robot(self.target_poses[8], robot_number=2)
                time.sleep(15)
                self.move_robot(self.target_poses[7], robot_number=2)
                time.sleep(5)
                self.move_robot(self.target_poses[8], robot_number=2)
                time.sleep(5)

                # NEW STEP: Publish ready for final device
                status_msg = String()
                status_msg.data = "UR10_2 is ready to collect final device."
                self.status_publisher_2_.publish(status_msg)
                self.get_logger().info("Published: UR10_2 is ready to collect final device.")
                self.ur10_2_ready_published = True  # Set the flag
                self.ur10_2_ready_start_time = time.time()  # Store the time

                self.turtlebot_msg_received_2 = True
                self.waiting_for_turtlebot = False

            elif "Turtlebot 2 reached picking place." in msg.data and not self.turtlebot_msg_received_final:
                self.get_logger().info("UR10_2: Final trigger received. Resuming final sequence.")
                self.move_robot(self.target_poses[9], robot_number=2)
                time.sleep(5)
                self.move_robot(self.target_poses[10], robot_number=2)
                time.sleep(5)
                self.move_robot(self.target_poses[9], robot_number=2)
                time.sleep(5)
                self.move_robot(self.target_poses[11], robot_number=2)
                time.sleep(10)
                self.move_robot(self.target_poses[12], robot_number=2)
                time.sleep(5)
                self.move_robot(self.target_poses[11], robot_number=2)
                time.sleep(5)

                self.object_loaded_2 = True
                self.turtlebot_msg_received_final = True
                self.time_to_publish_finished_loading_2 = time.time()
                self.sequence_2_finished = True  # set the flag
                self.has_moved_home_2 = False  # Reset flag before going home
                self.publish_finished_loading_2_flag = True
        except Exception as e:
            self.get_logger().error(f"turtlebot_callback_2 error: {e}")
            
    def timer_callback(self):
        try:
            if self.waiting_for_turtlebot:
                self.publish_ready_message()
            elif self.object_loaded_2 and time.time() - self.time_to_publish_finished_loading_2 < self.delay_2:
                status_msg = String()
                status_msg.data = "UR10_2: Finished loading second object."
                self.status_publisher_2_.publish(status_msg)
                self.get_logger().info("Published to /robot_status_2: UR10_2 finished loading second object.")
            elif self.object_loaded_2 and time.time() - self.time_to_publish_finished_loading_2 >= self.delay_2:
                if not self.has_moved_home_2:
                    self.move_robot(self.home_pose, robot_number=2)
                    time.sleep(5)
                    self.has_moved_home_2 = True
            # Check if UR10_2 ready message needs to be republished
            if self.ur10_2_ready_published and time.time() - self.ur10_2_ready_start_time < self.ur10_2_ready_duration:
                status_msg = String()
                status_msg.data = "UR10_2 is ready to collect final device."
                self.status_publisher_2_.publish(status_msg)
                self.get_logger().info("Published: UR10_2 is ready to collect final device.")
            elif self.ur10_2_ready_published and time.time() - self.ur10_2_ready_start_time >= self.ur10_2_ready_duration:
                self.ur10_2_ready_published = False  # stop publishing.
                if self.sequence_2_finished and not self.has_moved_home_2:
                    status_msg = String()
                    status_msg.data = "UR10_2: Finished loading second object."  # Corrected message.  moved here
                    self.status_publisher_2_.publish(status_msg)
                    self.get_logger().info("Published to /robot_status_2: UR10_2 finished loading second object.")
                    self.move_robot(self.home_pose, robot_number=2)
                    time.sleep(5)
                    self.has_moved_home_2 = True
            #publish message multiple times
            if self.publish_finished_loading_2_flag:
                status_msg = String()
                status_msg.data = "UR10_2: Finished loading second object."
                self.status_publisher_2_.publish(status_msg)
                self.get_logger().info("Published to /robot_status_2: UR10_2 finished loading second object.")
        except Exception as e:
            self.get_logger().error(f"timer_callback error: {e}")
            
    def move_robot(self, pose_string: str, robot_number: int):
        try:
            command = f'def my_prog():\n' \
                      f'  movej({pose_string}, a=1.2, v=0.25, r=0)\n' \
                      f'  textmsg("motion finished")\n' \
                      f'end\n'
            msg = String()
            msg.data = command
            elif robot_number == 2:
                self.script_publisher_2_.publish(msg)
                self.get_logger().info(
                    f"Published command to ur10_2/urscript_interface/script_command: {command}")
            else:
                self.get_logger().warning(f"Invalid robot number: {robot_number}")
        except Exception as e:
            self.get_logger().error(f"move_robot error: {e}")

def main(args=None):
    try:
        rclpy.init(args=args)
        move_robot_node = MoveRobot()
        rclpy.spin(move_robot_node)
        move_robot_node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"Main error: {e}")

if __name__ == '__main__':
    main()
