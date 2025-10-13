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
            self.script_publisher_1_: Publisher = self.create_publisher(
                String, 'ur10_1/urscript_interface/script_command', 10
            )
            self.status_publisher_1_: Publisher = self.create_publisher(
                String, '/robot_status', 10
            )
            # Subscribers
            self.turtlebot_subscriber = self.create_subscription(
                String, '/turtlebot_status', self.turtlebot_callback, 10
            )
            self.turtlebot_subscriber_2 = self.create_subscription(
                String, '/turtlebot_status_2', self.turtlebot_callback_2, 10
            )
            
             # Robot poses
            self.target_poses = [
                "[0, -1.57, 0, -1.57, 0, 0]",
                "[-0.24, -2.28, -1.50, -0.90, 1.50, 1.67]",
                "[-0.24, -2.48, -1.52, -0.69, 1.51, 1.67]",
                "[-1.54, -2.65, -0.67, -1.35, 1.52, 0]",
                "[-1.55, -2.77, -0.75, -1.15, 1.53, 0]",
            ]
            
            self.home_pose = "[0, -1.57, 0, -1.57, 0, 0]"

            self.timer = self.create_timer(0.5, self.timer_callback)
            self.has_moved_home_1 = False
            self.robot_ready = False
            self.waiting_for_turtlebot = True
            self.object_loaded_1 = False
            self.turtlebot_msg_received = False
            self.time_to_publish_finished_loading_1 = 0.0
            self.delay_1 = 5.0  #delay for UR10_1 home movement
            
            time.sleep(1)
            self.publish_ready_message()
            except Exception as e:
            self.get_logger().error(f"__init__ error: {e}")

    def publish_ready_message(self):
        try:
            status_msg = String()
            status_msg.data = "UR10_1 is ready to start."
            self.status_publisher_1_.publish(status_msg)
            self.get_logger().info("Published to /robot_status: UR10_1 is ready to start.")
            self.robot_ready = True
        except Exception as e:
            self.get_logger().error(f"publish_ready_message error: {e}")
            
    def turtlebot_callback(self, msg: String):
        try:
            self.get_logger().info(f"Received from TurtleBot: {msg.data}")
            if "Turtlebot reached picking place." in msg.data and not self.turtlebot_msg_received:
                self.get_logger().info(
                    "UR10 #1: Turtlebot reached picking place, ur10 is going to take the package")
                self.move_robot(self.target_poses[3], robot_number=1)
                time.sleep(10)
                self.move_robot(self.target_poses[4], robot_number=1)
                time.sleep(5)
                self.move_robot(self.target_poses[1], robot_number=1)
                time.sleep(10)
                self.move_robot(self.target_poses[2], robot_number=1)
                time.sleep(5)
                self.move_robot(self.target_poses[1], robot_number=1)
                time.sleep(5)
                status_msg = String()
                status_msg.data = "UR10_1: Finished loading the package."
                self.status_publisher_1_.publish(status_msg)
                self.get_logger().info("Published to /robot_status: UR10_1: Finished loading the package.")
                self.object_loaded_1 = True
                self.turtlebot_msg_received = True
                self.time_to_publish_finished_loading_1 = time.time()
            self.waiting_for_turtlebot = False
        except Exception as e:
            self.get_logger().error(f"turtlebot_callback error: {e}")

    def timer_callback(self):
        try:
            if self.waiting_for_turtlebot:
                self.publish_ready_message()
            elif self.object_loaded_1 and time.time() - self.time_to_publish_finished_loading_1 < self.delay_1:
                status_msg = String()
                status_msg.data = "UR10_1: Finished loading the package."
                self.status_publisher_1_.publish(status_msg)
                self.get_logger().info("Published to /robot_status: UR10_1: finished loading the package.")
            elif self.object_loaded_1 and time.time() - self.time_to_publish_finished_loading_1 >= self.delay_1:
                if not self.has_moved_home_1:
                    self.move_robot(self.home_pose, robot_number=1)
                    time.sleep(5)
                    self.has_moved_home_1 = True
                    
    def move_robot(self, pose_string: str, robot_number: int):
        try:
            command = f'def my_prog():\n' \
                      f'  movej({pose_string}, a=1.2, v=0.25, r=0)\n' \
                      f'  textmsg("motion finished")\n' \
                      f'end\n'
            msg = String()
            msg.data = command
            if robot_number == 1:
                self.script_publisher_1_.publish(msg)
                self.get_logger().info(
                    f"Published command to ur10_1/urscript_interface/script_command: {command}")
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
                    
                    
