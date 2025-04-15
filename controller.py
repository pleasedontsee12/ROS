import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.node import Node
# Assuming 'pop' is a custom library installed for controlling the specific robot (SerBot)
# If you don't have this library or are using a different robot,
# you'll need to replace Pilot.SerBot() and its methods with appropriate ones.
try:
    from pop import Pilot
except ImportError:
    print("Warning: 'pop' library not found. Robot control functionality will not work.")
    print("You might need to install it or adapt the code for your robot hardware.")
    # Define dummy classes/methods if needed for testing without the library
    class DummyPilot:
        def SerBot(self):
            return self # Return self to allow method chaining like Pilot.SerBot()
        def stop(self):
            print("[DummyPilot] stop()")
        def setSpeed(self, speed):
            print(f"[DummyPilot] setSpeed({speed})")
        def turnRight(self):
            print("[DummyPilot] turnRight()")
        def turnLeft(self):
            print("[DummyPilot] turnLeft()")
        def forward(self, speed):
            print(f"[DummyPilot] forward({speed})")
        # Add a dummy steering attribute if needed
        steering = 0.0
    Pilot = DummyPilot() # Use the dummy class if 'pop' is not available


class SubscriberNode(Node):
    def __init__(self):
        super().__init__('pop_controller_node') # Node name as in the OCR
        qos = QoSProfile(depth=10)

        # Initialize robot control object
        try:
            # This assumes Pilot.SerBot() initializes and returns the robot object
            self.bot = Pilot.SerBot()
            # Add a steering attribute if the library doesn't provide one directly
            if not hasattr(self.bot, 'steering'):
                 self.bot.steering = 0.0 # Initialize steering state if needed
            self.get_logger().info('SerBot controller initialized.')
        except Exception as e:
            self.get_logger().error(f"Failed to initialize SerBot: {e}")
            self.bot = None # Indicate that the bot is not available

        # Create subscription
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',       # Topic name to subscribe to
            self.callback,   # Callback function to execute upon receiving a message
            qos              # QoS profile
        )
        # self.subscription # This line in OCR doesn't do anything, removed.
        self.get_logger().info('Controller node subscribing to cmd_vel.')


    def callback(self, msg):
        if self.bot is None:
            self.get_logger().warn("Robot object not initialized, skipping command.")
            return

        # Extract linear speed (forward/backward) and angular speed (steering)
        # Note: Standard Twist uses linear.x for forward and angular.z for rotation.
        # The OCR code maps angular.z to steer, with a negative sign.
        speed = msg.linear.x
        steer = -msg.angular.z # Negative sign as per OCR logic

        self.get_logger().info(f'Received cmd_vel - Speed: {speed:.2f}, Steer: {steer:.2f}')

        # --- Robot Control Logic based on OCR ---
        # Use a small tolerance for floating point comparisons
        speed_tolerance = 1e-3
        steer_tolerance = 1e-3

        if abs(speed) <= speed_tolerance and abs(steer) <= steer_tolerance:
            # If both speed and steer are effectively zero, stop the robot
            self.bot.stop()
            self.get_logger().info('Command: Stop')
        else:
            # Logic when steering dominates (absolute steer value >= 0.6)
            if abs(steer) >= 0.6:
                # OCR sets speed to 2, likely a fixed turning speed
                self.bot.setSpeed(2)
                if steer > 0: # Positive steer in this logic means turn right
                    self.bot.turnRight()
                    self.get_logger().info('Command: Turn Right (High Steer)')
                elif steer < 0: # Negative steer means turn left
                    self.bot.turnLeft()
                    self.get_logger().info('Command: Turn Left (High Steer)')
                # Note: If steer is exactly 0.6 or -0.6, it might fall here.
                # If steer is exactly 0 but speed is non-zero, it falls to the 'else' below.
            else:
                # Logic for forward/backward movement with proportional steering
                # The OCR scales speed and steer. Adjust these formulas as needed for your robot.
                # Speed scaling: speed * 15 + 1 * sign(speed) -> Clamped between -6 and 6
                # Ensure speed is not zero before using sign
                speed_sign = np.sign(speed) if speed != 0 else 0
                scaled_speed = speed * 15 + 1 * speed_sign
                clamped_speed = min(max(scaled_speed, -6), 6) # Clamp between -6 and 6

                # Steering scaling: steer * 2.0 -> Clamped between -1 and 1
                scaled_steer = steer * 2.0
                clamped_steer = min(max(scaled_steer, -1), 1) # Clamp between -1 and 1

                self.bot.forward(clamped_speed)
                # Assuming the 'pop' library or the SerBot object handles steering proportionally
                # The OCR assigns to self.bot.steering, implying it's used internally by forward() or similar
                self.bot.steering = clamped_steer # Update steering value
                self.get_logger().info(f'Command: Forward ({clamped_speed:.2f}), Steer ({clamped_steer:.2f})')


def main(args=None):
    rclpy.init(args=args)
    print("Starting Controller Node...")
    try:
        controller_subscriber = SubscriberNode()
        rclpy.spin(controller_subscriber)
    except Exception as e:
        print(f"An error occurred during node execution: {e}")
    finally:
        # Ensure node is destroyed and rclpy is shut down even if errors occur
        if 'controller_subscriber' in locals() and controller_subscriber:
            controller_subscriber.destroy_node()
        rclpy.shutdown()
        print("Controller Node stopped.")

if __name__ == '__main__':
    main()