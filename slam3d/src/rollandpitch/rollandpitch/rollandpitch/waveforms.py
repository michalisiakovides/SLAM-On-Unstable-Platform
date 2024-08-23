#Sine Wave 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class SineWave(Node):
    def __init__(self):
        super().__init__('sine_wave_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.2, self.timer_callback)  # Publish at 5 Hz
        self.start_time = time.time()
        self.current_twist = Twist()
        self.frequency = 0.5  # Frequency of the sine wave in Hz
        self.amplitude = 15.0  # Amplitude of the sine wave
        
    def cmd_vel_callback(self, msg):
        # Store the current Twist message 
        self.current_twist = msg
        
    def timer_callback(self):
        new_twist = self.current_twist
        
        # Calculate elapsed time
        elapsed_time = time.time() - self.start_time

        # Sine wave parameters
        amplitude_radians = math.radians(self.amplitude)  # Convert amplitude to radians

        # Calculate sine wave for angular x and angular y
        new_twist.angular.x = amplitude_radians * math.sin(2 * math.pi * self.frequency * elapsed_time + math.pi/2)
        new_twist.angular.y = amplitude_radians * math.sin(2 * math.pi * self.frequency * elapsed_time + math.pi/2)
        
     
        # Publish the message
        self.publisher_.publish(new_twist)

def main(args=None):
    rclpy.init(args=args)
    sine_wave = SineWave()
    rclpy.spin(sine_wave)
    sine_wave.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
#Square Wave

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class SquareWave(Node):
    def __init__(self):
        super().__init__('square_wave_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.2, self.timer_callback)  # Timer callback at 5 Hz
        self.square_wave_period = 2.0  # Period of the square wave in seconds
        self.phase_shift = self.square_wave_period / 4  # Phase shift corresponding to pi/2
        self.start_time = time.time()
        self.current_twist = Twist()  # Store the current Twist message

    def cmd_vel_callback(self, msg):
        # Store the current Twist message 
        self.current_twist = msg

    def timer_callback(self):
        # Get the current time using time.time()
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        # Apply the phase shift by adjusting the time used in the square wave calculation
        t = (elapsed_time + self.phase_shift) % self.square_wave_period
        half_period = self.square_wave_period / 2

        # Determine the value of the square wave based on the adjusted time
        if t < half_period:
            square_wave_value = math.radians(15.0)  # High value of the square wave
        else:
            square_wave_value = -math.radians(15.0)  # Low value of the square wave

        # Update the current twist with the phase-shifted square wave value
        new_twist = self.current_twist
        new_twist.angular.x = square_wave_value
        new_twist.angular.y = square_wave_value

        # Publish the message
        self.publisher_.publish(new_twist)

def main(args=None):
    rclpy.init(args=args)
    square_wave = SquareWave()
    rclpy.spin(square_wave)
    square_wave.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""

