from nis import cat
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import serial 
from .arduino_comm import Arduino

class ArduinoCommNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.arduino = Arduino(port="/dev/ttyACM0")

    def timer_callback(self):
        try:
            self.arduino.write([1600,1600,1600])
            data = self.arduino.read_state(fields_types = [float, int, int, int, int])
            speed = data[0]
            is_auto = data[1] == 1
            curr_throttle = data[2]
            curr_steering = data[3]
            brake = data[4]

        except Exception as e:
            print(e)
    def destroy_node(self):
        self.arduino.write([1500,1500,1500])
        super().destroy_node()
def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArduinoCommNode()
        rclpy.spin(node)
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()