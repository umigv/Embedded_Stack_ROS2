import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  
import serial


class LEDSubscriber(Node):
    def __init__(self):
        super().__init__('LED_subscriber') 

        # Declare a subscription to 'is_auto' topic, which publishes Int32 messages
        self.subscription = self.create_subscription(
            Int32,
            'is_auto',
            self.is_auto_callback,
            10
        )
        
        # Initialize serial communication with Arduino
        self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)

    def is_auto_callback(self, msg):
            if msg.data == 1:
                self.serial_port.write(b'1')
            else:
                self.serial_port.write(b'0')


def main(args=None):
    rclpy.init(args=args)
    
    # Create the node and spin
    led_subscriber = LEDSubscriber()

    try:
        rclpy.spin(led_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Close the serial connection when shutting down
        led_subscriber.serial_port.close()
        led_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
