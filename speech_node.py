import rclpy
from std_msgs.msg import String, Bool
import os

def text_to_speech(text):
    command = f'echo "{text}" | piper --model en_GB-southern_english_female-low'
    os.system(command)

class TextSpeakerNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('text_speaker')
        self.subscription = self.create_subscription(String, 'text', self.listener_callback, 
10)
        self.publisher_ = self.create_publisher(Bool, 'speaking', 10)

    def listener_callback(self, msg):
        text_to_speech(msg.data)
        self.get_logger().info('Speaking: "%s"' % msg.data)
        speaking = Bool()
        speaking.data = True
        self.publisher_.publish(speaking)
        speaking.data = False
        self.publisher_.publish(speaking)

def main(args=None):
    rclpy.init(args=args)
    text_speaker = TextSpeakerNode()
    rclpy.spin(text_speaker)
    text_speaker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()