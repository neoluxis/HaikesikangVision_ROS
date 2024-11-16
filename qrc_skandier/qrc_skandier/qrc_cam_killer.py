import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class QrcCamKiller(Node):
    def __init__(self, name="1"):
        super().__init__(f'qrc_cam_killer_{name}')
        self.qrc_res_sub = self.create_subscription(
            String,
            "qrc_result",
            self.kill_qrc,
            10,
        )
        self.qrc_kill_pub = self.create_publisher(String, "kill_qrc", 10)

    def kill_qrc(self, msg):
        """To kill all nodes to scan a qrcode. They are"""
        if msg.data == "0000000":
            self.get_logger().info("Received: 0000000, no kills")
            return
        self.get_logger().info(f"Received: {msg.data}, killing qrc* nodes")
        self.qrc_kill_pub.publish(String(data="kill"))
        self.get_logger().info("Killed all qrc* nodes")
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    qrc_cam_killer = QrcCamKiller()
    rclpy.spin(qrc_cam_killer)
    rclpy.shutdown()
