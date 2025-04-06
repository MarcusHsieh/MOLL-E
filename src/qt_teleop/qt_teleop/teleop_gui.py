#!/usr/bin/env python3
import sys
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout, QSpacerItem, QSizePolicy

class TeleopNode(Node):
    def __init__(self):
        super().__init__('qt_teleop_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_filtered', 10)

    def publish_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: linear.x={linear_x}, angular.z={angular_z}')

class TeleopGUI(QWidget):
    def __init__(self, teleop_node):
        super().__init__()
        self.node = teleop_node
        self.init_ui()

    def init_ui(self):
        grid = QGridLayout()
        self.setLayout(grid)

        # Row 0: Forward commands (3 buttons)
        btn_forward_left = QPushButton('Forward Left')
        btn_forward = QPushButton('Forward')
        btn_forward_right = QPushButton('Forward Right')
        
        grid.addWidget(btn_forward_left, 0, 0)
        grid.addWidget(btn_forward, 0, 1)
        grid.addWidget(btn_forward_right, 0, 2)

        # Row 1: Neutral command (one button centered)
        btn_neutral = QPushButton('Neutral')
        # Optionally add spacers on left and right:
        spacer_left = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
        spacer_right = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
        grid.addItem(spacer_left, 1, 0)
        grid.addWidget(btn_neutral, 1, 1)
        grid.addItem(spacer_right, 1, 2)

        # Row 2: Backward commands (3 buttons)
        btn_backward_left = QPushButton('Backward Left')
        btn_backward = QPushButton('Backward')
        btn_backward_right = QPushButton('Backward Right')
        
        grid.addWidget(btn_backward_left, 2, 0)
        grid.addWidget(btn_backward, 2, 1)
        grid.addWidget(btn_backward_right, 2, 2)

        # Connect button clicks to publishing functions
        # Adjust twist values as needed:
        btn_forward_left.clicked.connect(lambda: self.node.publish_twist(0.5, 1.0))
        btn_forward.clicked.connect(lambda: self.node.publish_twist(0.5, 0.0))
        btn_forward_right.clicked.connect(lambda: self.node.publish_twist(0.5, -1.0))
        btn_neutral.clicked.connect(lambda: self.node.publish_twist(0.0, 0.0))
        btn_backward_left.clicked.connect(lambda: self.node.publish_twist(-0.5, 1.0))
        btn_backward.clicked.connect(lambda: self.node.publish_twist(-0.5, 0.0))
        btn_backward_right.clicked.connect(lambda: self.node.publish_twist(-0.5, -1.0))

        self.setWindowTitle('Teleop Joystick')
        self.show()

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()

    app = QApplication(sys.argv)
    gui = TeleopGUI(teleop_node)

    # Spin ROS in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(teleop_node,), daemon=True)
    spin_thread.start()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
