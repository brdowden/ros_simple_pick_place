#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from PyQt5.QtWidgets import QMainWindow, QApplication, QVBoxLayout, QLabel, QPushButton, QWidget
from PyQt5.QtCore import Qt
from std_msgs.msg import String
import subprocess

class MyGUI(QWidget):
    def __init__(self):
        super(MyGUI, self).__init__()
  
        layout = QVBoxLayout()
        self.setLayout(layout)

        # Add the first label
        self.label1 = QLabel("box starting position")
        layout.addWidget(self.label1)

        # Add the second label
        self.label2 = QLabel("Box drop zone")
        layout.addWidget(self.label2)

        # Add the third label
        self.label3 = QLabel("Box Current Position")
        layout.addWidget(self.label3)

         # Add the forth label
        self.label4 = QLabel("Box Current Orientation")
        layout.addWidget(self.label4)

        # Add the first button
        self.button1 = QPushButton("set joints to random position")
        self.button1.clicked.connect(self.on_button1_clicked)
        layout.addWidget(self.button1)

        # Add the second button
        self.button2 = QPushButton("run pick & place")
        self.button2.clicked.connect(self.on_button2_clicked)
        layout.addWidget(self.button2)

        # Add the Third button
        self.button3 = QPushButton("enable box pose updates")
        self.button3.clicked.connect(self.on_button3_clicked)
        layout.addWidget(self.button3)
       
        self.setWindowTitle("Pick & Place")
        self.setGeometry(100, 100, 400, 250)
        self.show()

        # Subscribe to the ROS topic
        self.subscriber = rospy.Subscriber("/spawn_point", String, self.callback1)
        self.subscriber = rospy.Subscriber("/drop_zone", String, self.callback2)
        self.subscriber = rospy.Subscriber("/box_position", String, self.callback3)
        self.subscriber = rospy.Subscriber("/box_orientation", String, self.callback4)

    def callback1(self, msg):
        # Update the label with the new data
        self.label1.setText(msg.data)

    def callback2(self, msg):
        # Update the label with the new data
        self.label2.setText(msg.data)
    
    def callback3(self, msg):
        # Update the label with the new data
        self.label3.setText("The box position is: " + msg.data)

    def callback4(self, msg):
        # Update the label with the new data
        self.label4.setText("The box orientation is: " + msg.data)

    def on_button1_clicked(self):
        # Handle button 1 clicked event
        # run random joint positions
        subprocess.Popen(['python3', '/home/ros/ws_moveit/src/simple_pick_place/scripts/random_joints.py'] )

    def on_button2_clicked(self):
        # Handle button 2 clicked event
        # run pick & place
        subprocess.Popen(['python3', '/home/ros/ws_moveit/src/simple_pick_place/scripts/pick_place.py'] )

    def on_button3_clicked(self):
        # Handle button 3 clicked event
        # turn on ros publisher for the joint space
        subprocess.Popen(['python3', '/home/ros/ws_moveit/src/simple_pick_place/scripts/publisher_box.py'] )

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("my_gui")

    # Start the GUI application
    app = QApplication([])
    gui = MyGUI()
    gui.show()
    app.exec_()

