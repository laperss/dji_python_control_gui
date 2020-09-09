#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
GUI for sending different types of commands to the DJI SDK. 

Linnea Persson
laperss@kth.se
"""
from __future__ import print_function
import sys
import os
import rospy
import math
from PyQt5 import QtCore, QtGui, QtWidgets
from dji_osdk_ros.srv import SDKControlAuthority
from sensor_msgs.msg import Joy, NavSatFix
from std_msgs.msg import Header
from threading import Thread
from flight_control_flag import *


ROS_MASTER_URI = os.environ["ROS_MASTER_URI"]



class WARA_Landing_GUI(QtWidgets.QWidget):
    """ Base GUI class for sending commands to DJI Matrice """
    publishing = False
    thread_started = False
    ctrl_auth = False
    ctrl_mode = 3
    axes = [0, 0, 0, 0, FLAG_ENU_VEL_YAWRATE]

    def __init__(self):
        QtWidgets.QWidget.__init__(self)
        self.setGeometry(0, 0, 700, 850)
        self.main_layout = QtWidgets.QGridLayout(self)

        self.rate = rospy.Rate(10)

        self.init_publishers()
        self.init_services()

        self.ns = rospy.get_namespace()

        btn_layout = QtWidgets.QGridLayout()
        exit_btn = QtWidgets.QPushButton("Exit")
        exit_btn.clicked.connect(self.closeEvent)
        exit_btn.setObjectName("menu_button")
        auth_btn = QtWidgets.QPushButton("Authorize control")
        auth_btn.clicked.connect(self.authorize)
        auth_btn.setObjectName("menu_button")

        btn_layout.addWidget(auth_btn, 0, 0, 2, 1)
        btn_layout.addWidget(exit_btn, 0, 1, 2, 1)
        self.main_layout.addLayout(btn_layout, 0, 0, 2, 4)

        setting_layout = QtWidgets.QGridLayout()
        mode_group = [['ENU position,\nyaw angle', 0],
                      ['ENU velocity,\nyawrate', 1],
                      ['Roll, pitch, \naltitude, yawrate', 2],
                      ['Roll, pitch, \nhdot, yawrate', 3]]

        control_label = QtWidgets.QLabel('Control settings')
        control_label.setObjectName("label")
        setting_layout.addWidget(control_label, 0, 0, 1, 2)
        
        ctrl_mode_group = QtWidgets.QButtonGroup(btn_layout)
        ctrl_mode_group.buttonClicked.connect(self.toggle_mode)
        j = 0
        for item in mode_group:
            btn = QtWidgets.QRadioButton(item[0])
            btn.setChecked(j == self.ctrl_mode)
            ctrl_mode_group.addButton(btn, j)
            setting_layout.addWidget(btn, int(j/2)+1, (j%2)*3, 1, 3)
            j += 1
        i = int(j/2)+1

        self.tabs = QtWidgets.QTabWidget()
        tab1 = QtWidgets.QWidget()
        tab2 = QtWidgets.QWidget()
        tab3 = QtWidgets.QWidget()
        tab4 = QtWidgets.QWidget()
        self.tabs.resize(100, 300)

        # Add tabs
        self.tabs.addTab(tab1, "ENU pos + yaw")
        self.tabs.addTab(tab2, "ENU vel + yawrate")
        self.tabs.addTab(tab3, "RPYrate + altitude")
        self.tabs.addTab(tab4, "RPYrate + hdot")
        self.tabs.setCurrentIndex(self.ctrl_mode)
        # Create first tab

        sliders = [[['Pos_x', 0, (-100, 100)],
                    ['Pos_y', 1, (-100, 100)],
                    ['Alt', 2, (0, ALT_MAX)],
                    ['Yaw', 3, (-YAW_ANGLE_LIM, YAW_ANGLE_LIM)]],
                   [['Vel_x', 0, (-HOR_VEL_LIM, HOR_VEL_LIM)],
                    ['Vel_y', 1, (-HOR_VEL_LIM, HOR_VEL_LIM)],
                    ['Vel_h', 2, (-ALT_VEL_LIM, ALT_VEL_LIM)],
                    ['Yawrate', 3, (-YAW_RATE_LIM, YAW_RATE_LIM)]],
                   [['Roll', 0, (-ROLL_ANGLE_LIM, ROLL_ANGLE_LIM)],
                    ['Pitch', 1, (-PITCH_ANGLE_LIM, PITCH_ANGLE_LIM)],
                    ['Alt', 2, (0, ALT_MAX)],
                    ['Yawrate', 3, (-YAW_RATE_LIM, YAW_RATE_LIM)]],
                   [['Roll', 0, (-ROLL_ANGLE_LIM, ROLL_ANGLE_LIM)],
                    ['Pitch', 1, (-PITCH_ANGLE_LIM, PITCH_ANGLE_LIM)],
                    ['Vel_h', 2, (-5, 5)],
                    ['Yawrate', 3, (-YAW_RATE_LIM, YAW_RATE_LIM)]]]

        j = 0
        for tab in [tab1, tab2, tab3, tab4]:
            tab.layout = QtWidgets.QGridLayout()
            tab.sliders = []
            for s in sliders[j]:
                
                slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
                slider.text = QtWidgets.QLabel('0.0')
                slider.axes = s[1]
                slider.mode = j
                if s[0] == 'Alt':
                    slider.setRange(0, s[2][1])
                    slider.scale = 1
                elif s[0] == 'Thrust':
                    slider.setRange(0, s[2][1])
                    slider.scale = 1
                else:
                    slider.setRange(-100, 100)
                    slider.scale = s[2][1]/100.0
                slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
                slider.valueChanged.connect(self.slider_moving)
                slider.setStyleSheet('height: 50px;')
                button = QtWidgets.QPushButton("Zero")
                button.clicked.connect(self.slider_zero)
                button.slider = slider

                tab.layout.addWidget(QtWidgets.QLabel(s[0]), 3+i, 0, 1, 1)
                tab.layout.addWidget(slider.text, 3+i, 1, 1, 1)
                tab.layout.addWidget(slider, 3+i, 2, 1, 4)
                tab.layout.addWidget(button, 3+i, 6, 1, 1)
                i += 1
                tab.setLayout(tab.layout)
                tab.sliders.append(slider)
            j += 1
        setting_layout.addWidget(self.tabs, i+2, 0, 4, 8)
        i = i+2

        publish_btn = QtWidgets.QPushButton("START publishing commands")
        publish_btn.setStyleSheet('height: 50px; background: #c8c3cc; font-weight: 800; font-size: 20px;')
        publish_btn.clicked.connect(self.toggle_publish)

        setting_layout.addWidget(publish_btn, 3+i+1, 0, 1, 8)

        self.main_layout.addLayout(setting_layout, 2, 0, 8, 5)

        if self.ctrl_mode == 0:
            self.axes[4] = FLAG_ENU_POS_YAW
        elif self.ctrl_mode == 1:
            self.axes[4] = FLAG_ENU_VEL_YAWRATE
        elif self.ctrl_mode == 2:
            self.axes[4] = FLAG_ROLL_PITCH_YAW
        elif self.ctrl_mode == 3:
            self.axes[4] = FLAG_ROLL_PITCH_YAW_RATE
        
        self.ctrl_thread = Thread(target=self.publish_cmd, args=(11,))        
        self.thread_started = True
        self.ctrl_thread.start()
        

    def init_publishers(self):
        rospy.loginfo("Start publisher at 'flight_control_setpoint_generic'")
        self.ctrl_pub = rospy.Publisher("flight_control_setpoint_generic",
            Joy, queue_size=10)

    def init_services(self):
        if not ROS_MASTER_URI == "http://localhost:11311":
            print("ROS_MASTER_URI = ", ROS_MASTER_URI)
            rospy.loginfo("Waiting for service: dji_sdk/sdk_control_authority")
            rospy.wait_for_service("dji_sdk/sdk_control_authority")
            rospy.loginfo("Running ROS with ROS_MASTER_URI %s" %
                          ROS_MASTER_URI)
        else:
            rospy.loginfo("Running on local machine")

        self.ctrl_auth_service = rospy.ServiceProxy(
            "dji_sdk/sdk_control_authority", SDKControlAuthority)

        rospy.loginfo("Control authority service found at 'dji_sdk/sdk_control_authority'")
    def get_position(self):
        rospy.loginfo("Wait for valid position...")
        msg = rospy.wait_for_message(
            "dji_sdk/gps_position", NavSatFix, 5.0)
        position = (msg.latitude, msg.longitude, msg.altitude)
        return position


    def closeEvent(self, event):
        if self.publishing:
            self.publishing = False

        self.thread_started = False
        self.ctrl_thread.join()
            
        rospy.loginfo("Exit application")
        QtWidgets.QApplication.quit()

    def publish_cmd(self, value):
        rospy.logdebug("Start publishing commands")
        j = 0
        while self.thread_started:
            if self.publishing:
                msg = Joy(Header(), self.axes, [])
                self.ctrl_pub.publish(msg)
                j += 1
            self.rate.sleep()
        return

    def authorize(self):
        if not self.ctrl_auth:
            rospy.logdebug("Authorize control")
            try:
                self.ctrl_auth_service(1)
            except:
                rospy.logwarn("Service not available")
            else:
                self.sender().setText("Deauthorize control")
                self.sender().setDown(1)
                self.ctrl_auth = True
        else:
            rospy.logdebug("Deauthorize control")
            try:
                self.ctrl_auth_service(0)
            except:
                rospy.logwarn("Service not available")
            else:
                self.sender().setText("Authorize control")
                self.sender().setDown(0)
                self.ctrl_auth = False

    def toggle_ctrl(self):
        self.get_position()

    def toggle_publish(self):
        if not self.publishing:
            rospy.logdebug("Start publishing")
            self.publishing = True
            self.sender().setText("Stop Publish CMD")
            self.sender().setDown(1)
        else:
            rospy.logdebug("Stop publishing")
            self.sender().setText("Start Publish CMD")
            self.sender().setDown(0)
            self.publishing = False

    def toggle_mode(self):
        id_ = self.sender().checkedId()
        self.tabs.setCurrentIndex(id_)
        sliders = self.tabs.widget(id_).sliders
        self.ctrl_mode = id_
        for slider in sliders:
            self.axes[slider.axes] = slider.value()*slider.scale
        if id_ == 0:
            rospy.loginfo("ENU position + yaw mode chosen")
            flag = FLAG_ENU_POS_YAW
        elif id_ == 1:
            rospy.loginfo("ENU velocity + yawrate mode chosen")
            flag = FLAG_ENU_VEL_YAWRATE
        elif id_ == 2:
            rospy.loginfo("Roll, pitch, yaw rate + altitude mode chosen")
            flag = FLAG_ROLL_PITCH_YAW
        elif id_ == 3:
            rospy.loginfo("Roll, pitch, yaw rate, w")
            flag = FLAG_ROLL_PITCH_YAW_ANGLE
        #elif id_ == 4:
        #    rospy.loginfo("Roll, pitch, yaw + thrust mode chosen")
        #    flag = FLAG_ROLL_PITCH_YAW_THRUST
        self.axes[4] = flag

    def slider_moving(self, value):
        if self.ctrl_mode == self.sender().mode:
            axes = self.sender().axes
            self.axes[axes] = value*self.sender().scale
        self.sender().text.setText(str(value*self.sender().scale))

    def slider_zero(self):
        self.sender().slider.setSliderPosition(0)
        

if __name__ == '__main__':
    rospy.logwarn('IN MAIN')
    rospy.init_node('wara_landing_gui', anonymous=True, log_level=rospy.DEBUG)

    app = QtWidgets.QApplication(sys.argv)
    w = WARA_Landing_GUI()  # QWidget()
    w.setStyleSheet("""
    .QSlider{
    min-height: 78px;
    max-height: 78px;
    }
    .QSlider::groove:horizontal {
    border: 1px solid #262626;
    height: 15px;
    background: #393939;
    margin: 0 12px;
    }
    .QSlider::handle:horizontal {
    background: #d9ecd0;
    border: 5px solid #77a8a8;
    width: 20px; 
    height: 100px;
    margin: -24px -12px;
    border-radius: 8px;
    }

    QSlider::add-page:qlineargradient {
    background: #c8c3cc;
    border-top-right-radius: 5px;
    border-bottom-right-radius: 5px;
    border-top-left-radius: 0px;
    border-bottom-left-radius: 0px;
    }
    QSlider::sub-page:qlineargradient {
    background: #563f46;
    border-top-right-radius: 0px;
    border-bottom-right-radius: 0px;
    border-top-left-radius: 5px;
    border-bottom-left-radius: 5px;
    }
    QRadioButton{
    min-width: 15em;
    font-weight: 250; 
    font: 18px;
    }
    QLabel{
    font-weight: 250; 
    font: bold 18px;
    }
    QLabel#label{
    font-weight: 250; 
    font: bold 24px;
    }
    QTab{
    font: bold 24px;
    }
    QPushButton{
    font: bold 18px;
    min-width: 8em;
    min-height: 2em;
    font-weight: 800; 
    }
    QPushButton#menu_button{
    min-width: 20em;
    }
    """)
    w.show()

    sys.exit(app.exec_())
