#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
The flight control flags.

Horizontal
 0x00 Command roll and pitch angle (Ground/Body, 0.611 rad (35 degree))
 0x40 Command horizontal velocities (Ground/Body, 30 m/s)
 0x80 Command position offsets (Ground/Body, N/A)

Vertical
 0x00 Command the vertical speed (Ground, -5 to 5 m/s)
 0x10 Command altitude (Ground, 0 to 120 m)
 0x20 Command thrust (Body, 0% to 100%)

Yaw
 0x00 Command yaw angle (Ground, -pi to pi)
 0x08 Command yaw rate (Ground, 5⁄6pi rad/s)

Coordinate
 0x00 Horizontal command is ground_ENU frame
 0x02 Horizontal command is body_FLU frame

Active Break
 0x00 No active break
 0x01 Actively break to hold position after stop sending setpoint

Linnea Persson
laperss@kth.se
"""

HORIZONTAL_ANGLE = 0x00
HORIZONTAL_VELOCITY = 0x40
HORIZONTAL_POSITION = 0x80

VERTICAL_VELOCITY = 0x00
VERTICAL_POSITION = 0x10
VERTICAL_THRUST = 0x20

YAW_ANGLE = 0x00
YAW_RATE = 0x08

HORIZONTAL_GROUND = 0x00
HORIZONTAL_BODY = 0x02

STABLE_DISABLE = 0x00
STABLE_ENABLE = 0x01

# Flight Control: PxPyPzYawCallback
FLAG_ENU_POS_YAW = (VERTICAL_POSITION |
                    HORIZONTAL_POSITION |
                    YAW_ANGLE |
                    HORIZONTAL_GROUND |
                    STABLE_ENABLE)

# Flight Control: VxVyVzYawrateCallback
FLAG_ENU_VEL_YAWRATE = (VERTICAL_VELOCITY |
                        HORIZONTAL_VELOCITY |
                        YAW_RATE |
                        HORIZONTAL_GROUND |
                        STABLE_ENABLE)

# Flight Control: RollPitchPzYawrateCallback
FLAG_ROLL_PITCH_YAW = (VERTICAL_POSITION |
                       HORIZONTAL_ANGLE |
                       YAW_RATE |
                       HORIZONTAL_BODY |
                       STABLE_DISABLE)

FLAG_ROLL_PITCH_YAW_ANGLE = (VERTICAL_VELOCITY |
                             HORIZONTAL_ANGLE |
                             YAW_ANGLE |
                             HORIZONTAL_BODY |
                             STABLE_DISABLE)

FLAG_ROLL_PITCH_YAW_RATE = (VERTICAL_VELOCITY |
                             HORIZONTAL_ANGLE |
                             YAW_RATE |
                             HORIZONTAL_BODY |
                             STABLE_DISABLE)

FLAG_ROLL_PITCH_YAW_THRUST = (VERTICAL_THRUST |
                              HORIZONTAL_ANGLE |
                              YAW_ANGLE |
                              HORIZONTAL_BODY |
                              STABLE_DISABLE)

ALT_MIN = 0
ALT_MAX = 20
ALT_VEL_LIM = 5               # m/s (should this be 4?)
ROLL_ANGLE_LIM = 0.611        # rad
PITCH_ANGLE_LIM = 0.611       # rad
YAW_ANGLE_LIM = 3.14159264    # rad
YAW_RATE_LIM = 5/6*3.14159264 # rad/s
HOR_VEL_LIM = 30              # m/s
