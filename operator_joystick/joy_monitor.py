#!/usr/bin/env python3
# Joystick monitor for demo videos. Reads /joy and prints one
# readable line describing what the operator is doing. No MQTT, no ROS
# publishing - purely a human-facing display.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

AXIS_LINEAR  = 1
AXIS_ANGULAR = 0
BTN_DEADMAN  = 4
DZ = 0.15   # match the joy node deadzone

class JoyMonitor(Node):
    def __init__(self):
        super().__init__("joy_monitor")
        self.create_subscription(Joy, "/joy", self.on_joy, 10)

    def on_joy(self, msg):
        fb = msg.axes[AXIS_LINEAR]
        lr = msg.axes[AXIS_ANGULAR]
        held = msg.buttons[BTN_DEADMAN] == 1

        parts = []
        if fb > DZ:
            parts.append("FORWARD")
        elif fb < -DZ:
            parts.append("BACKWARD")
        if lr > DZ:
            parts.append("LEFT")
        elif lr < -DZ:
            parts.append("RIGHT")
        motion = " + ".join(parts) if parts else "centered"

        deadman = "L1 HELD" if held else "released"

        print("\033[H\033[J", end="")   # clear screen, cursor to top-left
        print(f"  joystick: {motion}   deadman: {deadman}", flush=True)

def main():
    rclpy.init()
    rclpy.spin(JoyMonitor())
    rclpy.shutdown()

if __name__ == "__main__":
    main()