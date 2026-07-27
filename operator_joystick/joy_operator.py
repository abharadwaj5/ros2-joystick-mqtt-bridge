#!/usr/bin/env python3
# Physical-joystick operator device. Reads /joy from the ROS joy node and
# publishes command + heartbeat JSON over MQTT, the same contract the ESP32
# used. The robot-side bridge does not know which operator is talking.

import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import paho.mqtt.client as mqtt

BROKER   = "broker.hivemq.com"
PORT     = 1883
ROBOT_ID = "robotthinkit"

T_CMD   = f"remote_robot/{ROBOT_ID}/cmd"
T_HB    = f"remote_robot/{ROBOT_ID}/esp32_heartbeat"
T_STATE = f"remote_robot/{ROBOT_ID}/state"

AXIS_LINEAR  = 1   # left stick forward/back, forward = +1.0
AXIS_ANGULAR = 0   # left stick left/right,   left    = +1.0
BTN_DEADMAN  = 4   # L1, hold to run

MAX_LIN = 0.5
MAX_ANG = 1.0


class JoyOperator(Node):
    def __init__(self):
        super().__init__("joy_operator")
        self.linear = 0.0
        self.angular = 0.0
        self.deadman = False
        self.seq = 0

        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_timer(0.10, self.publish_cmd)
        self.create_timer(0.25, self.publish_hb)

        self.mqtt = mqtt.Client()
        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_message = self.on_state
        self.mqtt.connect(BROKER, PORT, keepalive=30)
        self.mqtt.loop_start()

        self.get_logger().info("joy operator started")

    def on_joy(self, msg):
        self.linear  = msg.axes[AXIS_LINEAR]  * MAX_LIN
        self.angular = msg.axes[AXIS_ANGULAR] * MAX_ANG
        self.deadman = msg.buttons[BTN_DEADMAN] == 1

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe(T_STATE)
        self.get_logger().info("MQTT connected and subscribed")

    def on_state(self, client, userdata, msg):
        try:
            s = json.loads(msg.payload)
        except json.JSONDecodeError:
            return
        self.get_logger().info(
            f"[state] mode={s.get('mode','?')} reason={s.get('reason','?')} "
            f"cmd_age_ms={s.get('last_cmd_age_ms',-1)}")

    def publish_cmd(self):
        cmd = {
            "linear_x":  round(self.linear, 3),
            "angular_z": round(self.angular, 3),
            "deadman":   self.deadman,
            "seq":       self.seq,
        }
        self.seq += 1
        self.mqtt.publish(T_CMD, json.dumps(cmd))

    def publish_hb(self):
        hb = {"alive": True, "seq": self.seq,
              "uptime_ms": int(self.get_clock().now().nanoseconds / 1e6)}
        self.mqtt.publish(T_HB, json.dumps(hb))


def main():
    rclpy.init()
    node = JoyOperator()
    try:
        rclpy.spin(node)
    finally:
        node.mqtt.loop_stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()