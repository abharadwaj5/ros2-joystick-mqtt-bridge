![TeleBridge](docs/banner.png)

**Fail-safe remote teleoperation for ROS 2 robots over MQTT.**

TeleBridge lets you drive a ROS 2 robot from a joystick that is not on the
robot's network, safely. A bridge node receives joystick commands as JSON over
MQTT, validates them, enforces the robot's own speed limits, and stops the robot
the instant the link becomes unhealthy. Two operator devices are included, a
physical game controller and a simulated ESP32, and the same bridge drives both
without changing a line.

![demo](docs/demo.gif)

    Operator (pick one)              MQTT broker            ROS 2 bridge (C++)
    physical joystick, or   -- cmd JSON     -->             validate, clamp
    Wokwi ESP32             -- heartbeat    -->             5 fallback checks
    state shown to operator <-- state JSON  <--             Twist on /cmd_vel @ 20 Hz

## Background: ROS 2, MQTT, and why a bridge

**ROS 2** is the standard framework for building robot software. Sensors,
controllers and actuators are separate programs (nodes) that exchange messages
over named topics. A mobile robot listens for velocity commands on a topic
called /cmd_vel, so anything that publishes to /cmd_vel can drive it. ROS 2
handles discovery and transport automatically through a system called DDS, which
is excellent on a single robot or a local network.

**MQTT** is a lightweight publish/subscribe messaging protocol built for the
internet and for constrained devices. Publishers send messages to topics on a
central broker, and subscribers receive them, without the two ever knowing about
each other directly. It is the common language of IoT sensors and
microcontrollers because it is small, tolerant of unreliable links, and works
across networks.

**Why a bridge between them?** DDS, which ROS 2 relies on, does not travel across
the internet. It uses network discovery that home routers, mobile networks, VPNs
and cloud NAT routinely block, so two ROS 2 machines on different networks
usually cannot see each other at all. MQTT was designed for exactly that
situation. A bridge that speaks MQTT on one side and ROS 2 on the other lets an
operator anywhere on the internet drive a robot, using the same protocol that
sensors and embedded devices already use. It joins the world of small networked
devices to the world of ROS robots.

## Why both a simulator and a real joystick?

The two operators show the same idea from two angles.

The **simulated ESP32** (running in Wokwi, needing no hardware) represents the
embedded case: a small, low-power microcontroller reading sensors, exactly the
kind of device MQTT was built for. Anyone can open it in a browser and run it,
which makes the project reproducible without buying anything.

The **physical joystick** represents the desktop case: a full operator station
with a real game controller. It also proves the point that matters most, that the
robot side does not care what the operator is. The same bridge, with no changes,
was driven first by the simulated ESP32 and then by a real DualSense. The
operator device is interchangeable because both sides agree only on a message
contract, not on any particular hardware.

That interchangeability is the whole argument for a bridge: build the safety and
control logic once, on the robot side, and let any operator device that can
produce the JSON drive the robot.

## Why state feedback matters

In remote operation the operator often cannot see the robot directly. That
changes what "safe" means. If the robot simply stops, the operator has no way of
knowing why, whether their own connection dropped, whether they released the
control, or whether the robot itself failed.

TeleBridge sends a state message back to the operator continuously, reporting
whether it is in remote control or fallback, and naming the reason. When the
robot stops, the operator sees precisely why: link lost, command stale, heartbeat
missing, or deadman released. This closes the loop. Remote operation without
feedback is driving blind; the return channel is what makes it operation rather
than guesswork.

## Safety notice

This is a learning prototype, not safety-rated software. It has no
authentication, no encryption, and no independent emergency stop, and has only
been tested against simulators. Do not use it to control physical machinery
capable of causing injury or damage. If you adapt it for real hardware, the
deadman must be backed by an independent hardware e-stop that does not depend on
this software, the network, or the broker.

## Project layout

The project is split into three parts that only ever meet through the MQTT
message contract, so each can be understood, tested, or replaced on its own.

    operator_joystick/
      joy_operator.py     Reads the joystick and publishes command and heartbeat
                          JSON over MQTT. The desktop operator.
      joy_monitor.py      Optional display-only node that prints the joystick
                          state for demos. Publishes nothing.

    operator_esp32/
      sketch.ino          The ESP32 firmware. Reads two potentiometers and a
                          button, publishes the same JSON as the joystick
                          operator, and lights an LED on remote-control
                          confirmation.
      diagram.json        The Wokwi wiring.
      libraries.txt       The required Arduino libraries.

    ros2_ws/src/remote_robot_bridge/
      src/mqtt_bridge_node.cpp        The ROS 2 bridge. Receives MQTT, applies
                                      the safety checks, publishes /cmd_vel.
      include/.../fallback_logic.hpp  The pure safety-decision function, with no
                                      ROS or MQTT, so it can be unit-tested.
      test/test_fallback_logic.cpp    The unit tests for that function.

# Getting started

## 1. Build the bridge

The bridge is a standard ROS 2 C++ package, built with colcon.

Install its two dependencies, the MQTT client library and the JSON parser:

    sudo apt install -y libmosquitto-dev nlohmann-json3-dev

Build the workspace. colcon compiles the package and places the runnable node
where ROS can find it:

    cd ros2_ws
    colcon build

Make this terminal aware of the freshly built node, then run it:

    source install/setup.bash
    ros2 run remote_robot_bridge mqtt_bridge

On startup the node connects to the broker and prints "MQTT connected and
subscribed", followed by a once-per-second status line reporting whether it is
driving the robot or holding it stopped, and why.

## 2. Choose an operator

An operator is any device that publishes the command and heartbeat JSON. Two are
included. Run one of them alongside the bridge.

### Operator A: physical joystick

This is the desktop operator: a game controller drives the robot through the ROS
2 joy driver. It was tested with a Sony DualSense over USB, but any controller the
joy driver recognises will work.

Install the joy driver and the Python MQTT library:

    sudo apt install -y ros-jazzy-joy python3-paho-mqtt

Find your controller's device id (the joy driver lists what it can see):

    ros2 run joy joy_enumerate_devices

Start the joy driver, which publishes the controller's state on the /joy topic.
The deadzone stops small thumb-drift on the stick from creeping the robot:

    ros2 run joy joy_node --ros-args -p device_id:=0 -p deadzone:=0.15

Run the operator, which turns /joy into the MQTT command and heartbeat JSON:

    python3 operator_joystick/joy_operator.py

The left stick drives, forward and back on one axis and turning on the other, and
L1 is the deadman, which must be held for the robot to move. Axis and button
numbers vary between controllers; they are constants at the top of
joy_operator.py, and you can check yours with `ros2 topic echo /joy --once`.

### Operator B: simulated ESP32 (no hardware)

![wokwi circuit](docs/wokwi.png)

This operator runs entirely in the browser, no hardware required. It shows the
embedded case: a microcontroller reading sensors and publishing the same MQTT
command and heartbeat JSON as the joystick operator.

1. Open https://wokwi.com and create a new project using the ESP32 (Arduino)
   template.
2. Replace the three project files with the ones in operator_esp32/:
   - sketch.ino    the firmware (reads inputs, publishes MQTT, drives the LED)
   - diagram.json  the wiring (two potentiometers, a pushbutton, an LED)
   - libraries.txt the required Arduino libraries
3. Press the play button.

The two potentiometers are the joystick axes, the red pushbutton is the deadman
(hold to run), and the green LED lights only while the robot confirms remote
control. Open the serial monitor to watch the commands being sent and the robot
state coming back.

## 3. Drive a robot

The bridge publishes standard velocity commands, so anything that listens on
/cmd_vel can be driven with no change to the bridge. The simplest target is
turtlesim, a lightweight 2D robot simulator included with ROS.

    sudo apt install -y ros-jazzy-turtlesim
    ros2 run turtlesim turtlesim_node

Start the bridge with a remap so its /cmd_vel output is delivered to the turtle.
This is the only thing that changes between one robot and another; the bridge
itself is untouched:

    ros2 run remote_robot_bridge mqtt_bridge --ros-args -r /cmd_vel:=/turtle1/cmd_vel

Hold the deadman and drive. Release it mid-motion and the robot stops within one
control tick; kill the operator entirely and it stops within one second. The same
approach drives a Gazebo rover or a real diff-drive base by remapping to that
robot's velocity topic.

## 4. Configuration

The operator and the bridge must agree on two things: which broker they meet on,
and which robot they are talking about. These are set in the source of each side.

    Broker      Where operator and bridge connect. Set by the BROKER constant in
                the operator and the mosquitto_connect call in the bridge.
                Default: broker.hivemq.com, port 1883.

    Robot ID    The name used in the topic strings, so several robots can share a
                broker without interfering. Set in the topic strings on both
                sides. Default: robotthinkit.

    Limits      The maximum linear and angular speed. Enforced on the robot side,
                so the operator can never exceed them. See "Adjusting speed".

    Timeout     How long a command or heartbeat may be missing before the robot
                stops. Set in the bridge. Default: 1000 ms, against commands at
                10 Hz and heartbeats at 4 Hz, which leaves a wide margin for lost
                messages.

A public broker is used because browser-based Wokwi cannot reach a broker on the
local machine. Note that a public broker is open: anyone could publish to your
topics, so choose a unique robot ID and treat this as a lab setup rather than a
deployment.

### Adjusting speed

The robot enforces its own limits, so speed is capped in two places and the lower
value wins. To drive faster, raise both.

In operator_joystick/joy_operator.py:

    MAX_LIN = 0.5    # increase for faster forward and back
    MAX_ANG = 1.0    # increase for faster turning

And the clamp in ros2_ws/src/remote_robot_bridge/src/mqtt_bridge_node.cpp:

    if (lin >  0.5) lin =  0.5;   // raise these four limits to match
    if (lin < -0.5) lin = -0.5;
    if (ang >  1.0) ang =  1.0;
    if (ang < -1.0) ang = -1.0;

Keeping the clamp on the robot side is deliberate: the robot never trusts the
operator to send sane values, so the final limit lives with the robot.

## The safety design

The bridge publishes /cmd_vel continuously at 20 Hz. Publishing on a timer,
rather than only when a command arrives, is what makes a stop an actively
maintained command instead of just the absence of one. Every tick re-evaluates
these conditions, and the first failure names the reason reported back to the
operator:

- the MQTT connection is down
- no command has been received since startup
- the last command is older than one second
- the last heartbeat is older than one second
- the last command was invalid or missing fields
- the deadman is not held

Only if all pass are the clamped joystick values forwarded; otherwise the robot
receives explicit zeros. The fallback state is also the initial state: at boot
the robot has no evidence anyone is in control, so it is stopped.

## Tests

The safety logic is separated from ROS, MQTT and the clock into a single pure
function (fallback_logic.hpp), which makes it directly testable. The unit tests
in test/test_fallback_logic.cpp exercise the whole decision table: a healthy
case that is allowed to move, each fallback condition in isolation, the ordering
rule that the first failing check names the reason, and the clamping of
excessive values.

This matters because the logic can be verified without a robot, a broker, or
any waiting. A situation that is awkward to produce for real, such as a command
that is several seconds old, is simply a number handed to the function, so every
branch is checked in milliseconds. The tests also guard against regressions:
they were written after the command and state paths were unified, and would have
caught the earlier bug where the two paths reported the same condition with
different wording.

Run them with:

    cd ros2_ws
    colcon test --packages-select remote_robot_bridge
    colcon test-result --verbose

## Where this could go

This is a foundation, not a finished product. Because the operator and robot are
decoupled by a message contract, the same bridge could sit under much larger
systems. The pattern, a networked operator, a transport that crosses networks,
and a robot-side safety layer that fails to a stop, is the basis of any serious
remote or supervised operation. What gets built on top of it is left open on
purpose; the point of this repository is the safe, reusable core.

## Learn more

- [ROS 2](https://docs.ros.org)
- [MQTT protocol](https://mqtt.org)
- [Eclipse Mosquitto (MQTT broker)](https://mosquitto.org)
- [HiveMQ public broker](https://www.hivemq.com/mqtt/public-mqtt-broker/)
- [Wokwi ESP32 simulator](https://wokwi.com)
- [ROS 2 joy package](https://index.ros.org/p/joy/)

## License

MIT. See LICENSE.