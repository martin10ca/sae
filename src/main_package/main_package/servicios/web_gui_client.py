#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int8MultiArray
from service_package.srv import SetSpeed
from flask import Flask, render_template, request, jsonify
import threading
import os


class SpeedWebClient(Node):
    def __init__(self):
        super().__init__("web_gui_node")

        # Declare parameters
        self.declare_parameter(
            "distance_topics",
            rclpy.Parameter.Type.STRING_ARRAY
        )
        self.declare_parameter("tcrt_topic", "tcrt5000_topic")

        # Load parameters
        self.distance_topics = (
            self.get_parameter("distance_topics").get_parameter_value().string_array_value
        )
        self.tcrt_topic = self.get_parameter("tcrt_topic").value

        # State variables
        self.current_speed = "---"
        self.current_status = "---"
        self.tcrt_values = ["-", "-", "-", "-", "-"]
        self.dist = {topic: "---" for topic in self.distance_topics}

        # NEW: acceleration Z
        self.mpu_accel_z = 0.0

        # Service client
        self.cli = self.create_client(SetSpeed, "set_speed")

        # Subscriptions
        self.create_subscription(Float32, "max_speed_pct", self.speed_callback, 10)
        self.create_subscription(String, "status_text", self.status_callback, 10)
        self.create_subscription(Int8MultiArray, self.tcrt_topic, self.tcrt_callback, 10)

        # NEW: MPU6050 accel z topic
        self.create_subscription(
            Float32, "mpu6050_accel_z", self.mpu_accel_callback, 10
        )

        # Distance sensors
        for topic in self.distance_topics:
            self.create_subscription(
                Float32,
                topic,
                lambda msg, t=topic: self.dist_update(t, msg.data),
                10
            )

        self.get_logger().info("Web GUI Client initialized")

    # ------------------ Callbacks ------------------
    def speed_callback(self, msg):
        self.current_speed = msg.data

    def status_callback(self, msg):
        self.current_status = msg.data

    def tcrt_callback(self, msg):
        self.tcrt_values = list(msg.data)

    def dist_update(self, topic, value):
        self.dist[topic] = value

    def mpu_accel_callback(self, msg):
        self.mpu_accel_z = msg.data

    # ------------------ Service ------------------
    def send_speed_request(self, pct):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            return False, "Service not available"

        req = SetSpeed.Request()
        req.pct_speed = float(pct)

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            return False, "Service call failed"

        return future.result().success, future.result().message

    # ------------------ Data to web ------------------
    def get_status(self):
        return {
            "speed": self.current_speed,
            "status": self.current_status,
            "tcrt": self.tcrt_values,
            "dist_sensors": self.dist,
            "mpu_accel_z": self.mpu_accel_z
        }


# ------------------ Flask integration ------------------

def get_template_folder():
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory("main_package")
        return os.path.join(pkg_share, "templates")
    except:
        return os.path.join(os.path.dirname(__file__), "templates")


template_folder = get_template_folder()
app = Flask(__name__, template_folder=template_folder)

ros_node = None


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/set_speed", methods=["POST"])
def set_speed():
    data = request.get_json()
    speed = float(data.get("speed", 0))
    success, msg = ros_node.send_speed_request(speed)
    return jsonify({"success": success, "message": msg})


@app.route("/status")
def status():
    return jsonify(ros_node.get_status())


def spin_ros():
    rclpy.spin(ros_node)


def main():
    global ros_node

    rclpy.init()
    ros_node = SpeedWebClient()

    thread = threading.Thread(target=spin_ros, daemon=True)
    thread.start()

    print("Web server running at http://raspberrypi.local:5000")

    app.run(host="0.0.0.0", port=5000, debug=False)

    ros_node.destroy_node()


if __name__ == "__main__":
    main()
