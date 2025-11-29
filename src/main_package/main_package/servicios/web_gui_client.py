#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from service_package.srv import SetSpeed
from std_msgs.msg import Float32, String, Int8MultiArray
from flask import Flask, render_template, request, jsonify
import threading
import os


class SpeedWebClient(Node):
    def __init__(self):
        super().__init__('web_speed_client')
        
        # Service client
        self.cli = self.create_client(SetSpeed, 'set_speed')

        # State variables
        self.current_speed = 0.0
        self.current_status = "Waiting for connection..."
        self.mpu_z = 0.0
        self.tcrt_values = [0,0,0,0,0]

        # Distance sensors (ADS1115)
        self.dist = {
            "front_right": 0.0,
            "front_left": 0.0,
            "back_right": 0.0,
            "back_left": 0.0
        }

        # Subscriptions

        # max speed status
        self.sub_speed = self.create_subscription(
            Float32, 'max_speed_pct', self.speed_callback, 10)

        self.sub_status = self.create_subscription(
            String, 'status_text', self.status_callback, 10)

        # MPU distance Z
        self.sub_mpu = self.create_subscription(
            Float32, 'mpu6050_distance_z', self.mpu_callback, 10)

        # TCRT5000 (5 sensors)
        self.sub_tcrt = self.create_subscription(
            Int8MultiArray, 'tcrt5000_topic', self.tcrt_callback, 10)

        # ADS1115 (4 distance sensors)
        self.sub_dist_front_r = self.create_subscription(
            Float32, 'distance/front_right',
            lambda msg: self.dist_update("front_right", msg.data), 10)

        self.sub_dist_front_l = self.create_subscription(
            Float32, 'distance/front_left',
            lambda msg: self.dist_update("front_left", msg.data), 10)

        self.sub_dist_back_r = self.create_subscription(
            Float32, 'distance/back_right',
            lambda msg: self.dist_update("back_right", msg.data), 10)

        self.sub_dist_back_l = self.create_subscription(
            Float32, 'distance/back_left',
            lambda msg: self.dist_update("back_left", msg.data), 10)

        self.get_logger().info('Web GUI Client initialized')

    # -----------------------------
    # SENSOR CALLBACKS
    # -----------------------------
    def speed_callback(self, msg):
        self.current_speed = msg.data

    def status_callback(self, msg):
        self.current_status = msg.data

    def mpu_callback(self, msg):
        self.mpu_z = msg.data

    def tcrt_callback(self, msg):
        self.tcrt_values = list(msg.data)

    def dist_update(self, key, value):
        self.dist[key] = value

    # -----------------------------
    # SERVICE CALL
    # -----------------------------
    def send_speed_request(self, pct_speed):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            return False, "Service not available"
        
        request = SetSpeed.Request()
        request.pct_speed = float(pct_speed)
        
        try:
            future = self.cli.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                response = future.result()
                return response.success, response.message
            else:
                return False, "Service call failed"
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return False, str(e)

    # -----------------------------
    # SEND STATUS TO FRONTEND
    # -----------------------------
    def get_status(self):
        return {
            'speed': self.current_speed,
            'status': self.current_status,
            'mpu_z': self.mpu_z,
            'tcrt': self.tcrt_values,
            'dist_sensors': self.dist
        }


# ======================================
# Flask + ROS Integration
# ======================================

def get_template_folder():
    try:
        from ament_index_python.packages import get_package_share_directory
        package_share = get_package_share_directory('main_package')
        template_dir = os.path.join(package_share, 'templates')
        if os.path.exists(template_dir):
            return template_dir
    except:
        pass
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    template_dir = os.path.join(current_dir, 'templates')
    if os.path.exists(template_dir):
        return template_dir
    
    parent_dir = os.path.dirname(current_dir)
    template_dir = os.path.join(parent_dir, 'templates')
    return template_dir


template_folder = get_template_folder()
app = Flask(__name__, template_folder=template_folder)
ros_node = None


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/set_speed', methods=['POST'])
def set_speed():
    try:
        data = request.get_json()
        speed = float(data.get('speed', 0))
        
        if 0 <= speed <= 100:
            success, message = ros_node.send_speed_request(speed)
            return jsonify({
                'success': success,
                'message': message,
                'speed': speed
            })
        else:
            return jsonify({'success': False, 'message': 'Speed must be between 0 and 100'}), 400

    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500


@app.route('/status', methods=['GET'])
def get_status():
    return jsonify(ros_node.get_status())


def spin_ros():
    rclpy.spin(ros_node)


def main():
    global ros_node
    
    rclpy.init()
    ros_node = SpeedWebClient()
    
    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()
    
    print(f"\n📁 Template folder: {template_folder}")
    print(f"   Template exists: {os.path.exists(os.path.join(template_folder, 'index.html'))}")

    print("\n" + "="*50)
    print("🌐 Web GUI Server Started!")
    print("="*50)
    print("\n📱 Access from your browser at:")
    print("   http://raspberrypi.local:5000\n")

    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
