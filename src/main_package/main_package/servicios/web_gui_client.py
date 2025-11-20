#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from service_package.srv import SetSpeed
from std_msgs.msg import Float32, String
from flask import Flask, render_template, request, jsonify
import threading
import os


class SpeedWebClient(Node):
    def __init__(self):
        super().__init__('web_speed_client')
        
        # Service client
        self.cli = self.create_client(SetSpeed, 'set_speed')
        
        # Subscribers to monitor status
        self.sub_speed = self.create_subscription(
            Float32, 'max_speed_pct', self.speed_callback, 10)
        self.sub_status = self.create_subscription(
            String, 'status_text', self.status_callback, 10)
        # TODO Suscribe to mpu acceleration in Z
        self.current_speed = 0.0
        self.current_status = "Waiting for connection..."
        
        self.get_logger().info('Web GUI Client initialized')

    def speed_callback(self, msg):
        self.current_speed = msg.data

    def status_callback(self, msg):
        self.current_status = msg.data

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

    def get_status(self):
        return {
            'speed': self.current_speed,
            'status': self.current_status
        }


# Get the path to the templates directory
def get_template_folder():
    # Get the share directory path
    try:
        from ament_index_python.packages import get_package_share_directory
        package_share = get_package_share_directory('main_package')
        template_dir = os.path.join(package_share, 'templates')
        if os.path.exists(template_dir):
            return template_dir
    except:
        pass
    
    # Fallback: use the directory relative to this file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    template_dir = os.path.join(current_dir, 'templates')
    if os.path.exists(template_dir):
        return template_dir
    
    # Last resort: check parent directory
    parent_dir = os.path.dirname(current_dir)
    template_dir = os.path.join(parent_dir, 'templates')
    return template_dir


# Flask app with template folder
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
            return jsonify({
                'success': False,
                'message': 'Speed must be between 0 and 100'
            }), 400
    except Exception as e:
        return jsonify({
            'success': False,
            'message': str(e)
        }), 500


@app.route('/status', methods=['GET'])
def get_status():
    status = ros_node.get_status()
    return jsonify(status)


def spin_ros():
    rclpy.spin(ros_node)


def main():
    global ros_node
    
    # Initialize ROS2
    rclpy.init()
    ros_node = SpeedWebClient()
    
    # Start ROS2 in separate thread
    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()
    
    # Print template folder location for debugging
    print(f"\n📁 Template folder: {template_folder}")
    print(f"   Template exists: {os.path.exists(os.path.join(template_folder, 'index.html'))}")
    
    # Start Flask server
    print("\n" + "="*50)
    print("🌐 Web GUI Server Started!")
    print("="*50)
    print("\n📱 Access from your browser at:")
    print("   http://raspberrypi.local:5000")
    print("   or")
    print("   http://192.168.216.14:5000")
    print("\n Press Ctrl+C to stop\n")
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()