#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from service_package.srv import SetSpeed
from std_msgs.msg import Float32, String
import tkinter as tk
from tkinter import ttk
import threading


class SpeedGUIClient(Node):
    def __init__(self, gui_callback=None):
        super().__init__('gui_speed_client')
        
        # Service client
        self.cli = self.create_client(SetSpeed, 'set_speed')
        
        # Subscribers to monitor status
        self.sub_speed = self.create_subscription(
            Float32, 'max_speed_pct', self.speed_callback, 10)
        self.sub_status = self.create_subscription(
            String, 'status_text', self.status_callback, 10)
        
        self.gui_callback = gui_callback
        self.current_speed = 0.0
        self.current_status = "Waiting for connection..."
        
        # Wait for service
        self.get_logger().info('Waiting for "set_speed" service...')

    def speed_callback(self, msg):
        self.current_speed = msg.data
        if self.gui_callback:
            self.gui_callback('speed', self.current_speed)

    def status_callback(self, msg):
        self.current_status = msg.data
        if self.gui_callback:
            self.gui_callback('status', self.current_status)

    def send_speed_request(self, pct_speed):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available')
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


class SpeedControlGUI:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("SAE Speed Control")
        self.window.geometry("400x300")
        
        # Initialize ROS2
        rclpy.init()
        self.ros_node = SpeedGUIClient(gui_callback=self.update_from_ros)
        
        # Start ROS2 spin in separate thread
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()
        
        self.create_widgets()
        
    def spin_ros(self):
        rclpy.spin(self.ros_node)
    
    def create_widgets(self):
        # Title
        title = tk.Label(self.window, text="Motor Speed Control", 
                        font=("Arial", 16, "bold"))
        title.pack(pady=10)
        
        # Speed input frame
        input_frame = tk.Frame(self.window)
        input_frame.pack(pady=10)
        
        tk.Label(input_frame, text="Speed (%):").pack(side=tk.LEFT, padx=5)
        
        self.speed_entry = tk.Entry(input_frame, width=10)
        self.speed_entry.pack(side=tk.LEFT, padx=5)
        self.speed_entry.insert(0, "0")
        
        # Slider
        self.speed_slider = tk.Scale(
            self.window, from_=0, to=100, orient=tk.HORIZONTAL,
            length=300, label="Speed Percentage",
            command=self.slider_changed)
        self.speed_slider.pack(pady=10)
        
        # Buttons frame
        button_frame = tk.Frame(self.window)
        button_frame.pack(pady=10)
        
        self.send_btn = tk.Button(
            button_frame, text="Set Speed", 
            command=self.send_speed, bg="green", fg="white",
            width=10)
        self.send_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_btn = tk.Button(
            button_frame, text="STOP", 
            command=self.emergency_stop, bg="red", fg="white",
            width=10)
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Status display
        status_frame = tk.LabelFrame(self.window, text="Status", padx=10, pady=10)
        status_frame.pack(pady=10, padx=20, fill=tk.BOTH)
        
        self.status_label = tk.Label(
            status_frame, text="Waiting for connection...", 
            wraplength=350, justify=tk.LEFT)
        self.status_label.pack()
        
        self.current_speed_label = tk.Label(
            status_frame, text="Current Speed: 0.0%", 
            font=("Arial", 10, "bold"))
        self.current_speed_label.pack()
    
    def slider_changed(self, value):
        self.speed_entry.delete(0, tk.END)
        self.speed_entry.insert(0, str(int(float(value))))
    
    def send_speed(self):
        try:
            speed = float(self.speed_entry.get())
            if 0 <= speed <= 100:
                self.send_btn.config(state=tk.DISABLED, text="Sending...")
                self.window.update()
                
                success, message = self.ros_node.send_speed_request(speed)
                
                self.send_btn.config(state=tk.NORMAL, text="Set Speed")
                
                if success:
                    self.status_label.config(
                        text=f"✓ {message}", fg="green")
                    self.speed_slider.set(speed)
                else:
                    self.status_label.config(
                        text=f"✗ Error: {message}", fg="red")
            else:
                self.status_label.config(
                    text="✗ Speed must be between 0 and 100", fg="red")
        except ValueError:
            self.status_label.config(
                text="✗ Please enter a valid number", fg="red")
    
    def emergency_stop(self):
        self.speed_entry.delete(0, tk.END)
        self.speed_entry.insert(0, "0")
        self.speed_slider.set(0)
        self.send_speed()
    
    def update_from_ros(self, topic, value):
        if topic == 'speed':
            self.window.after(0, lambda: self.current_speed_label.config(
                text=f"Current Speed: {value:.1f}%"))
        elif topic == 'status':
            self.window.after(0, lambda: self.status_label.config(
                text=value, fg="blue"))
    
    def run(self):
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.window.mainloop()
    
    def on_closing(self):
        self.ros_node.destroy_node()
        rclpy.shutdown()
        self.window.destroy()


def main():
    gui = SpeedControlGUI()
    gui.run()


if __name__ == '__main__':
    main()