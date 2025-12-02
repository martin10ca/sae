from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # -----------------------------
        #   TCRT5000 LINE SENSORS
        # -----------------------------
        Node(
            package='main_package',
            executable='tcrt5000_node',
            name='tcrt5000_node',
            parameters=[
                {"pinFarLeft": 31},
                {"pinLeft": 37},
                {"pinCenter": 13},
                {"pinRight": 11},
                {"pinFarRight": 29},
            ]
        ),

        # -----------------------------
        #   MPU6050 IMU
        # -----------------------------
        Node(
            package='main_package',
            executable='mpu6050_node',
            name='mpu6050_node'
        ),

        # -----------------------------
        #   FRONT RIGHT DISTANCE SENSOR (ADC0)
        # -----------------------------
        Node(
            package='main_package',
            executable='distanceSensor_node',
            name='distance_front_right',
            parameters=[{"adc_channel": 0}],
            remappings=[
                ("distanceSensor_topic", "distance/front_right")
            ]
        ),

        # -----------------------------
        #   SPEED SERVICE SERVER
        # -----------------------------
        Node(
            package='main_package',
            executable='server',
            name='speed_server'
        ),

        # -----------------------------
        #   WEB GUI
        # -----------------------------
        Node(
            package='main_package',
            executable='web_gui',
            name='web_gui_node',
            parameters=[
                {"distance_topics": [
                    "distance/front_right",
                ]},
                {"tcrt_topic": "tcrt5000_topic"},
                {"mpu_topic": "mpu6050_distance_z"}
            ]
        ),
    ])
