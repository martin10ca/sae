from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        #  TCRT5000 Sensor Node
        Node(
            package='main_package',
            executable='tcrt5000_node',
            name='tcrt5000_node',
            parameters=[
                {"pinFarLeft": 11},
                {"pinLeft": 13},
                {"pinCenter": 29},
                {"pinRight": 31},
                {"pinFarRight": 37},
            ]
        ),

        #  MPU6050 Sensor Node
        Node(
            package='main_package',
            executable='mpu6050_node',
            name='mpu6050_node'
        ),

        #  ADS1115 Distance Sensor Node
        Node(
            package='main_package',
            executable='distanceSensor_node',
            name='distance_front',
            parameters=[{"adc_channel": 0}],
            remappings=[("distanceSensor_topic", "distance/front_right")]
        ),

        """
        Node(
            package='main_package',
            executable='distanceSensor_node',
            name='distance_left',
            parameters=[{"adc_channel": 1}],
            remappings=[("distanceSensor_topic", "distance/front_left")]
        ),

        Node(
            package='main_package',
            executable='distanceSensor_node',
            name='distance_right',
            parameters=[{"adc_channel": 2}],
            remappings=[("distanceSensor_topic", "distance/back_right")]
        ),
        Node(
            package='main_package',
            executable='distanceSensor_node',
            name='distance_right',
            parameters=[{"adc_channel": 3}],
            remappings=[("distanceSensor_topic", "distance/back_left")]
        ),
        """,

        #  Motor Actuator (FRONT)
        Node(
            package='main_package',
            executable='m1_actuator',
            name='m1_actuator_front'
        ),
        """
        Node(
            package='main_package',
            executable='m1_actuator',
            name='m1_actuator_back'
        ),""",

        #  Controller Node (FRONT)
        Node(
            package='main_package',
            executable='m1_controller',
            name='m1_controller_front'
        ),
        
        """
        Node(
            package='main_package',
            executable='m1_controller',
            name='m1_controller_back'
        ),
        """,

        #  Speed Service Server
        Node(
            package='main_package',
            executable='server',
            name='speed_server'
        ),

        #  Web GUI (Flask + ROS2 bridge)
        Node(
            package='main_package',
            executable='web_gui',
            name='web_gui_node'
        ),
    ])
