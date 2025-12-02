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
            parameters=[{"adc_channel": 2}],
            remappings=[
                ("distanceSensor_topic", "distance/front_right")
            ]
        ),

        # -----------------------------
        #   MOTOR M1 FRONT (ACTUATOR)
        # -----------------------------
        Node(
            package='main_package',
            executable='m1_actuator',
            name='m1_actuator_front',
            parameters=[
                {"ain1": 17},
                {"ain2": 27},
                {"pwma": 22},
                {"pwm_frequency": 500},
                {"cmd_topic": "/m1/front/cmd"}
            ]
        ),

        # -----------------------------
        #   MOTOR M1 FRONT (CONTROLLER)
        # -----------------------------
        Node(
            package='main_package',
            executable='m1_controller',
            name='m1_controller_front',
            parameters=[
                {"cmd_topic": "/m1/front/cmd"}
            ]
        ),

        # =========================================================
        #   SOLENOIDS (M2 FRONT LEFT)
        # =========================================================

        # M2 Controller — listens to chosen distance sensor
        Node(
            package='main_package',
            executable='m2_controller',
            name='m2_front_left_ctrl',
            parameters=[
                {"distance_topic": "distance/front_right"},
                {"cmd_topic": "/m2/front_left/cmd"}
            ]
        ),

        # M2 Actuator — listens to its own cmd topic
        Node(
            package='main_package',
            executable='m2_actuator',
            name='m2_front_left_actuator',
            parameters=[
                {"solenoid_pin": 22},
                {"cmd_topic": "/m2/front_left/cmd"}
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
                # LISTA DE TOPICOS DE DISTANCIA A ESCUCHAR
                {"distance_topics": [
                    "distance/front_right"
                    # Agregar más aquí en el futuro
                ]},

                # TOPICO TCRT
                {"tcrt_topic": "tcrt5000_topic"},

                # TOPICO CORRECTO DEL MPU6050
                {"mpu_topic": "mpu6050_distance_z"}
            ]
        ),
    ])
