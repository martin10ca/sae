from setuptools import setup, find_packages
import glob
import os
package_name = 'main_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'templates'), 
            glob.glob('main_package/templates/*.html')),
        (os.path.join('share', package_name, 'launch'),
            glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taller',
    maintainer_email='taller@example.com',
    description='Paquete principal con sensores, actuadores, controladores y servicios del proyecto SAE',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Controladores
            'm1_controller = main_package.controladores.M1Controller:main',

            # Actuadores
            'm1_actuator = main_package.actuadores.M1Actuator:main',

            # Sensores
            'mpu6050_node = main_package.sensores.mpu6050_node:main',
            'tcrt5000_node = main_package.sensores.tcrt5000_node:main',
            'distanceSensor_node = main_package.sensores.distanceSensor_node:main',

            # Servicios
            'server = main_package.servicios.server:main',
            'client = main_package.servicios.client:main',
            'gui_client = main_package.servicios.gui_client:main',
            'web_gui = main_package.servicios.web_gui_client:main',

        ],
    },
)
