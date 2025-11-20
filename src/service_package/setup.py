from setuptools import find_packages, setup

package_name = 'service_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/service_package']),
        ('share/service_package', ['package.xml']),
        # Copiar el archivo .srv desde su ubicación fuente
        ('share/service_package/srv', ['srv/SetSpeed.srv']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taller',
    maintainer_email='taller@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'server = service_package.server:main',
            'client = service_package.client:main',
        ],
    },
)
