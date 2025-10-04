from setuptools import setup, find_packages

package_name = 'cmd_safety'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['cmd_safety', 'cmd_safety.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/safety_pipeline.launch.py']),
        ('share/' + package_name + '/config', ['config/safety.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Safety node for cmd_vel gating with estop and lidar checks (ROS 2 Jazzy).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_node = cmd_safety.safety_node:main',
            'laser_sim  = cmd_safety.laser_sim:main',   # util de prueba opcional
        ],
    },
    python_requires='>=3.10',
)
