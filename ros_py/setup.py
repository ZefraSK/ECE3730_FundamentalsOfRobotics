from setuptools import find_packages, setup

package_name = 'ros_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryanle',
    maintainer_email='ryanle@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ros_py.publisher_member_function:main',
            'listener = ros_py.subscriber_member_function:main',
            'server = ros_py.service_member_function:main',
            'client = ros_py.client_member_function:main',
            'fibonacci_action_server = ros_py.fibonacci_action_server:main',
            'fibonacci_action_client = ros_py.fibonacci_action_client:main',
            'minimal_param_node = ros_py.python_parameters_node:main' 
        ],
    },
)
