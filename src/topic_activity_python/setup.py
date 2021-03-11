from setuptools import setup

package_name = 'topic_activity_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='confo',
    maintainer_email='confo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "number_publisher = topic_activity_python.number_publisher:main",
            "number_counter = topic_activity_python.number_counter:main",
            "led_panel = topic_activity_python.led_panel:main",
            "battery = topic_activity_python.battery:main"
        ],
    },
)
