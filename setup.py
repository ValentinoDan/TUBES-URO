from setuptools import setup

package_name = 'line_following'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python'],
    zip_safe=True,
    maintainer='Daniel',
    maintainer_email='danielkusumo550@gmail.com',
    description='ROS2 package for simple line following using OpenCV',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_following_node = line_following.line_following_node.line_following_node:main',
        ],
    },
)
