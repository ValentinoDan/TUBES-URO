import launch # Modul utama untuk mendefinisikan dan menjalankan launch script
from launch import LaunchDescription # digunakan untuk mengemas semua komponen launch (node, proses)
from launch.actions import DeclareLaunchArgument, ExecuteProcess # DeclareLaunchArgument digunakan untuk mendeklarasikan argumen launch yang dapat disesuaikan pengguna saat menjalankan launch file
# ExecuteProcess: digunakan untuk menjalankan proses eksternal, seperti Gazebo
from launch_ros.actions import Node # Node: digunakan untuk mendefinisikan dan menjalankan node di ROS 2.

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_description', default_value='robot.urdf', description='Path to URDF file'),
        # Mendeklarasikan argumen robot_description dengan nilai default 'robot.urdf'

        # Node untuk mempublikasikan robot state
        Node(
            package='robot_state_publisher', # Node berasal dari package robot_state_publisher
            executable='robot_state_publisher', # Menjalankan executable robot_state_publisher yang bertugas menjalankan file URDF
            name='robot_state_publisher', 
            parameters=[{'robot_description': open('urdf/robot.urdf', 'r').read()}], # Menggunakan parameter robot_description, diisi dengan isi file 'urdf/robot.urdf'
            output='screen' # akan ditampilkan di terminal
        ),
        
        # Node untuk mengontrol posisi joint robot, jika diperlukan
        Node(
            package='joint_state_publisher_gui', # Node ini berasal dari package joint_state_publisher_gui
            executable='joint_state_publisher_gui', # Node ini menyediakan antarmuka GUI untuk mengontrol posisi joint robot secara manual
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Proses untuk menjalankan Gazebo dengan world file
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 'worlds/line_following_world.world'],
            output='screen'
        ),

         # Node untuk line following robot
        Node(
            package='line_following',  
            executable='line_following_node',  
            name='line_following_node',
            output='screen',
            parameters=[{'robot_description': open('urdf/robot.urdf', 'r').read()}],  
            remappings=[('/camera/image_raw', '/camera/image_raw')]  
        )
    ])
