import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    # package_dir = get_package_share_directory('amcl3d')
    package_share_dir = get_package_share_directory('amcl3d')
    # package_root_dir = os.path.abspath(os.path.join(package_share_dir, '..', '..'))
    rviz_config_path = os.path.join(package_share_dir,'amcl3d.rviz')

    # 定义启动参数
    map_name_path = LaunchConfiguration('map_name_path')
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_z = LaunchConfiguration('init_z')
    init_a = LaunchConfiguration('init_a')
    num_particles = LaunchConfiguration('num_particles')
    alpha = LaunchConfiguration('alpha')
    take_off_height = LaunchConfiguration('take_off_height')

    # 定义节点
    amcl3d_node = Node(
        package='amcl3d',
        executable='amcl3d',
        # name='amcl3d_node',
        output='screen',
        # remappings=[
        #     ('laser_sensor', '/pointcloud'),
        #     ('odometry', '/odom'),
        #     ('radiorange_sensor', '/nanotron_node/range')
        # ],
        parameters=[
            {'base_frame_id': 'base_link'},
            {'odom_frame_id': 'odom'},
            {'global_frame_id': 'world'},
            {'map_path': map_name_path},
            {'set_initial_pose': True},
            {'init_x': init_x},
            {'init_y': init_y},
            {'init_z': init_z},
            {'init_a': init_a},
            {'init_x_dev': 0.05},
            {'init_y_dev': 0.05},
            {'init_z_dev': 0.05},
            {'init_a_dev': 0.1},
            {'publish_point_cloud_rate': 10.0},
            {'grid_slice_z': -1.0},
            {'publish_grid_slice_rate': 10.0},
            {'sensor_dev': 0.05},
            {'sensor_range': 0.53},
            {'voxel_size': 0.1},
            {'num_particles': num_particles},
            {'odom_x_mod': 0.1},
            {'odom_y_mod': 0.1},
            {'odom_z_mod': 0.01},
            {'odom_a_mod': 0.3},
            {'resample_interval': 0},
            {'update_rate': 100},
            {'d_th': 0.05},
            {'a_th': 0.03},
            {'take_off_height': take_off_height},
            {'alpha': alpha}
        ]
    )

    # 定义 RVIZ 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # 定义 starter 节点，并延迟启动
    # starter_node = TimerAction(
    #     period=2.0,  # 延迟2秒
    #     actions=[
    #         Node(
    #             package='amcl3d',
    #             executable='starter',
    #             name='starter_node',
    #             output='screen'
    #         )
    #     ]
    # )/home/wan/ROS2_code/mcl/src/amcl3d/data/test_map.bt

    # 创建 LaunchDescription 对象
    ld = LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'map_name_path',
            default_value=os.path.join(package_share_dir, 'data', 'test_map.bt'),
            # default_value=os.path.join('home','wan','ROS2_code', 'src','mcl', 'amcl3d', 'data', 'test_map.bt'),
            description='Path to the map file'
        ),
        DeclareLaunchArgument(
            'init_x',
            default_value='0.0',
            description='Initial x position'
        ),
        DeclareLaunchArgument(
            'init_y',
            default_value='0.0',
            description='Initial y position'
        ),
        DeclareLaunchArgument(
            'init_z',
            default_value='0.0',
            description='Initial z position'
        ),
        DeclareLaunchArgument(
            'init_a',
            default_value='0.0',
            description='Initial orientation'
        ),
        DeclareLaunchArgument(
            'num_particles',
            default_value='600',
            description='Number of particles'
        ),
        DeclareLaunchArgument(
            'alpha',
            default_value='1.0',
            description='Alpha value'
        ),
        DeclareLaunchArgument(
            'take_off_height',
            default_value='0.5',
            description='Take-off height'
        ),
        # 添加节点
        amcl3d_node,
        rviz_node
        # starter_node
        # TimerAction(
        #     period=2.0,  # 延迟2秒
        #     actions=[
        #         Node(
        #             package='amcl3d',
        #             executable='starter',
        #             name='starter_node',
        #             output='screen'
        #         )
        #     ]
        # )
    ])

    return ld