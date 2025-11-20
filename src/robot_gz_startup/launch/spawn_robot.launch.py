# 이 런치 스크립트에서는 Gazebo 프로그램과 함께
# ROS와 Gazebo가 서로 통신할 수 있게 해주는 필수 패키지들을 실행합니다.
# 이 패키지들은 "ros_gz" 패키지에서 이미 제공하고 있습니다:
# - ros_gz_bridge   (ROS와 Gazebo 토픽 간의 변환을 활성화)
# - ros_gz_sim      (Gazebo를 실행하기 위한 인자를 받을 수 있는 런치 스크립트 제공)
# - ros_gz_image    (Gazebo와 ROS 간의 효율적인 이미지 전송을 위한 브리지 유형 제공)
# 추가로 이 스크립트는 몇 가지 인자를 입력받습니다 (CLI 또는 다른 런치 파일/스크립트로부터):
# - robot_name      (Gazebo 시뮬레이션 내에서 로봇이 가질 이름)
# - world_name      (월드 묘사가 담긴 sdf 파일)
# - gz_server_only  (서버만 실행할지, GUI와 서버를 모두 실행할지 결정하는 불리언 값)
# 마지막으로 환경 변수를 추가하거나 로봇의 메시(mesh) 위치를 추가해야 합니다.
# 그래야 Gazebo가 우리 로봇의 메시를 불러올 수 있습니다.
# - GZ_SIM_RESOURCE_PATH    (메시와 같은 추가 파일들을 위한 환경 변수 이름)
# - GZ_SIM_PLUGIN_PATH      (센서 플러그인들이 위치한 환경 변수 이름)
import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

# 다음을 대체하는 치환 유형:
# "from ament_index_python import get_package_share_directory"
from launch_ros.substitutions import FindPackageShare

# - DeclareLaunchArgument
#   -> (CLI나 런치 파일/스크립트에서 전달되는 인자를 정의할 수 있게 함)
# - IncludeLaunchDescription
#   -> (다른 패키지의 런치 파일을 가져와 인자를 전달할 수 있게 함)
# - SetEnvironmentVariable
#   -> (새로운 환경 변수 설정)
# - ExecuteProcess
#   -> 런타임에 명령어를 평가하고 실행
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo , ExecuteProcess , RegisterEventHandler, GroupAction, AppendEnvironmentVariable

# - LaunchConfiguration
#   -> (런치 인자를 저장할 수 있게 함 (인자는 로컬이며 이 파일 범위 내에서만 유효))
# - PythonExpression
#   -> (치환(substitutions)과 이 스크립트의 변수들을 섞어서 사용할 수 있게 함)
# - PathJoinSubstitution
#   -> ("os.path.join"과 같으나 비동기로 수행됨)
# - IfElseSubstitution # humble 버전에는 없음
#   -> (2개의 치환 중 1개를 반환)
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution , TextSubstitution

# - PythonLaunchDescriptionSource
#   -> (ROS에게 포함된 파일이 Python 기반임을 알림 (그 외: XML 또는 YAML))
from launch.launch_description_sources import PythonLaunchDescriptionSource

# - OnProcessExit
#   -> (EventHandler 모듈(RegisterEventHandler 참조)을 실행할 때, 어떤 이벤트에 대해 실행할지 알려줘야 함. OnProcessExit는 그 이벤트 중 하나)
from launch.event_handlers import OnProcessExit

# - IfCondition
#   -> (런치 타임에 확인되는 조건. 결과가 true이면 노드나 액션이 실행됨)
# - UnlessCondition
#   -> (런치 타임에 확인되는 조건. 결과가 false이면 노드나 액션이 실행됨)
from launch.conditions import IfCondition , UnlessCondition

def generate_launch_description():

    # (1) 먼저 가능한 인자들을 정의(DEFINE)합니다 (CLI를 통해 설정 가능).
    # 이것은 특정 인자가 존재하며 나중에 CLI나 다른 런치 파일/스크립트를 통해
    # 이 스크립트로 제공될 수 있음을 ROS2에 알립니다.
    robot_name = LaunchConfiguration('robot_name')
    world_name = LaunchConfiguration('world_name')
    gz_server_only = LaunchConfiguration('gz_server_only')
    robot_namespace = LaunchConfiguration('ns')
    # 이것은 Gazebo에서 시뮬레이션을 실행할 때 중요한 인자입니다.
    # 이 값을 'true'로 설정하면 Gazebo 서버(및 GUI)를 처음 부팅할 때만 
    # 필요한 특정 노드들을 실행하지 않습니다.
    add_robot = LaunchConfiguration('add_robot')
    # 추가 로봇을 스폰(spawn)할 때는 x, y 좌표도 포함해야 합니다.
    robot_x_coord = LaunchConfiguration('robot_x_coord')
    robot_y_coord = LaunchConfiguration('robot_y_coord')
    robot_z_coord = LaunchConfiguration('robot_z_coord')

    # (2) 다음으로 런치 파일의 인자들을 선언(DECLARE)합니다.
    # 이 인자들은 CLI나 다른 런치 파일/스크립트에 의해 전달될 수도 있고 아닐 수도 있습니다.
    # 추가적으로 기본값(default value)을 설정할 수 있습니다. 기본값이 설정되지 않은 경우,
    # 이 인자들 없이 스크립트를 실행하면 에러 메시지가 발생합니다.
    robot_name_arg = DeclareLaunchArgument(
        name='robot_name',
        description='[인자] Gazebo 시뮬레이션 내 로봇의 이름'
    )
    robot_namespace_arg = DeclareLaunchArgument(
        name='ns',
        description='[인자] 동일한 시뮬레이션에서 여러 로봇을 실행할 때 엔티티(시뮬레이션), 노드, 토픽을 분리하기 위해 필요한 네임스페이스'
    )
    world_name_arg = DeclareLaunchArgument(
        name='world_name',
        description='[인자] Gazebo 시뮬레이션 월드 생성을 위해 사용되는 /world 디렉토리에 저장된 sdf 파일의 이름',
        default_value='empty_world'
    )
    gz_server_only_arg = DeclareLaunchArgument(
        name='gz_server_only',
        description='[인자] Gazebo 서버만 실행할지, 아니면 GUI와 서버를 둘 다 실행할지 선택',
        default_value='false'
    )
    add_robot_arg = DeclareLaunchArgument(
        name='add_robot',
        description='[인자] 이미 시뮬레이션이 실행 중일 때 이 인자를 추가하여 시작',
        default_value='false'
    )
    robot_x_coord_arg = DeclareLaunchArgument(
        name='robot_x_coord',
        description='[인자] 로봇을 스폰할 때 (0,0,0)이 아닌 다른 위치에 스폰하고 싶을 경우 이 x 값을 추가',
        default_value='0'
    )
    robot_y_coord_arg = DeclareLaunchArgument(
        name='robot_y_coord',
        description='[인자] 로봇을 스폰할 때 (0,0,0)이 아닌 다른 위치에 스폰하고 싶을 경우 이 y 값을 추가',
        default_value='0'
    )
    robot_z_coord_arg = DeclareLaunchArgument(
        name='robot_z_coord',
        description='[인자] 로봇을 스폰할 때 (0,0,0)이 아닌 다른 위치에 스폰하고 싶을 경우 이 z 값을 추가',
        default_value='0.1'
    )
    # 참고 :
    # "LaunchConfiguration" -> 이것들은 커맨드 라인 인자에 대한 참조(REFERENCES)입니다.
    # "DeclareLaunchArgument" -> 이것들은 CLI나 다른 런치 스크립트/파일로부터 받고자 하는 인자들입니다.

    # 규칙(CONVENTIONS) ============================================================

    # 다음 단락은 명명 규칙에 있어 매우 중요하며, 나중에 새로운 로봇 패키지를 
    # 추가하려 할 때 참고해야 합니다:

    # 1) 로봇 패키지의 이름은:
    # ${ROBOTNAME}_description

    # 2) 로봇의 메인 xacro 파일 이름은:
    # ${ROBOTNAME}_main.xacro

    # 3) robot_state_publisher를 시작하는 로봇의 런치 파일 이름은:
    # ${ROBOTNAME}_description.launch.py

    # 4) 컨트롤러를 시작하는 로봇의 런치 파일 이름은:
    # ${ROBOTNAME}_controllers.launch.py

    # 5) ros_gz_bridge를 위한 로봇 설정 파일 이름은:
    # gz_bridge.yaml

    # 6) 이 패키지 내의 월드 파일은 ".sdf" 파일입니다.

    # 규칙(CONVENTIONS) ============================================================

    # (3) 전달된 파일과 패키지 이름이 파일 시스템에 존재하는지 확인합니다.
    world_pkg_path = get_package_share_directory("robot_gz_startup")

    robot_pkg_path = FindPackageShare(
        # "robot_name"과 확장자의 최종 결과가 문자열이므로,
        # 이를 하나의 단일 문자열로 만들기 위해 작은따옴표 사이에 넣습니다.
        PythonExpression(["'", robot_name, "_description", "'"])
    )

    robot_urdf_path = PathJoinSubstitution([
        robot_pkg_path,
        "urdf",
        PythonExpression([
            "'", robot_name, "_main.xacro", "'"
        ])
    ])
    robot_launch_path = PathJoinSubstitution([
        robot_pkg_path,
        "launch",
        PythonExpression([
            "'", robot_name, "_description.launch.py", "'"
        ])
    ])
    robot_controllers_launch_path = PathJoinSubstitution([
        robot_pkg_path,
        "launch",
        PythonExpression([
            "'", robot_name, "_controllers.launch.py", "'"
        ])
    ])
    robot_gz_bridge_path = PathJoinSubstitution([
        robot_pkg_path,
        "config",
        "gz_bridge.yaml"
    ])
    world_sdf_path = PathJoinSubstitution([
        world_pkg_path,
        "worlds",
        PythonExpression([
            "'", world_name, ".sdf", "'"
        ])
    ])

    # 개선 사항(IMPROVEMENTS) ============================================================

    # 참고: 비동기 조건 검사를 어떻게 수행하는지 정확히 모르겠습니다.
    # "IfCondition" 모듈이나 "IfElseSubstitution" 모듈, 혹은 "opaque" 함수를 사용해야 한다는 것은 알고 있습니다.
    # "PythonExpression" 모듈을 사용할 수도 있지만, 비동기 조건을 생성하는 데 "IfElseSubstitution"을 
    # 사용할 수 있다면 덜 권장되는 방식일 것입니다. 확실치는 않습니다.

    # 개선 사항(IMPROVEMENTS) ============================================================




    # (4) 기존 패키지들을 사용하여 모든 노드와 프로그램을 시작합니다:
    # - ros_gz_sim
    # (이 패키지의 런치 파일을 사용하여 추가 인자와 함께 Gazebo를 더 쉽게 시작하고 로봇을 스폰)
    # - spawn_entity
    # (Gazebo에 로봇을 스폰하기 위해 ros_gz_sim 패키지에서 제공하는 Python 스크립트)
    # - ros_gz_bridge
    # (이 패키지의 런치 파일을 사용하여 ROS 토픽을 Gazebo 토픽으로 변환, 인자 입력 가능)
    # - robot_state_publisher
    # ((보통) 포함된 로봇 패키지 런치 파일에 존재함 (만약 없다면 로봇 패키지 안에 생성해야 함))

    # (4.1) ros_gz_sim
    # 3가지가 필요합니다:
    # 1) 'ros_gz_sim' 런치 파일의 경로 찾기
    # 2) 사용자의 'gz_server_only' 인자 입력을 확인하여 GUI와 서버를 모두 실행할지 서버만 실행할지 결정
    # 2.1) gz_server.launch.py 파일 실행
    # 또는
    # 2.2) gz_sim.launch.py 파일 실행
    # 3) 'add_robot'에 따라 무시할지 결정:
    
    # 1)
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')

    # 2) 및 3) Python 조건문을 사용하여 런치 파일과 인자 결정
    gz_server = PythonExpression([" False if ","'",gz_server_only,"'","=='false' else True "])
    
    gz_launch_file = PythonExpression([
        "'" + os.path.join(ros_gz_sim_pkg_path, 'launch', 'gz_server.launch.py') + "'" ,
        " if ",
        gz_server,
        " else ",
        "'" + os.path.join(ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py') + "'"
    ])

    LogInfo(msg=gz_launch_file)

    world_arg_name = PythonExpression([
        "'world_sdf_file' if ",
        gz_server,
        " else 'gz_args'"
    ])

    world_arg = PythonExpression([
        "'", world_sdf_path, "'",
        " if ",
        gz_server,
        " else " ,
        "'-r -v 4 ' + ",
        "'",world_sdf_path,"'"
    ])

    # 3) 디스크립션(description) 런치
    ros_gz_launch_desc = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(gz_launch_file),
        launch_arguments={
            world_arg_name: world_arg,
            'on_exit_shutdown': 'true'
        }.items(),
        condition=UnlessCondition(add_robot)
    )

    # (4.2) spawn_entity
    # 1가지가 필요합니다:
    # 1) robot_state_publisher 노드에서 보낸 "robot_description" 토픽을 읽고
    # create 명령을 실행하는 노드 시작

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
                '-topic', PythonExpression(["'",robot_namespace,'/robot_description',"'"]),
                '-name', robot_namespace,
                # 어떤 이유에서인지 jetracer와 다른 모델들이 지면(ground_plane) 아래에 스폰되므로 Z 좌표에 1을 더합니다.
                '-z',robot_z_coord,
                # x,y 평면상의 로봇 위치를 위해
                '-x',robot_x_coord,
                '-y',robot_y_coord,
                # 필요에 따라 추가 인자를 나중에 제공할 수 있음
                # ...
        ]
    )

    # (4.3) ros_gz_bridge
    # 참고: "브리지"가 ROS 토픽과 Gazebo 토픽을 서로 통신하게 해주는 방법을 제공하지만,
    # 여전히 수동 설정이 필요합니다. 이는 나중에 필요에 맞춰 생성할 수 있는 
    # YAML 파일을 통해 수행할 수 있습니다.

    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=robot_namespace,
        arguments=[
            # 수동 설정
            # 수동 설정
            # <토픽>@<ROS2_메시지_타입>@<Gazebo_메시지_타입>
            # ROS 메시지 타입 뒤에 오는 것들:
            # -> "@" 는 양방향 브리지입니다.
            # -> "[" 는 Gazebo에서 ROS로의 브리지입니다.
            # -> "]" 는 ROS에서 Gazebo로의 브리지입니다.
            # 더 많은 ROS2 및 Gazebo 토픽은 다음에서 찾을 수 있습니다: https://docs.ros.org/en/jazzy/p/ros_gz_bridge/
            
            
            # 브리지 자체는 나중에 다음 명령어를 사용하여 설정할 수 있습니다:
            # ros2 run ros_gz_bridge parameter_bridge

            # 또는 설정으로 yaml 파일을 사용할 수 있습니다.
            # 이에 대한 자세한 내용은 다음에서 찾을 수 있습니다: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md
            # 또한 다음을 실행할 수 있습니다: ros2 run ros_gz_bridge parameter_bridge --help

            # 우리의 목적에 맞게 브리지를 올바르게 설정하기 위해
            # 다음 링크에서 찾을 수 있는 몇 가지 인자를 전달해야 합니다: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md#example-6-using-ros-namespace-with-the-bridge
            '--ros-args',
            '-p',PythonExpression(["'",'config_file:=',robot_gz_bridge_path,"'"]),
            # '-p','expand_gz_topic_names:=true',
            # 네임스페이스 사용은 서로 다른 로봇에 대해 동일한 토픽(예: /robot_description)을 격리하는 방법입니다.
            # 다른 토픽들이 위치할 토픽을 생성함으로써 가능합니다 (예: /jetank -> /jetank/tf 및 /jetank/robot_description).
            # 문제는 기본적으로 브리지가 Gazebo 토픽에는 ROS 네임스페이스를 적용하지 않아 문제가 발생할 수 있다는 점입니다.
            # 왜냐하면 Gazebo가 브로드캐스트한 /tf는 어떤 로봇을 위한 것인지 알 수 없기 때문입니다.
            # 이것이 우리가 "expand_gz_topic_names" 파라미터를 활성화하는 이유입니다.
        ],
        remappings=[
            ('/tf','tf'),
            ('/tf_static','tf_static'),
        ],
        output='screen'
    )

    # "/camera/image_raw"를 파라미터 브리지에 넣을 수도 있었지만,
    # image_bridge가 이미지 토픽에 대해 더 효율적인 브리지를 제공합니다.
    # 마이그레이션 가이드 참조: https://gazebosim.org/docs/latest/migrating_gazebo_classic_ros2_packages/
    ros_gz_image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        namespace=robot_namespace,
        arguments=[PythonExpression(["'/",robot_namespace,'/camera/image_raw',"'"]), "camera/image_raw"],
        output='screen'
    )

    # (4.4) robot_state_publisher && robot_controllers
    # 2가지가 필요합니다:
    # 1) robot_state_publisher 노드를 시작하는 로봇 패키지의 런치 파일
    # launch 디렉토리 안에 위치하며 이름은 (관례에 따라) "${ROBOTNAME}_description.launch.py"입니다.
    # 2) 컨트롤러 노드를 시작하는 로봇 패키지의 런치 파일
    # launch 디렉토리 안에 위치하며 이름은 (관례에 따라) "${ROBOTNAME}_controllers.launch.py"입니다.
    # 여기서 ${ROBOTNAME}은 로봇의 이름입니다.

    # 1)
    robot_launch_desc = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([robot_launch_path]),
        launch_arguments=[
            ['use_sim_time', 'true'],
            ['ns',robot_namespace]
        ]
    )

    # 2)
    robot_controllers_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([robot_controllers_launch_path]),
        launch_arguments=[
            ['use_sim_time', 'true'],
            ['ns',robot_namespace],
        ]
    )


    # (5) 새로운 환경 변수 설정
    # 이것들을 로드하는 방법과 이유는 여기서 찾을 수 있습니다: https://gazebosim.org/api/sim/8/resources.html
    # 로봇을 추가할 경우 해당 메시(meshes)를 환경 변수에 추가합니다. 월드 메시의 경우도 마찬가지입니다.
    # env_var_resource_robots=AppendEnvironmentVariable(
    #     'GZ_SIM_RESOURCE_PATH',
    #     PathJoinSubstitution([
    #         robot_pkg_path,
    #         'meshes'
    #     ]),
    # )

    # 월드 sdf 파일에서 사용되는 메시가 포함된 메시 디렉토리의 경로를 추가합니다.
    env_var_resource_worlds=SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(
            world_pkg_path,
            'meshes'
        ),
    )

    # 서버만 실행할 때 'gz_server.launch.py' 런치 스크립트가 이 설정을 하지 않기 때문에
    # 이것이 필요할 수 있음을 확인했습니다. 반면 'gz_sim.launch.py' 런치 스크립트는 이 설정을 합니다.
    env_var_plugin=SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib/',
    )
    
    # (6) 모든 것을 좀 더 선형적인 방식으로 시작하고 
    # 노드 시작 순서를 제어하기 위해 몇 가지 이벤트 핸들러를 설정합니다.
    # 로봇이 Gazebo 내부에 스폰된 후에만 로봇 컨트롤러의 
    # 런치 디스크립션을 호출합니다.
    launch_desc_after_entity_is_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[robot_controllers_launch],
        )
    )

    # (7) 마지막으로 런치 디스크립션을 반환합니다.
    return LaunchDescription([
        # 모든 환경 변수들
        env_var_resource_worlds,
        env_var_plugin,
        # env_var_resource_robots,
        # 이 런치 파일의 인자가 제공되지 않았을 경우를 위해 선언된 모든 인자 반환
        robot_name_arg,
        world_name_arg,
        gz_server_only_arg,
        robot_namespace_arg,
        add_robot_arg,
        robot_x_coord_arg,
        robot_y_coord_arg,
        robot_z_coord_arg,
        # 필요한 모든 노드 + 런치 디스크립션들
        robot_launch_desc,
        ros_gz_launch_desc,
        ros_gz_bridge_node,
        ros_gz_image_bridge_node,
        spawn_entity_node,
        # 모든 이벤트들
        launch_desc_after_entity_is_spawn,
        
        
        # 디버그 정보
        LogInfo(msg=gz_launch_file),
        LogInfo(msg=world_arg_name),
        LogInfo(msg=world_arg)
    ])
