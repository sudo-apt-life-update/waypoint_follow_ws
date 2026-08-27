"""Bring up the open-loop path-following stack.

    ros2 launch g1_walk walk.launch.py path:=square_2m.yaml

Nothing here moves the robot on its own. The launch brings up the bridge with
its gate closed and the follower idle; walking takes two more explicit acts:

    ros2 service call /g1_loco_bridge/enable std_srvs/srv/Trigger
    ros2 service call /path_follower/start  std_srvs/srv/Trigger

Two helper processes must already be running, because neither can live inside a
ROS node -- unitree_sdk2 and ROS 2 each link their own CycloneDDS and corrupt
the heap in one address space:

  1. Commands (required):
         ros2 run g1_loco_server g1_loco_server --iface=eno1

  2. State, for RViz and logging (optional, state:=true):
         ~/workspaces/unitree/unitree_sdk2/build/bin/g1_state_server eno1

Arguments worth knowing:
    path            file in config/paths, or an absolute path
    linear_scale    open-loop calibration; see config/paths/straight_3m.yaml
    angular_scale   the same, for turns
    state           also bridge joints + IMU (needs g1_state_server)
    rviz            RViz with the model and the planned path (implies state)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_path_file(context):
    """Let `path:=square_2m.yaml` mean the packaged file of that name.

    Resolved here rather than in the node so a typo fails at launch, naming the
    directory that was searched, instead of inside a node that then exits.
    """
    requested = LaunchConfiguration("path").perform(context)
    share = get_package_share_directory("g1_walk")

    if os.path.isabs(requested):
        candidate = requested
    else:
        candidate = os.path.join(share, "config", "paths", requested)

    if not os.path.isfile(candidate):
        available = sorted(
            name for name in os.listdir(os.path.join(share, "config", "paths"))
            if name.endswith(".yaml")
        )
        raise RuntimeError(
            f"path file not found: {candidate}\n"
            f"  available in the package: {', '.join(available)}"
        )

    return [_follower_node(candidate)]


def _f(name):
    """A launch argument as a float parameter.

    Launch arguments are strings, and rclpy infers a type when it converts
    them: `ramp:=0` becomes INTEGER and is then refused by a parameter
    declared DOUBLE, while `ramp:=0.0` works. Forcing the type here means both
    spellings behave, which is the difference between a usable flag and a
    launch that dies on a plausible-looking value.
    """
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _b(name):
    """A launch argument as a bool parameter, for the same reason."""
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _follower_node(path_file):
    return Node(
        package="g1_walk",
        executable="path_follower",
        name="path_follower",
        output="screen",
        parameters=[{
            "path_file": path_file,
            "rate": _f("rate"),
            "linear_scale": _f("linear_scale"),
            "angular_scale": _f("angular_scale"),
            "yaw_trim": _f("yaw_trim"),
            "ramp_down_override": _f("ramp_down"),
            "ramp_override": _f("ramp"),
            "distance_offset": _f("distance_offset"),
            "autostart": _b("autostart"),
            "feedback": _b("feedback"),
            "use_sim_time": False,
        }],
    )


def generate_launch_description():
    walk_share = get_package_share_directory("g1_walk")
    description_share = get_package_share_directory("g1_description")
    urdf_path = os.path.join(description_share, "urdf", "g1_29dof.urdf")

    with open(urdf_path, "r") as handle:
        robot_description = handle.read()

    # rviz:=true implies state:=true -- an RViz window showing a robot frozen
    # in its zero pose is a misleading way to watch a walk.
    state_enabled = PythonExpression([
        "'", LaunchConfiguration("state"), "' == 'true' or '",
        LaunchConfiguration("rviz"), "' == 'true'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "path", default_value="nudge.yaml",
            description="path file in config/paths, or an absolute path. "
                        "Defaults to the shortest one on purpose."),
        DeclareLaunchArgument(
            "rate", default_value="20.0",
            description="/cmd_vel publish rate [Hz]"),
        DeclareLaunchArgument(
            "linear_scale", default_value="1.0",
            description="multiplies commanded time for straights; "
                        "measure it with config/paths/straight_3m.yaml"),
        DeclareLaunchArgument(
            "angular_scale", default_value="1.0",
            description="multiplies commanded time for turns"),
        DeclareLaunchArgument(
            # The one remaining difference between `loco_cli move`, which
            # tracks distance to within 5%, and a path run, which loses half of
            # it: the CLI sends 0.15 s and the bridge 0.5 s. Exposed so the two
            # can be compared directly instead of argued about.
            "command_duration", default_value="0.5",
            description="how long the robot honours each velocity command [s]"),
        DeclareLaunchArgument(
            "ramp", default_value="-1.0",
            description="override every segment's ramp-in [s]; -1 keeps the "
                        "path file's value, 0 means a step command. On this "
                        "robot a ramp below ~0.4 m/s covers no ground, so "
                        "ramp:=0 ramp_down:=0 may track better."),
        DeclareLaunchArgument(
            "ramp_down", default_value="-1.0",
            description="override every segment's ramp-out [s]; -1 keeps the "
                        "path file's value, 0 means a step stop."),
        DeclareLaunchArgument(
            "distance_offset", default_value="0.0",
            description="metres added to each translation segment to cancel "
                        "the fixed distance the robot settles backwards when "
                        "a segment ends. Measured ~0.35 m on this robot."),
        DeclareLaunchArgument(
            "yaw_trim", default_value="0.0",
            description="rad/s added on straight segments to cancel a "
                        "consistent veer; positive steers left"),
        DeclareLaunchArgument(
            "autostart", default_value="false",
            description="start walking without the ~/start service. "
                        "Leave this false unless you know why you want it."),
        DeclareLaunchArgument(
            "server_address", default_value="127.0.0.1",
            description="host running g1_loco_server"),
        DeclareLaunchArgument(
            "state_address", default_value="127.0.0.1",
            description="host running g1_state_server"),
        DeclareLaunchArgument(
            "feedback", default_value="false",
            description="close the loop: walk each segment until /odom says "
                        "it has arrived, holding heading. Needs odom:=true. "
                        "With this on, leave linear_scale and "
                        "distance_offset at their defaults."),
        DeclareLaunchArgument(
            "odom", default_value="true",
            description="publish the robot's state estimate as /odom + TF. "
                        "Needed for closed-loop path following."),
        DeclareLaunchArgument(
            "state", default_value="false",
            description="also bridge joints + IMU (needs g1_state_server). "
                        "Implied by rviz:=true."),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="RViz with the robot model and the planned path"),

        # The only node that can make the robot move. Gate starts closed.
        Node(
            package="g1_walk",
            executable="loco_bridge",
            name="g1_loco_bridge",
            output="screen",
            parameters=[{
                "server_address": LaunchConfiguration("server_address"),
                "port": 5558,
                "rate": _f("rate"),
                "command_duration": _f("command_duration"),
                "start_enabled": False,
                "use_sim_time": False,
            }],
        ),

        # The robot's own state estimate -> /odom + TF. Harmless when the
        # robot is absent (it warns), and required for feedback control.
        Node(
            package="g1_walk",
            executable="odom_bridge",
            name="g1_odom_bridge",
            output="screen",
            condition=IfCondition(LaunchConfiguration("odom")),
            parameters=[{
                "server_address": LaunchConfiguration("server_address"),
                "port": 5560,
                "use_sim_time": False,
            }],
        ),

        OpaqueFunction(function=_resolve_path_file),

        # Below here is observation only -- nothing needed to walk.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            condition=IfCondition(state_enabled),
            parameters=[{
                "robot_description": robot_description,
                # robot_state_publisher does NOT simply follow /joint_states --
                # it republishes TF at publish_frequency, whose default is
                # 20 Hz regardless of how fast joints arrive.
                "publish_frequency": 200.0,
                "use_sim_time": False,
            }],
        ),

        Node(
            package="g1_walk",
            executable="state_bridge",
            name="g1_state_bridge",
            output="screen",
            condition=IfCondition(state_enabled),
            parameters=[{
                "server_address": LaunchConfiguration("state_address"),
                "port": 5557,
                "use_sim_time": False,
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", os.path.join(walk_share, "rviz", "walk.rviz")],
            parameters=[{"use_sim_time": False}],
        ),
    ])
