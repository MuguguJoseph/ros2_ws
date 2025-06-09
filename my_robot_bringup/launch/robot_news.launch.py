from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    remap_topic_name = ("/dhjwndnm", "/my_robot_news")
    robot_names = ["CP30", "BYD", "U8", "tesla", "GLE", "V8"]
    robots_nesws_station_nodes = []

    for robot_name in robot_names:
        robots_nesws_station_nodes.append(
            Node(
                package="my_py_pkg",
                executable="robots_news_station",
                name="my_robot_news_station_" + robot_name,
                parameters=[{"robot_name": robot_name}],
                remappings=[remap_topic_name],
            )
        )

    robot7_news_station_node = Node(
        package="my_py_pkg", executable="smartphone", remappings=[remap_topic_name]
    )

    for node in robots_nesws_station_nodes:
        ld.add_action(node)
        pass
    ld.add_action(robot7_news_station_node)
    return ld

    """robot1_news_station_node = Node(
        package="my_py_pkg",
        executable="robots_news_station",
        name="my_robot_news_station_CP30",
        parameters=[{"robot_name": "CP30"}],
        remappings=[remap_topic_name],
    )
    robot2_news_station_node = Node(
        package="my_py_pkg",
        executable="robots_news_station",
        name="my_robot_news_station_BYD",
        parameters=[{"robot_name": "BYD"}],
        remappings=[remap_topic_name],
    )
    robot3_news_station_node = Node(
        package="my_py_pkg",
        executable="robots_news_station",
        name="my_robot_news_station_U8",
        parameters=[{"robot_name": "U8"}],
        remappings=[remap_topic_name],
    )
    robot4_news_station_node = Node(
        package="my_py_pkg",
        executable="robots_news_station",
        name="my_robot_news_station_tesla",
        parameters=[{"robot_name": "tesla"}],
        remappings=[remap_topic_name],
    )
    robot5_news_station_node = Node(
        package="my_py_pkg",
        executable="robots_news_station",
        name="my_robot_news_station_GLE",
        parameters=[{"robot_name": "GLE"}],
        remappings=[remap_topic_name],
    )
    robot6_news_station_node = Node(
        package="my_py_pkg",
        executable="robots_news_station",
        name="my_robot_news_station_V8",
        parameters=[{"robot_name": "V8"}],
        remappings=[remap_topic_name],
    )
    

    ld.add_action(robot1_news_station_node)
    ld.add_action(robot2_news_station_node)
    ld.add_action(robot3_news_station_node)
    ld.add_action(robot4_news_station_node)
    ld.add_action(robot5_news_station_node)
    ld.add_action(robot6_news_station_node)
    ld.add_action(robot7_news_station_node)
"""
