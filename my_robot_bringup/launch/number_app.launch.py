# minimal code
# to create a node import a node object
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # this method name here aboove should be exactly this way
    # as when  you install the launch file  the the
    # launch functionality  will actually create a new program
    # which will take that function as it is  to launch the application
    # however if it does not find that exact name nothing will happen
    ld = LaunchDescription()
    # you must create nodes in this launch file to call the
    # other node in the other packages

    # and since for the topic you may require to change it to a similar vale for various
    # nodes you can declare the tuple outside and then just be calling it
    rename_number_topic = ("/number", "/my_number23")

    number_publisher_node = Node(
        package="my_py_pkg",
        executable="number_publisher",
        name="my_number_publisher",
        # to rename a topic you can utilise this array list of tuples
        # A list can also be renamed the same way like a topic
        remappings=[rename_number_topic],
        # to add parameters to your program,we won't use the array list of tuples
        # but instead, we will use an array list of dictionaries
        parameters=[{"test321": 10, "publish_freq": 2.0}],
    )
    number_counter_node = Node(
        package="my_py_pkg",
        executable="summernode",
        name="my_number_counter",
        remappings=[rename_number_topic, ("/number_count", "/my_number_count")],
    )
    # to rename a node just add the variable name,as seen here above

    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)

    return ld
    pass
