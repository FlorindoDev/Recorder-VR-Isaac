from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    talker = Node(
	    package='my_vr',
	    executable='pub.py', 
	    name='talker'
	)
	
    listener = Node(
	    package='my_vr',
	    executable='sub.py',
	    name='listener'
	)
	
    return LaunchDescription([talker, listener])