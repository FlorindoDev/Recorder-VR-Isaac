from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    talker = Node(
	    package='my_prova',
	    executable='talker', 
	    name='talker'
	)
	
    listener = Node(
	    package='my_prova',
	    executable='listener', 
	    name='listener'
	)
	
    return LaunchDescription([talker, listener])