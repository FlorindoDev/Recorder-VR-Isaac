"""
ROS2 Humble node to read OpenVR controller data and publish it on a topic.

This node initializes the OpenVR runtime, queries all tracked devices,
filters for controllers, reads their joystick axes and trigger values,
and publishes these values as Float32MultiArray on `/openvr/controller`.
"""
import rclpy  # ROS 2 client library
from rclpy.node import Node  # Base class for ROS 2 nodes
from std_msgs.msg import Float32MultiArray  # Message type for multi-dimensional float array

# OpenVR Python bindings; install via `pip install openvr` or `pip install pyopenvr`
import openvr


def get_controller_states(vr_system):
    """
    Query all tracked devices and return a list of controller states.

    Each entry is a tuple: (device_index, axis_x, axis_y, trigger_value).
    """
    states = []
    # Iterate over maximum possible tracked devices
    for device_index in range(openvr.k_unMaxTrackedDeviceCount):
        # Determine the type of the tracked device
        device_class = vr_system.getTrackedDeviceClass(device_index)
        # We're only interested in controller devices
        if device_class == openvr.TrackedDeviceClass_Controller:
            # Retrieve controller state (buttons, axes, etc.)
            got_state, state = vr_system.getControllerState(device_index)
            if got_state:
                # Axis 0 is typically the thumbstick or touchpad: (x,y)
                axis_x = state.rAxis[0].x
                axis_y = state.rAxis[0].y
                # Axis 1 is often the trigger analog value
                trigger = state.rAxis[1].x
                # Append tuple of controller index and values
                states.append((device_index, axis_x, axis_y, trigger))
    return states


class OpenVRPublisher(Node):
    """
    ROS 2 node that publishes OpenVR controller data at a fixed rate.
    """
    def __init__(self):
        super().__init__('openvr_publisher')
        # Create publisher: topic name, message type, queue size
        self.pub = self.create_publisher(Float32MultiArray, 'openvr/controller', 10)

        # Initialize OpenVR runtime in an application context
        openvr.init(openvr.VRApplication_Other)
        # Obtain the VRSystem interface for querying devices
        self.vr_system = openvr.VRSystem()

        # Setup a timer to call `timer_callback` every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('OpenVRPublisher initialized, publishing every 0.1s')

    def timer_callback(self):
        """
        Callback that runs on a timer to read controller states and publish them.
        """
        # Create the multi-array message
        msg = Float32MultiArray()
        data = []

        # Retrieve latest controller states
        states = get_controller_states(self.vr_system)
        for idx, x, y, trig in states:
            # Flatten each tuple into the data array
            data.extend([float(idx), x, y, trig])

        msg.data = data
        self.pub.publish(msg)
        self.get_logger().debug(f'Published controller data: {data}')

    def destroy_node(self):
        """
        Clean up the OpenVR runtime on node shutdown.
        """
        openvr.shutdown()
        super().destroy_node()


def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)
    node = OpenVRPublisher()

    try:
        # Spin to keep the node alive and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down')

    # Cleanly destroy the node and shutdown ROS 2
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
