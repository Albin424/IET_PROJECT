import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from mavros_msgs.msg import CommandTOL

# Global variables
landing_altitude = 0.5  # Altitude at which the drone should land

# Callback function for the marker info topic
def marker_info_callback(data):
    # Extract the pose and angle information from the topic data
    drone_pose = data.pose
    angle = data.angle

    # Check if the drone is above the landing altitude
    if drone_pose.position.z > landing_altitude:
        # Command the drone to descend at a fixed rate
        descent_rate = 0.2  # Adjust this value based on your needs
        setpoint_pose = PoseStamped()
        setpoint_pose.pose.position.z = max(drone_pose.position.z - descent_rate, landing_altitude)
        setpoint_pose.header.stamp = rospy.Time.now()
        setpoint_pose.header.frame_id = 'base_link'  # Adjust the frame ID based on your setup

        # Publish the setpoint pose for the drone to descend
        setpoint_pose_pub.publish(setpoint_pose)
    else:
        # Perform landing logic (e.g., land command, disarm, etc.)
        land_command = CommandTOL()
        land_command.min_pitch = angle
        land_command.yaw = 0.0  # Adjust the yaw angle if needed
        land_command.latitude = drone_pose.position.x  # Adjust the latitude coordinate
        land_command.longitude = drone_pose.position.y  # Adjust the longitude coordinate
        land_command.altitude = 0.0  # Adjust the target altitude for landing
        land_command.header.stamp = rospy.Time.now()
        land_command.header.frame_id = 'base_link'  # Adjust the frame ID based on your setup

        # Publish the land command
        land_command_pub.publish(land_command)

# Main function
if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('landing_node', anonymous=True)

    # Subscribe to the marker info topic
    rospy.Subscriber('marker_info', PoseStamped, marker_info_callback)

    # Create publishers for setpoint pose and land command
    setpoint_pose_pub = rospy.Publisher('setpoint_pose', PoseStamped, queue_size=10)
    land_command_pub = rospy.Publisher('land_command', CommandTOL, queue_size=10)

    # Spin the ROS node to receive messages
    rospy.spin()

