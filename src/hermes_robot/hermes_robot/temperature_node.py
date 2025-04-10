import rospy
from std_msgs.msg import Float32

file_name = "/sys/class/thermal/thermal_zone0/temp"

def read_temperature():
    try:
        with open(file_name, 'r') as file:
            temp_str = file.read().strip()
            return float(temp_str) / 1000.0  # Convert millidegree Celsius to degree Celsius
    except Exception as e:
        rospy.logerr(f"Failed to read temperature: {e}")
        return None

def temperature_publisher():
    rospy.init_node('temperature_node', anonymous=True)
    pub = rospy.Publisher('temperature', Float32, queue_size=10)
    rate = rospy.Rate(1)  # Publish at 1 Hz

    while not rospy.is_shutdown():
        temperature = read_temperature()
        if temperature is not None:
            pub.publish(temperature)
        rate.sleep()

if __name__ == '__main__':
    try:
        temperature_publisher()
    except rospy.ROSInterruptException:
        pass