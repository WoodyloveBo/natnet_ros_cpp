import rospy
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class QuadrotorDataProcessor:
    def __init__(self):
        rospy.init_node('quadrotor_data_processor', anonymous=True)
        
        rospy.Subscriber('/natnet_ros/cf1/pose', PoseStamped, self.quadrotor_callback)
        rospy.Subscriber('/natnet_ros/payload/pose', PoseStamped, self.payload_callback)
                
        self.distance_pub = rospy.Publisher('/quadrotor_payload_distance', Float32, queue_size=10)
        
        self.quadrotor_pose = None
        self.payload_pose = None

    def quadrotor_callback(self, msg):
        self.quadrotor_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.calculate_distance()

    def payload_callback(self, msg):
        self.payload_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.calculate_distance()

    def calculate_distance(self):
        if self.quadrotor_pose is not None and self.payload_pose is not None:
            dx = self.quadrotor_pose[0]- self.payload_pose[0]
            dy = self.quadrotor_pose[1]- self.payload_pose[1]
            dz = self.quadrotor_pose[2]- self.payload_pose[2]
            distance = math.sqrt(dx**2 + dy**2 + dz **2)

            self.distance_pub.publish(Float32(distance))


if __name__ == "__main__":
    try:
        processor = QuadrotorDataProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass