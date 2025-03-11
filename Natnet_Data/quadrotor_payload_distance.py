import rospy
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
import message_filters

class QuadrotorDataProcessor:
    def __init__(self):
        rospy.init_node('quadrotor_data_processor', anonymous=True)
        
        self.quadrotor_sub = message_filters.Subscriber('/natnet_ros/cf1/pose', PoseStamped)
        self.payload_sub = message_filters.Subscriber('/natnet_ros/payload/pose', PoseStamped)

        self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.quadrotor_sub, self.payload_sub], queue_size=20, slop=0.1)
        self.synchronizer.registerCallback(self.synced_callback)
        
        self.distance_pub = rospy.Publisher('/quadrotor_payload_distance', Range, queue_size=10)
        
        self.quadrotor_pose = None
        self.payload_pose = None

    def synced_callback(self, quadrotor_msg, payload_msg):
        self.quadrotor_pose = (quadrotor_msg.pose.position.x, quadrotor_msg.pose.position.y, quadrotor_msg.pose.position.z)
        self.payload_pose = (payload_msg.pose.position.x, payload_msg.pose.position.y, payload_msg.pose.position.z)
        

        if self.quadrotor_pose is not None and self.payload_pose is not None:
            dx = self.quadrotor_pose[0]- self.payload_pose[0]
            dy = self.quadrotor_pose[1]- self.payload_pose[1]
            dz = self.quadrotor_pose[2]- self.payload_pose[2]
            distance = math.sqrt(dx**2 + dy**2 + dz **2)

            range_msg = Range()
            range_msg.header.stamp = quadrotor_msg.header.stamp
            range_msg.header.frame_id = "world"
            range_msg.range = distance

            self.distance_pub.publish(range_msg)


if __name__ == "__main__":
    try:
        processor = QuadrotorDataProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass