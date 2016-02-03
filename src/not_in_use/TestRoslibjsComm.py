__author__ = 'aishwarya'

import rospy, time, random, string  
from std_msgs.msg import String

def publish_users() :
    topic_name = '/new_user_topic'
    pub = rospy.Publisher(topic_name, String, queue_size=1, tcp_nodelay=True)
    while True :
        N = 5
        # Create a random string of length N
        rand_str = ''.join(random.choice(string.ascii_uppercase + string.digits) for _ in range(N))
        pub.publish(rand_str)
        time.sleep(3)
    
if __name__ == '__main__' :
    rospy.init_node('test_roslibjs_comm')
    publish_users()
