import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped
import numpy as np
import time


def publish_callback(self):
    msg = Float64MultiArray()
    msg.data = self.array
    self.pub_array.publish(msg)

class NN:
    def __init__(self):
        print('Controller: initializing node')
        self.acc = np.zeros((1,51))
        self.array = np.zeros((1,51))
        self.sub_array = rospy.Subscriber("/acc", Float64MultiArray, self.acc_callback)
        self.pub_array = rospy.Publisher("/nuovo_acc", Float64MultiArray, queue_size=1)

    def acc_callback(self, msg):
        self.acc = msg.data
        self.array_data = np.array([self.acc])
        self.array = np.concatenate((self.array_data))

        publish_callback(self)
    
        

   

            
if __name__ == '__main__':
    print('starting listen topic.py')
    rospy.init_node('NN')
    try:
        node = NN()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')

        
   