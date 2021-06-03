import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped
import numpy as np
import time


matrix=[]
def array_callback(self):
    msg = Float64MultiArray()
    msg.data = self.array
    self.pub_array.publish(msg)

class dati:
    def __init__(self):
        print('Controller: initializing node')
        self.F_array = np.zeros((1,3))
        self.acc_array = np.zeros((1,51))
        self.gyro_array = np.zeros((1,51))
        self.hand_array = np.zeros((1,1))
        self.array= np.zeros((1,106))
        self.sub_hand = rospy.Subscriber("hand_opening", Float64, self.hand_callback)
        self.sub_F_ext = rospy.Subscriber("F_ext", WrenchStamped, self.F_ext_callback)
        self.sub_acc = rospy.Subscriber("acc", Float64MultiArray, self.acc_callback)
        self.sub_gyro = rospy.Subscriber("gyro", Float64MultiArray, self.gyro_callback)
        self.pub_array = rospy.Publisher("array", Float64MultiArray, queue_size=1)


    def hand_callback(self, msg):

        self.hand = msg.data
        self.hand_array = np.array([self.hand])

    def F_ext_callback(self, msg):

        self.F_ext_x = msg.wrench.force.x
        self.F_ext_y = msg.wrench.force.y
        self.F_ext_z = msg.wrench.force.z
        self.F_array = np.array([self.F_ext_x, self.F_ext_y, self.F_ext_z])
    
    def acc_callback(self, msg):

        self.acc = msg.data
        self.acc_array = np.array([self.acc])
    
    def gyro_callback(self, msg):

        self.gyro = msg.data
        self.gyro_array = np.array([self.gyro])

        
######  
        self.array = np.concatenate((self.acc_array, self.gyro_array, self.hand_array, self.F_array), axis=None)
        array_callback(self)
        matrix.append(self.array)
        len_matrix = len(matrix)
        print'Array of data %s' %self.array
               

            
if __name__ == '__main__':
    print('starting listen topic.py')
    rospy.init_node('dati')
    try:
        node = dati()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')

        
   