#!/usr/bin/env python

import rospy
import random
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from iteration_learning import IterationLearning

random_generate = False


class IRLParameterUpdate:
    """ """
    def __init__(self):

        self.safetyFactor = 1.6
        self.rho = 9.0

        self.arr = Float32MultiArray()
        self.arr.layout.dim.append(MultiArrayDimension())
        self.arr.layout.dim[0].size = 2
        self.arr.layout.dim[0].stride = 1

        # init the node
        rospy.init_node('irl_parameter_update', anonymous=False)

        # publish the computed parameters
        self.publisher = rospy.Publisher('/parameters_tuning', Float32MultiArray, queue_size=1)

        self.il = IterationLearning([0, 8], [0.9, 1.6])
        X = np.array([[0, 0.9],
                      [8, 1.6]])
        y = np.array([0, 1])

        self.il.two_init_point(X, y)

        self.rate = rospy.Rate(200)

        self.i = 0

    def compute_parameter(self, data):
        """ """
        if random_generate:
            # random generated
            self.safetyFactor = 1 + random.random()*0.5
            self.rho = 1 + 7*random.random()
        else:
            # print(data.data)
            trail = self.il.run(data.data)
            self.safetyFactor = trail[1]
            self.rho = trail[0]
        # self.arr.data[0] = self.safetyFactor
        # self.arr.data[1] = self.rho
        self.arr.data = [self.safetyFactor, self.rho]

    def call_back(self, data):
        """ """
        # data.data is the float given back

        print('received', data)
        if self.i == 0:
            trail = self.il.update_data_x()
            self.safetyFactor = trail[1]
            self.rho = trail[0]
            self.arr.data = [self.safetyFactor, self.rho]
        else:
            self.compute_parameter(data)

        self.publisher.publish(self.arr)

        print('sending rho : ', self.rho)
        print('sending sf : ', self.safetyFactor)

        self.i += 1

        # rospy.spin()

    def run_the_node(self):
        """ """
        # subscribe to motion generator
        rospy.Subscriber('/motion_generator_to_parameter_update', Float32, self.call_back)

        rospy.spin()


if __name__ == '__main__':
    try:
        x = IRLParameterUpdate()
        x.run_the_node()
    except rospy.ROSInterruptException:
        pass
