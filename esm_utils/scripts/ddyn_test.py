#!/usr/bin/env python

import rospy
from esm_utils_2.ddyn import DDynamicReconfigure

rospy.init_node('ddyn_test')

a = 0
b = 0


def callback(config, level):
    print "Config callback triggered: {}".format(config)
    global a, b
    a = config['a']
    b = config['b']
    return config


ddr = DDynamicReconfigure()
ddr.add_variable("a", "float/double variable", 0.0, -1.0, 1.0)
ddr.add_variable("b", "integer variable", 0, -1, 1)

ddr.start(callback)


def run():
    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        print "Parameters are a={a} and b={b}".format(a=a, b=b)
        r.sleep()


if __name__ == '__main__':
    run()
