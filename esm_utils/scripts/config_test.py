#!/usr/bin/env python

import rospy
from dynamic_reconfigure.server import Server
from esm_utils.cfg import Test1Config

rospy.init_node('config_test')

a = 0
b = 0


def callback(config, level):
    print "Config callback triggered: {}".format(config)
    global a, b
    a = config['a']
    b = config['b']
    return config


srv = Server(Test1Config, callback)
srv.update_configuration()

def run():
    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        print "Parameters are a={a} and b={b}".format(a=a, b=b)
        r.sleep()


if __name__ == '__main__':
    run()
