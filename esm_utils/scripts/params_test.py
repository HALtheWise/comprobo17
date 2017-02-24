#!/usr/bin/env python

import rospy

rospy.init_node('cat')

from esm_utils_2.ddyn import params, NewParam

params.x = params.New(5)

def callback(config, level):
    print "Config callback triggered: {}".format(config)
    return config

params._ddr.start(callback)

def run():
    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        print "Parameters are unknown"
        r.sleep()


if __name__ == '__main__':
    run()


