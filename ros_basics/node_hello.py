#!/usr/bin/env python

import rospy


def main():    
    
    rospy.init_node('node_hello_ros', anonymous=True)

    rospy.loginfo("Hello World!")

    param_config_my = rospy.get_param('details')
    first_name = param_config_my['name']['first']
    phone = param_config_my['contact']['phone']
    rospy.loginfo(first_name)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
