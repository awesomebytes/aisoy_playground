#!/usr/bin/env python

import rospy
from airos4_msgs.srv import SetString, SetStringRequest, Say, SayRequest
from std_msgs.msg import String


SET_GRAM_SRV = '/airos4/asr/set_grammar'

def asr_cb(data):
    print "he recibido: " + str(data)
    tts_srv = rospy.ServiceProxy('/tts/say', Say)
    r = SayRequest()
    r.sentence = "He entendido  " + data.data
    r.moveMouth = True
    tts_srv.call(r)


if __name__ == '__main__':
    rospy.init_node('repeat_heard_node')
    # rosservice call /airos4/asr/set_language "data: 'es'"
    # rosservice call /airos4/asr/set_grammar "data: 'derecha | izquierda'"
    # rosservice call /airos4/tts/set_language "data: 'es'"

    # set_str_srv = rospy.ServiceProxy(SET_GRAM_SRV, SetString)
    # r = SetStringRequest()
    # r.data = 'hi | bye | yes | no'
    # rospy.loginfo("Doing service call to: " + SET_GRAM_SRV)
    # set_str_srv.call(r)
    # rospy.loginfo("Done!")

    # rostopic echo /airos4/asr/recognition

    asr_sub = rospy.Subscriber('/airos4/asr/recognition', String, asr_cb)
    rospy.spin()


# rosservice call /airos4/asr/set_language "data: 'en'"
# rosservice call /airos4/asr/set_grammar "data: 'hello my friend|bye'"
# roslaunch airos4_asr asr.launch
# roslaunch airos4_mouth mouth.launch
# roslaunch airos4_tts tts_nodelet.launch