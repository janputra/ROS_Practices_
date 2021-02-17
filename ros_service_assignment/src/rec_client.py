#!/usr/bin/env python
import sys
import rospy
from ros_service_assignment.srv import RectangleAreaService
from ros_service_assignment.srv import RectangleAreaServiceRequest
from ros_service_assignment.srv import RectangleAreaServiceResponse

def request_area(w,h):
    rospy.wait_for_service('Rec_area')
    try:
        area = rospy.ServiceProxy('Rec_area',RectangleAreaService)
        resp= area(w,h)
        return resp.area
    except rospy.ServiceException(e):
        print("Service call failled : %s"%e)

if __name__=="__main__":
    if len(sys.argv) == 3:
        w= int(sys.argv[1])
        h= int(sys.argv[2])
    else:
        print("%s"%sys.argv[0])
        sys.exit(1)
    print("Requesting Area of W : %s H: %s"%(w,h))
    s= request_area(w,h)
    print("Calculated Area : %s"%s)