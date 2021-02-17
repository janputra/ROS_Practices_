#!/usr/bin/env python
from ros_service_assignment.srv import RectangleAreaService
from ros_service_assignment.srv import RectangleAreaServiceRequest
from ros_service_assignment.srv import RectangleAreaServiceResponse
import rospy

def handler(req):
    area = req.width*req.height
    print("Width : %s , Height : %s"%(req.width,req.height))
    print("Area : %s" %area)
    print("Returning Calculation")
    return RectangleAreaServiceResponse(area)

def Rec_server():
    rospy.init_node('Rec_server')
    s= rospy.Service('Rec_area',RectangleAreaService,handler)
    print('Ready to calculate area')
    rospy.spin()

if __name__=="__main__":
    Rec_server()