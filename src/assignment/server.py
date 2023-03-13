import rospy
from ros_service_assignment.srv import RectangleAreaService

def handle_calculate_rectangle_area(req):
    result = req.width * req.height
    print("%s + %s = %s"%(req.a, req.b, result))
    return result

def setup_server():
    rospy.init_node("RectangleAreaServer")
    server = rospy.Service("/calculate_rectangle_area", RectangleAreaService, handle_calculate_rectangle_area)
    
    rospy.spin()

if __name__ == "__main__":
    setup_server()