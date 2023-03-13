import rospy
import sys
from ros_service_assignment.srv import RectangleAreaService

def setup_client(width, height):
    rospy.wait_for_service("/calculat_rectangle_area")
    try:
        client = rospy.ServiceProxy("/calculate_rectangle_area", RectangleAreaService)
        response = client(width, height)
        return response.area
    except rospy.ServiceException as e:
        rospy.logwarn("Service failed: " + str(e))

if __name__ == "__main__":
    if len(sys.argv) == 3:
        width = int(sys.argv[1])
        height = int(sys.argv[2])
    else:
        print ("%s [x y]"%sys.argv[0])
        sys.exit(1)
    c = setup_client(width, height)
    print ("%s + %s = %s"%(width, height, c))