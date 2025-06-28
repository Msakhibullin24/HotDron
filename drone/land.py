from lib import HotDrone
import rospy

rospy.init_node('land')

drone = HotDrone()

drone.emergency_land()