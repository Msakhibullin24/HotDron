from lib import HotDrone
import rospy

rospy.init_node('flight')

drone = HotDrone()

drone.run()