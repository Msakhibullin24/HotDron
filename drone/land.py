from lib import HotDrone

rospy.init_node('land')

drone = HotDrone()

drone.emergency_land()