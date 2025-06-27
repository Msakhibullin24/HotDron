from lib import HotDrone

rospy.init_node('flight')

drone = HotDrone()

drone.run()