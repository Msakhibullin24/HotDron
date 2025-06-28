import math
import logging
import sys
import time
import threading

import rospy
from clover import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

class HotDrone:
    def __init__(self) -> None:
        self.autoland = rospy.ServiceProxy("land", Trigger)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.set_led = rospy.ServiceProxy('led/set_effect', srv.SetLEDEffect)

        self.force_arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
        self.logger.addHandler(handler)
        
        # Threading control for async publisher
        self.publisher_thread = None
        self.stop_publisher = threading.Event()

    def navigate_wait(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        yaw: float = float("nan"),
        speed: float = 0.5,
        frame_id: str = "",
        auto_arm: bool = False,
        tolerance: float = 0.2,
    ) -> None:
        self.logger.info(f"Navigating to x={x:.2f} y={y:.2f} z={z:.2f} in {frame_id}")
        self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        telem = self.get_telemetry(frame_id="body")
        self.logger.info(telem)
        if not telem.armed:
            raise RuntimeError("Arming failed!")

        while True:
            telem = self.get_telemetry(frame_id="navigate_target")
            if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < tolerance:
                self.wait(0.1)
                self.logger.info("Arrived at target")
                break
            self.wait(0.1)
    
    def takeoff(self, z=1.1, delay: float = 0.5) -> None:
        self.logger.info(f"Taking off to z={z:.2f}")
        self.set_led(effect='blink', r=255, g=255, b=255)

        self.force_arm(True)
        self.send_fake_pos_async(duration=3.0)  # Start async publisher
        self.wait(2)
        self.navigate_wait(z=z, speed=0.5, frame_id="body", auto_arm=True)

        telem = self.get_telemetry(frame_id="body")
        if not telem.armed:
            raise RuntimeError("Arming failed!")
        self.set_led(effect='blink', r=255, g=165, b=0)

        self.navigate(x=telem.x, y=telem.y, z=z, yaw=float("nan"), speed=0.5, frame_id="aruco_map")
        self.wait(delay)

        self.logger.info("Takeoff done")
        self.set_led(r=0, g=255, b=0)
        self.stop_fake_pos_async()  # Stop async publisher

    def land(self, z=0.5, delay: float = 4.0, frame_id="aruco_map") -> None:
        telem = self.get_telemetry(frame_id=frame_id)
        self.logger.info("Pre-landing")
        self.set_led(effect='blink', r=255, g=255, b=255)
        self.navigate_wait(x=telem.x, y=telem.y, z=z, frame_id=frame_id)
        self.wait(1.0)
        self.logger.info("Landing")
        self.set_led(effect='blink', r=255, g=165, b=0)
        self.wait(1.0)
        self.force_arm(False)
        self.wait(delay)
        self.logger.info("Landed")
        self.set_led(r=0, g=255, b=0)

    def wait(self, duration: float):
        rospy.sleep(duration)
        if rospy.is_shutdown():
            raise RuntimeError("rospy shutdown")
        
    def run(self) -> None:
        z = 1.1

        self.takeoff(z=z, delay=0.5)
        # while True:
        #     telem = self.get_telemetry(frame_id="body")
        #     self.logger.info(telem)
        #     self.wait(0.5)

        self.navigate_wait(x=0, y=0, z=z, yaw=math.pi, speed=0.5, frame_id="aruco_81", tolerance=0.2)
        # self.navigate_wait(x=0.5625, y=0.5625, z=z, speed=0.5, frame_id="aruco_map", tolerance=0.2)
        self.wait(2)

        self.land()

    def _fake_pos_publisher(self, duration=5.0):
        """Internal method to run the fake position publisher"""
        self.logger.info(f"Started vision pose publishing for {duration}s")
        pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        
        rate = rospy.Rate(50)  # 50Hz publishing rate
        start_time = time.time()
        initial_z = -1.3941157568261844
        target_z = initial_z + 20.0  # Move up 20 meters over duration
        
        while (time.time() - start_time < duration and 
               not rospy.is_shutdown() and 
               not self.stop_publisher.is_set()):
            
            elapsed = time.time() - start_time
            progress = elapsed / duration  # 0 to 1
            current_z = initial_z + (target_z - initial_z) * progress
            
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "body"
            msg.pose.position.x = -0.9737948572061019
            msg.pose.position.y = 1.3070810124006333
            msg.pose.position.z = -current_z  # Gradually increasing z
            msg.pose.orientation.x = -0.007933059759995434
            msg.pose.orientation.y = -0.009343381468109018
            msg.pose.orientation.z = -0.6053151200902469
            msg.pose.orientation.w = 0.7958915586785147
            pub.publish(msg)
            rate.sleep()
        
        self.logger.info("Stopped vision pose publishing")

    def send_fake_pos_async(self, duration=5.0):
        """Start fake position publisher in a separate thread"""
        if self.publisher_thread is not None and self.publisher_thread.is_alive():
            self.logger.warning("Publisher thread already running")
            return
        
        self.stop_publisher.clear()
        self.publisher_thread = threading.Thread(
            target=self._fake_pos_publisher, 
            args=(duration,),
            daemon=True
        )
        self.publisher_thread.start()
        self.logger.info("Started async fake position publisher")

    def stop_fake_pos_async(self):
        """Stop the async fake position publisher"""
        if self.publisher_thread is not None and self.publisher_thread.is_alive():
            self.stop_publisher.set()
            self.publisher_thread.join(timeout=1.0)  # Wait up to 1 second for thread to finish
            self.logger.info("Stopped async fake position publisher")

    def send_fake_pos(self, duration=5.0):
        """Send fake position data with gradual upward movement (synchronous version)"""
        self._fake_pos_publisher(duration)

    def emergency_land(self) -> None:
        self.stop_fake_pos_async()  # Stop any running publisher
        self.land()
