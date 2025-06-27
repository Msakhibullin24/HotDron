import math
import logging

import rospy
from clover import srv
from std_srvs.srv import Trigger


class HotDrone:
    def __init__(self) -> None:
        self.autoland = rospy.ServiceProxy("land", Trigger)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.set_led = rospy.ServiceProxy('led/set_effect', srv.SetLEDEffect)
        self.logger = logging.getLogger(__name__)

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

        self.navigate_wait(z=z, speed=0.5, frame_id="body", auto_arm=True)

        telem = self.get_telemetry(frame_id="body")
        if not telem.armed:
            raise RuntimeError("Arming failed!")
        self.set_led(effect='blink', r=255, g=165, b=0)

        self.wait(delay)
        self.navigate(x=telem.x, y=telem.y, z=z, yaw=float("nan"), speed=0.5, frame_id="aruco_map")

        self.logger.info("Takeoff done")
        self.set_led(r=0, g=255, b=0)

    def land(self, z=0.5, delay: float = 4.0, frame_id="aruco_map") -> None:
        telem = self.get_telemetry(frame_id=frame_id)
        self.logger.info("Pre-landing")
        self.set_led(effect='blink', r=255, g=255, b=255)
        self.navigate_wait(x=telem.x, y=telem.y, z=z, frame_id=frame_id)
        self.wait(1.0)
        self.logger.info("Landing")
        self.set_led(effect='blink', r=255, g=165, b=0)
        self.autoland()
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
        self.wait(10)

        self.navigate_wait(x=0, y=0, z=z, speed=0.5, frame_id="aruco_81", tolerance=0.2)
        # self.navigate_wait(x=0.5625, y=0.5625, z=z, speed=0.5, frame_id="aruco_map", tolerance=0.2)
        self.wait(10)

        self.land()
        self.wait(10)

