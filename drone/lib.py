import math
import logging
import socket
import sys
import time
import threading
import os
import requests
import json

class UDPLogHandler(logging.Handler):
    """Custom logging handler that sends logs to UDP server"""
    def __init__(self, server_ip='192.168.2.69', server_port=9999, drone_name='unknown_drone'):
        super().__init__()
        self.server_ip = server_ip
        self.server_port = server_port
        self.drone_name = drone_name
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
    def emit(self, record):
        try:
            # Map Python logging levels to our custom log types
            level_mapping = {
                'DEBUG': 'info',
                'INFO': 'drone',
                'WARNING': 'warning',
                'ERROR': 'error',
                'CRITICAL': 'critical'
            }
            log_type = level_mapping.get(record.levelname, 'info')
            
            # Format message: drone_id|log_type|message
            message = f"{self.drone_name}|{log_type}|{record.getMessage()}"
            
            # Send to UDP server
            self.sock.sendto(message.encode('utf-8'), (self.server_ip, self.server_port))
        except Exception:
            # Don't raise exceptions in logging handler
            pass
    
    def close(self):
        if hasattr(self, 'sock'):
            self.sock.close()
        super().close()


class DroneCoords:
    """Object to hold drone coordinates with x, y, z attributes"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z
    
    def __str__(self):
        return f"DroneCoords(x={self.x}, y={self.y}, z={self.z})"

def get_drone_coords(drone_name) -> DroneCoords:
    """
    Get coordinates for a specific drone name from the circle-sheep API endpoint.
    
    Args:
        drone_name (str): Single drone name to get coordinates for
        
    Returns:
        DroneCoords: Object with x, y, z attributes
    """
    api_url = 'http://192.168.2.95:8000'
    
    try:
        response = requests.get(f"{api_url}/circle-sheep")
        response.raise_for_status()
        
        positions = response.json()
        
        if not isinstance(positions, list):
            print("Error: Invalid response format from server.")
            return DroneCoords()
        
        # Search for the drone by name in the response
        for pos in positions:
            if pos.get('drone_name') == drone_name:
                coords = pos.get('coords')
                
                if coords and len(coords) >= 3:
                    x, y, z = coords[0], coords[1], coords[2]
                    print(f"Got coordinates for {drone_name}: x={x}, y={y}, z={z}")
                    return DroneCoords(x, y, z)
                else:
                    print(f"Warning: Invalid coordinates for {drone_name}")
                    return DroneCoords()
        
        # If drone not found in response
        print(f"Error: Drone '{drone_name}' not found in API response.")
        return DroneCoords()
        
    except requests.exceptions.RequestException as e:
        print(f"Error making request to /circle-sheep: {e}")
        return DroneCoords()
    except json.JSONDecodeError:
        print("Error: Could not decode JSON response from server.")
        return DroneCoords()

import rospy
from clover import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode

class HotDrone:
    def __init__(self) -> None:
        self.autoland = rospy.ServiceProxy("land", Trigger)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.set_led = rospy.ServiceProxy('led/set_effect', srv.SetLEDEffect)
        self.set_mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        self.force_arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        self.initial_z = 0

        self.drone_name = os.environ.get('DRONE_NAME', 'unknown_drone')
        print(f"Running on drone: {self.drone_name}")
        
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
        self.logger.addHandler(handler)
        
        # UDP handler (new)
        try:
            self.udp_handler = UDPLogHandler(
                server_ip='192.168.2.69', 
                server_port=9999, 
                drone_name=self.drone_name
            )
            self.logger.addHandler(self.udp_handler)
            self.logger.info("UDP logging initialized successfully")
        except Exception as e:
            print(f"Failed to initialize UDP logging: {e}")
            self.udp_handler = None
        
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
        if not telem.armed:
            raise RuntimeError("Arming failed!")

        while True:
            telem = self.get_telemetry(frame_id="navigate_target")
            if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < tolerance:
                self.wait(0.1)
                self.logger.info("Arrived at target")
                break
            self.wait(0.1)
    
    def takeoff(self, z=1.1, delay: float = 0.5, time_spam: float = 2.5, time_warm: float = 2, time_up: float = 0.5) -> None:
        self.logger.info(f"Taking off to z={z:.2f}")
        self.set_led(effect='blink', r=255, g=255, b=255)

        self.force_arm(True)
        self.send_fake_pos_async(duration=time_spam)  # Start async publisher
        self.wait(time_warm)
        self.navigate(z=100, speed=0.5, frame_id="body", auto_arm=True)
        self.wait(time_up)
        telem = self.get_telemetry(frame_id="aruco_map")
        if not telem.armed:
            raise RuntimeError("Arming failed!")
        self.set_led(effect='blink', r=255, g=165, b=0)

        self.navigate(x=telem.x, y=telem.y, z=z, yaw=math.pi, speed=0.5, frame_id="aruco_map")
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
        telem = self.get_telemetry(frame_id="base_link")
        self.navigate(x=telem.x, y=telem.y, z=-1, speed=1, frame_id="base_link")
        self.wait(1)
        self.navigate(x=telem.x, y=telem.y, z=-1, speed=0, frame_id="base_link")
        #self.autoland()
        self.force_arm(False)
        self.logger.info("Landed")
        self.set_mode_service(custom_mode="AUTO.LAND")
        self.wait(0.2)
        while self.get_telemetry(frame_id="body").mode != "STABILIZED":
            self.set_mode_service(custom_mode="STABILIZED")
            self.wait(3)
        self.wait(1)
        self.force_arm(True)
        self.wait(1)
        self.force_arm(False)
        self.set_led(r=0, g=255, b=0)

    def wait(self, duration: float):
        rospy.sleep(duration)
        if rospy.is_shutdown():
            raise RuntimeError("rospy shutdown")
        
    def run(self) -> None:
        z = 1.1
        # while True:
        #     telem = self.get_telemetry(frame_id="body")
        #     self.logger.info(telem)
        #     self.wait(0.5)
        telem = self.get_telemetry(frame_id="aruco_map")
        self.logger.info(f"Mode: {telem.mode} Arm: {telem.armed}")
        if self.drone_name == "drone5":
            self.takeoff(z=1.5, delay=0.5, time_spam=3.5, time_warm=2, time_up=1.5)
        elif self.drone_name == "drone10":
            self.takeoff(z=1.5, delay=0.5, time_spam=3, time_warm=2, time_up=0.5)
        elif self.drone_name == "drone8":
            self.takeoff(z=1.5, delay=0.5, time_spam=3.5, time_warm=2, time_up=1.5)
        elif self.drone_name == "drone12":
            self.takeoff(z=1.5, delay=0.5, time_spam=3, time_warm=2, time_up=0.5)
        else:
            # Default fallback if drone name not recognized
            self.logger.warning(f"Unknown drone name: {self.drone_name}, using default aruco_81")
            self.navigate_wait(x=0, y=0, z=z, yaw=math.pi, speed=0.5, frame_id="aruco_81", tolerance=0.2)
        # self.navigate_wait(x=0.5625, y=0.5625, z=z, speed=0.5, frame_id="aruco_map", tolerance=0.2)
        self.wait(2)
        telem = self.get_telemetry(frame_id="aruco_map")
        self.logger.info(f"Mode: {telem.mode} Arm: {telem.armed}")
        ''''''
        self.wait(2)
        telem = self.get_telemetry(frame_id="aruco_map")
        self.logger.info(f"Mode: {telem.mode} Arm: {telem.armed}")
        self.wait(2)
        cords = get_drone_coords(drone_name=self.drone_name)
        self.wait(2)
        self.navigate_wait(x=cords.x, y=cords.y, z=1.1, yaw=math.pi, speed=0.3, frame_id="aruco_map", tolerance=0.1)
        self.wait(2)
        self.land()
        ''''''
        # self.land()
    def _fake_pos_publisher(self, duration=5.0):
        """Internal method to run the fake position publisher"""
        self.logger.info(f"Started vision pose publishing for {duration}s")
        pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        
        rate = rospy.Rate(50)  # 50Hz publishing rate
        start_time = time.time()
        
        target_z = self.initial_z + 20.0  # Move up 20 meters over duration
        current_z_temp = 0
        while (time.time() - start_time < duration and 
               not rospy.is_shutdown() and 
               not self.stop_publisher.is_set()):
            
            elapsed = time.time() - start_time
            progress = elapsed / duration  # 0 to 1
            current_z = self.initial_z + (target_z - self.initial_z) * progress
            current_z_temp = current_z
            
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "body"
            msg.pose.position.x = 0
            msg.pose.position.y = 0
            msg.pose.position.z = -current_z  # Gradually increasing z
            msg.pose.orientation.x = 0
            msg.pose.orientation.y = 0
            msg.pose.orientation.z = 0
            msg.pose.orientation.w = 0.7958915586785147
            pub.publish(msg)
            rate.sleep()
        
        self.logger.info("Stopped vision pose publishing")

        self.initial_z = current_z_temp

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
