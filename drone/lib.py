import math
import logging
import sys
import time
import threading
import os
import requests
import json

class DroneCoords:
    """Object to hold drone coordinates with x, y, z attributes"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z
    
    def __str__(self):
        return f"DroneCoords(x={self.x}, y={self.y}, z={self.z})"

class GameAPI:
    """API client for the sheep-wolf game"""
    def __init__(self, api_url='http://192.168.2.95:8000'):
        self.api_url = api_url
    
    def get_game_state(self):
        """Get current game state"""
        try:
            response = requests.get(f"{self.api_url}/game-state")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Error getting game state: {e}")
            return None
    
    def start_game(self):
        """Start the game"""
        try:
            response = requests.get(f"{self.api_url}/start")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Error starting game: {e}")
            return None
    
    def stop_game(self):
        """Stop the game"""
        try:
            response = requests.get(f"{self.api_url}/stop")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Error stopping game: {e}")
            return None

def wait_for_game_start(drone_name, game_api=None, check_interval=2.0):
    """Wait until game starts - blocking function"""
    if game_api is None:
        game_api = GameAPI()
    
    print(f"{drone_name} waiting for game to start...")
    while not rospy.is_shutdown():
        game_state = game_api.get_game_state()
        if game_state and game_state.get('status') == 'active':
            print(f"{drone_name}: Game started!")
            return True
        print(f"{drone_name}: Game status: {game_state.get('status') if game_state else 'unknown'}. Waiting...")
        time.sleep(check_interval)
    return False

def wait_for_drone_move(drone_name, game_api=None, check_interval=1.0):
    """
    Wait for this drone's turn and return coordinates - BLOCKING function
    Only returns when it's actually time for this drone to move
    """
    if game_api is None:
        game_api = GameAPI()
    
    try:
        drone_number = int(drone_name.replace('drone', ''))
    except ValueError:
        print(f"Invalid drone name format: {drone_name}")
        return None
    
    print(f"🤖 {drone_name} waiting for turn...")
    
    while not rospy.is_shutdown():
        game_state = game_api.get_game_state()
        
        if not game_state:
            print(f"{drone_name}: Could not get game state, retrying...")
            time.sleep(check_interval)
            continue
        
        # If game is not active, keep waiting
        if game_state.get('status') != 'active':
            print(f"{drone_name}: Game not active (status: {game_state.get('status')}), waiting...")
            time.sleep(check_interval)
            continue
        
        # Check if it's this drone's turn
        current_drone = game_state.get('drone')
        if current_drone != drone_number:
            # Not this drone's turn, keep waiting silently
            time.sleep(check_interval)
            continue
        
        # It's this drone's turn! Get coordinates
        target_coords = game_state.get('to')
        if target_coords and isinstance(target_coords, list) and len(target_coords) >= 3:
            x, y, z = target_coords[0], target_coords[1], target_coords[2]
            print(f"🎯 {drone_name} turn! Moving to x={x:.3f}, y={y:.3f}, z={z:.3f}")
            return DroneCoords(x, y, z)
        else:
            print(f"{drone_name}: Invalid coordinates: {target_coords}")
            time.sleep(check_interval)
            continue
    
    print(f"{drone_name}: ROS shutdown detected")
    return None

def trigger_next_move(game_api=None):
    """
    Trigger the next move in the game (this would call /start to continue the game)
    """
    if game_api is None:
        game_api = GameAPI()
    
    result = game_api.start_game()
    if result:
        print("✅ Triggered next game move")
        return True
    else:
        print("❌ Failed to trigger next move")
        return False

def get_current_drone_move(drone_name, game_api=None):
    """
    Non-blocking version - returns coordinates if it's drone's turn, None otherwise
    """
    if game_api is None:
        game_api = GameAPI()
    
    try:
        drone_number = int(drone_name.replace('drone', ''))
    except ValueError:
        return None
    
    game_state = game_api.get_game_state()
    if (not game_state or 
        game_state.get('status') != 'active' or 
        game_state.get('drone') != drone_number):
        return None
    
    target_coords = game_state.get('to')
    if target_coords and isinstance(target_coords, list) and len(target_coords) >= 3:
        return DroneCoords(target_coords[0], target_coords[1], target_coords[2])
    
    return None

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

        self.navigate(x=telem.x, y=telem.y, z=z, yaw=math.pi, speed=0.3, frame_id="aruco_map")
        self.wait(delay)

        self.logger.info("Takeoff done")
        self.set_led(r=0, g=255, b=0)
        self.stop_fake_pos_async()  # Stop async publisher

    def land(self, prl_aruco: str = "aruco_map", prl_speed=0.5, prl_bias_x = -0.08, prl_bias_y=0.1, prl_z=0.6, prl_tol=0.1, delay: float = 4.0, fall_time=1) -> None:
        telem = self.get_telemetry(frame_id="aruco_map")
        self.logger.info("Pre-landing")
        self.set_led(effect='blink', r=255, g=255, b=255)
        if prl_aruco == None:
            self.navigate_wait(x=telem.x, y=telem.y, z=z, speed=prl_speed, frame_id="aruco_map", tolerance=prl_tol)
        else:
            self.navigate_wait(x=prl_bias_x, y=prl_bias_y, z=prl_z, speed=prl_speed, frame_id=prl_aruco, tolerance=prl_tol)
        self.wait(2.0)
        self.logger.info("Landing")
        self.set_led(effect='blink', r=255, g=165, b=0)
        telem = self.get_telemetry(frame_id="base_link")
        self.navigate(x=telem.x, y=telem.y, z=-1, speed=1, frame_id="base_link")
        self.wait(fall_time)
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
        elif self.drone_name == "drone8": # вроде ок
            self.takeoff(z=1.5, delay=4, time_spam=3.5, time_warm=2, time_up=1.5)
            self.wait(3)
            self.navigate_wait(x=-0.1, y=0.1, z=1.1, yaw=math.pi, speed=0.3, frame_id="aruco_86", tolerance=0.07)
            self.wait(3)
            self.land(prl_aruco = "aruco_86", prl_bias_x = -0.08, prl_bias_y=0.1, prl_z=0.6, prl_speed=0.3, prl_tol=0.07, fall_time=1.5)
        elif self.drone_name == "drone12":
            self.takeoff(z=1.5, delay=0.5, time_spam=3, time_warm=2, time_up=0.5)
        else:
            # Default fallback if drone name not recognized
            self.logger.warning(f"Unknown drone name: {self.drone_name}, using default aruco_81")
            self.navigate_wait(x=0, y=0, z=z, yaw=math.pi, speed=0.5, frame_id="aruco_81", tolerance=0.2)
        # self.navigate_wait(x=0.5625, y=0.5625, z=z, speed=0.5, frame_id="aruco_map", tolerance=0.2)
        # self.wait(2)
        # self.navigate_wait(x=-0.08, y=0.1, z=1.1, yaw=math.pi, speed=0.3, frame_id="aruco_86", tolerance=0.1)
        # self.wait(2)
        # self.land()
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
