import requests
import json

class Requests:
    def __init__(self, navigate_wait, drone_names):
        self.api_url = 'http://192.168.2.95:8000/'
        self.navigateWait = navigate_wait
        self.drone_names = drone_names

    def encircleSheep(self):
        """
        Gets the positions to encircle the sheep and navigates the drones.
        """
        try:
            response = requests.get(f"{self.api_url}/circle-sheep")
            response.raise_for_status()  # Raise an exception for bad status codes
            
            positions = response.json()
            
            if not isinstance(positions, list):
                print("Error: Invalid response format from server.")
                return

            for i, pos in enumerate(positions):
                if i < len(self.drone_names):
                    drone_name = self.drone_names[i]
                    coords = pos.get('coords')
                    if coords and len(coords) >= 2:
                        x, y = coords[0], coords[1]
                        print(f"Navigating {drone_name} to x={x}, y={y}")
                        # Assuming navigateWait is a function that takes drone_name, x, and y
                        # Since we don't have the full context of how navigateWait is called,
                        # we will call it with the available parameters.
                        # The user should adjust this if the function signature is different.
                        self.navigateWait(x=x, y=y, frame_id=drone_name)

                        self.wait(1.0)
                    else:
                        print(f"Warning: Invalid coordinates for position {i}")
                else:
                    print(f"Warning: Not enough drones for all positions. Position {i} skipped.")

        except requests.exceptions.RequestException as e:
            print(f"Error making request to /circle-sheep: {e}")
        except json.JSONDecodeError:
            print("Error: Could not decode JSON response from server.")
