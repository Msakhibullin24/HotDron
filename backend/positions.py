import cv2
import requests
import numpy as np
from .popukai import get_converted_coords

# Настройка ArUco словаря и параметров
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

# Получение кадра из MJPEG-потока
def get_frame_from_stream(url):
    try:
        stream = requests.get(url, stream=True, timeout=2)
        bytes_data = b''
        for chunk in stream.iter_content(chunk_size=1024):
            bytes_data += chunk
            a = bytes_data.find(b'\xff\xd8')  # Start of JPEG
            b = bytes_data.find(b'\xff\xd9')  # End of JPEG
            if a != -1 and b != -1:
                jpg = bytes_data[a:b+2]
                bytes_data = bytes_data[b+2:]
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                return frame
    except requests.exceptions.RequestException as e:
        print(f"Error getting frame from stream: {e}")
        return None
    return None

def get_positions(stream_url: str):
    frame = get_frame_from_stream(stream_url)
    if frame is None:
        return {"error": "Could not get frame from stream"}

    # Параметры для коррекции перспективы (взяты из detector.py)
    pts1 = np.float32([[147, 59], [443, 33], [457, 338], [166, 347]])
    width, height = 300, 350
    pts2 = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    
    warped_frame = cv2.warpPerspective(frame, matrix, (width, height))
    gray = cv2.cvtColor(warped_frame, cv2.COLOR_BGR2GRAY)
    
    corners, ids, _ = DETECTOR.detectMarkers(gray)
    
    positions = {}
    if ids is not None:
        for i, marker_id_array in enumerate(ids):
            marker_id = int(marker_id_array[0])
            
            marker_corners = corners[i].reshape((4, 2))
            cX = int(np.mean(marker_corners[:, 0]))
            cY = int(np.mean(marker_corners[:, 1]))
            
            positions[marker_id] = get_converted_coords(cX, cY)
            
    return positions

if __name__ == '__main__':
    # Пример использования
    stream_url = 'http://192.168.2.59:8080/stream?topic=/main_camera/image_raw'
    positions = get_positions(stream_url)
    print(positions)
