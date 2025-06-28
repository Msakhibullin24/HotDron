import numpy as np
import cv2
import requests
from popukai import get_converted_coords


ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

# Получение кадров из MJPEG-потока
def get_latest_frame(url):
    stream = requests.get(url, stream=True)
    bytes_data = b''
    for chunk in stream.iter_content(chunk_size=1024):
        bytes_data += chunk
        a = bytes_data.find(b'\xff\xd8')  # Start of JPEG
        b = bytes_data.find(b'\xff\xd9')  # End of JPEG

        while a != -1 and b != -1 and b > a:
            jpg = bytes_data[a:b+2]
            bytes_data = bytes_data[b+2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            yield frame
            a = bytes_data.find(b'\xff\xd8')
            b = bytes_data.find(b'\xff\xd9')

if __name__ == '__main__':
    stream_url = 'http://192.168.2.59:8080/stream?topic=/main_camera/image_raw'  # замените на свой

    WOLVES_IDS = {151, 152, 153, 154}
    SHEEP_ID = 47


    pts1 = np.float32([[117, 29], [473, 3], [487, 368], [136, 377]])
    
    width, height = 600, 700
    pts2 = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    
    matrix = cv2.getPerspectiveTransform(pts1, pts2)

    frame_gen = get_latest_frame(stream_url)

    for frame in frame_gen:
        if frame is None:
            continue

        warped_frame = cv2.warpPerspective(frame, matrix, (width, height))

        gray = cv2.cvtColor(warped_frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = DETECTOR.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(warped_frame, corners, ids)
            for i, marker_id_array in enumerate(ids):
                marker_id = marker_id_array[0]
                
                marker_corners = corners[i].reshape((4, 2))
                cX = int(np.mean(marker_corners[:, 0]))
                cY = int(np.mean(marker_corners[:, 1]))

                class_name = None
                if marker_id in WOLVES_IDS:
                    class_name = "Волк"
                elif marker_id == SHEEP_ID:
                    class_name = "Овца"
                
                if class_name:
                    converted_coords = get_converted_coords(cX, cY)
                    print(f"Класс: {class_name}, ID: {marker_id}, Координаты: x={cX}, y={cY}, Конвертированные: {converted_coords}")

        cv2.imshow('ArUco Detection', warped_frame)

        if cv2.waitKey(1) == 27:  # Нажмите ESC для выхода
            break

    cv2.destroyAllWindows()
    