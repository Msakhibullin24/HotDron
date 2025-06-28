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

        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        corners, ids, _ = detector.detectMarkers(gray)

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
    