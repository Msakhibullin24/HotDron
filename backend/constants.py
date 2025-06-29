SHEEP = -1
EMPTY = 0

stream_url = 'http://192.168.2.59:8080/stream?topic=/main_camera/image_raw'

SHEEP_ID = 202
DRONE_IDS = [153, 152, 154 ,151]
INITIAL_DRONE_POSITIONS = {
    153: 'H8',
    152: 'F8',
    154: 'D8',
    151: 'B8'
}
DRONE_ID_TO_DRONE_NAMES = {
    153: 'drone12',
    152: 'drone8',
    154: 'drone5',
    151: 'drone10'
}

INITIAL_SHEEP_POSITION = 'A1'
API_KEY=''

CONNECT_TO_CAM_SHEEP = False
CONNECT_TO_CAM_WOLF = False