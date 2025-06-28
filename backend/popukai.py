def convert_to_json_coords(pixel_x, pixel_y):
    """
    Конвертирует координаты центра маркера (pixel_x, pixel_y) из выпрямленного 
    изображения (warped frame) в реальные мировые координаты для дронов.

    Эта версия учитывает отступы (margins) шахматной доски внутри 
    выпрямленного изображения, что значительно повышает точность.
    """
    
    # --- Параметры преобразования ---
    
    # Размеры выпрямленного изображения
    WARPED_WIDTH = 600.0
    WARPED_HEIGHT = 700.0
    
    # РАССЧИТАННЫЕ ОТСТУПЫ (в пикселях)
    # Это расстояние от края изображения до края игрового поля
    PIXEL_MARGIN_X = 28.0
    PIXEL_MARGIN_Y = 37.0

    # Реальные пиксельные координаты краев доски
    board_pixel_min_x = PIXEL_MARGIN_X
    board_pixel_max_x = WARPED_WIDTH - PIXEL_MARGIN_X
    
    board_pixel_min_y = PIXEL_MARGIN_Y
    board_pixel_max_y = WARPED_HEIGHT - PIXEL_MARGIN_Y

    # Реальная ширина и высота доски в пикселях
    board_pixel_width = board_pixel_max_x - board_pixel_min_x
    board_pixel_height = board_pixel_max_y - board_pixel_min_y
    
    # Диапазон мировых координат (из вашего JSON с верными данными)
    WORLD_X_MIN = -1.3125
    WORLD_X_MAX = 1.3125
    WORLD_Y_MIN = -1.3125
    WORLD_Y_MAX = 1.3125

    # --- Вычисления ---

    # 1. Нормализуем координату: переводим ее в диапазон [0, 1] относительно РЕАЛЬНЫХ границ доски
    #    (pixel_x - board_pixel_min_x) - получаем позицию относительно левого края доски
    #    Делим на board_pixel_width - получаем долю от 0 до 1
    norm_x = (pixel_x - board_pixel_min_x) / board_pixel_width
    norm_y = (pixel_y - board_pixel_min_y) / board_pixel_height

    # 2. Масштабируем нормализованную координату до мировых размеров
    #    Здесь мы снова учитываем инверсию оси Y
    world_x = -(norm_x * (WORLD_X_MAX - WORLD_X_MIN) + WORLD_X_MIN) + 0.15
    world_y = -((1.0 - norm_y) * (WORLD_Y_MAX - WORLD_Y_MIN) + WORLD_Y_MIN) + 0.18
    
    return {
        "x": round(world_x, 4),
        "y": round(world_y, 4)
    }



def get_converted_coords(x, y):
    return convert_to_json_coords(x, y)
