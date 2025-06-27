def convert_to_json_coords(user_x, user_y):

    
    json_x = -0.010869565 * user_x + 1.60597826
    json_y = 1.3125
    
    return {
        "x": round(json_x, 4), # Округляем для удобства
        "y": round(json_y, 4)
    }

def get_converted_coords(x, y):
    return convert_to_json_coords(x, y)
