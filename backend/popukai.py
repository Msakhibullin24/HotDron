def convert_to_json_coords(user_x, user_y):

    json_x = -0.0105228 * user_x + 1.606
    json_y = -0.00109677 * user_y + 1.606
    
    return {
        "x": round(json_x, 4),
        "y": round(json_y, 4)
    }

def get_converted_coords(x, y):
    return convert_to_json_coords(x, y)
