def convert_to_json_coords(user_x, user_y):

    json_x = -0.00662879 * user_x + 2.127841
    json_y = -0.00559701 * user_y + 1.816231
    
    return {
        "x": round(json_x, 4),
        "y": round(json_y, 4)
    }

def get_converted_coords(x, y):
    return convert_to_json_coords(x, y)
