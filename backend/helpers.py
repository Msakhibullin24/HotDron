def to_algebraic(pos):
    r, c = pos
    return f"{chr(ord('a') + c)}{8 - r}"

def from_algebraic(cell_str):
    if not cell_str or len(cell_str) != 2:
        return None
    try:
        c = ord(cell_str[0].lower()) - ord('a')
        r = 8 - int(cell_str[1])
        if not (0 <= r < 8 and 0 <= c < 8):
            return None
        return r, c
    except (ValueError, IndexError):
        return None
