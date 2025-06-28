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

def print_board(board):
    print("\n  a b c d e f g h")
    print(" +-----------------+")
    for i, row in enumerate(board):
        print(f"{8 - i}|", end="")
        for j, cell in enumerate(row):
            char = " "
            if cell in DRONE_IDS: char = "W"
            elif cell == SHEEP: char = "S"
            elif (i + j) % 2 == 0: char = "."
            print(f" {char}", end="")
        print(f" |{8 - i}")
    print(" +-----------------+")
    print("  a b c d e f g h\n")

def transform_game_state(gameStateRaw):
    return {
        "status": gameStateRaw["status"], 
        "drone": gameStateRaw["drone"], 
        "to": gameStateRaw["to"],
        "sheepPos": gameStateRaw["sheepPos"]
    }
