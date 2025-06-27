import asyncio
import json
import sys
import copy
import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

# --- Game Logic (from gamelogic.py) ---
WOLF = 1
SHEEP = -1
EMPTY = 0


DRONE_IDS = [151, 43, 15, 33]
SHEEP_ID = 88
INITIAL_DRONE_POSITIONS = {
    151: 'A2',
    43: 'A4',
    15: 'A6',
    33: 'A8'
}
INITIAL_SHEEP_POSITION = 'H1'
API_KEY=''

initial_game_state = "151:A2;43:A4;15:A6;33:A8;88:H1;"


def to_algebraic(pos):
    """Converts (row, col) to algebraic notation like 'A1'."""
    r, c = pos
    return f"{chr(ord('a') + c)}{8 - r}"

def from_algebraic(cell_str):
    """Converts algebraic notation like 'A1' to (row, col)."""
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

class GameState:
    """Manages the state of the game board."""
    def __init__(self, wolf_positions, sheep_position):
        self.board = [[EMPTY for _ in range(8)] for _ in range(8)]
        for r, c in wolf_positions:
            self.board[r][c] = WOLF
        self.board[sheep_position[0]][sheep_position[1]] = SHEEP
        self.current_player = SHEEP

    def is_valid_pos(self, r, c):
        return 0 <= r < 8 and 0 <= c < 8

    def get_sheep_moves(self):
        moves = []
        for r in range(8):
            for c in range(8):
                if self.board[r][c] == SHEEP:
                    for dr, dc in [(-1, -1), (-1, 1)]:
                        nr, nc = r + dr, c + dc
                        if self.is_valid_pos(nr, nc) and self.board[nr][nc] == EMPTY:
                            moves.append(((r, c), (nr, nc)))
                    return moves
        return moves

    def make_move(self, move):
        new_state = copy.deepcopy(self)
        (from_r, from_c), (to_r, to_c) = move
        piece = new_state.board[from_r][from_c]
        new_state.board[to_r][to_c] = piece
        new_state.board[from_r][from_c] = EMPTY
        new_state.current_player *= -1
        return new_state

    def get_board_state_str(self):
        """Generates the board state string for the AI."""
        drone_positions = {}
        sheep_position = None
        wolf_ids_cycle = [151, 43, 15, 33]
        wolf_idx = 0
        for r in range(8):
            for c in range(8):
                if self.board[r][c] == WOLF:
                    drone_positions[wolf_ids_cycle[wolf_idx]] = to_algebraic((r, c))
                    wolf_idx = (wolf_idx + 1) % len(wolf_ids_cycle)
                elif self.board[r][c] == SHEEP:
                    sheep_position = to_algebraic((r,c))
        
        state_parts = [f"{id}:{pos}" for id, pos in drone_positions.items()]
        if sheep_position:
            state_parts.append(f"88:{sheep_position}")
        return ";".join(state_parts) + ";"


def print_board(board):
    """Prints the game board to the console."""
    print("\n  a b c d e f g h")
    print(" +-----------------+")
    for i, row in enumerate(board):
        print(f"{8 - i}|", end="")
        for j, cell in enumerate(row):
            char = " "
            if cell == WOLF: char = "W"
            elif cell == SHEEP: char = "S"
            elif (i + j) % 2 == 0: char = "."
            print(f" {char}", end="")
        print(f" |{8 - i}")
    print(" +-----------------+")
    print("  a b c d e f g h\n")


# --- FastAPI Application Setup ---
app = FastAPI()

from alg import AlgorithmicMoveGenerator
from gemini import GeminiMoveGenerator

CELL_TO_COORDS = {}
try:
    with open('aruco_map.json', 'r') as f:
        ARUCO_MAP_DATA = json.load(f)
    CELL_TO_COORDS = {item['cell']: [item['x'], item['y'], item['z']] for item in ARUCO_MAP_DATA if 'cell' in item}
except (FileNotFoundError, json.JSONDecodeError) as e:
    print(f"Warning: Could not load or parse aruco_map.json: {e}")

game_state_api = { "status": "stop", "drone": None, "to": None }

class GameStateResponse(BaseModel):
    status: str
    drone: int | None = None
    to: list[float] | None = None

@app.get("/game-state", response_model=GameStateResponse)
async def get_game_state():
    return game_state_api

@app.post("/stop", response_model=GameStateResponse)
async def stop_game():
    game_state_api.update({"status": "stop", "drone": None, "to": None})
    return game_state_api

@app.post("/start", response_model=GameStateResponse)
async def start_game():
    # This endpoint is simplified and not connected to the debug loop.
    return game_state_api

# --- Interactive Debug Game Loop ---
async def debug_game_loop(mode: str):
    """Runs an interactive game session in the terminal."""
    print(f"--- Starting Interactive Debug Game (Mode: {mode}) ---")
    
    initial_wolf_pos = [(0, 1), (0, 3), (0, 5), (0, 7)]
    initial_sheep_pos = (7, 0)

    game = GameState(initial_wolf_pos, initial_sheep_pos)
    
    initial_drone_positions_alg = {DRONE_IDS[i]: to_algebraic(pos) for i, pos in enumerate(initial_wolf_pos)}
    initial_sheep_position_alg = to_algebraic(initial_sheep_pos)
    
    move_generator = GeminiMoveGenerator(
        mode,
        api_key=API_KEY,
        drone_ids=DRONE_IDS,
        sheep_id=SHEEP_ID,
        initial_drone_positions=initial_drone_positions_alg,
        initial_sheep_position=initial_sheep_position_alg
    ) if mode  == 'ai' else AlgorithmicMoveGenerator(game)

    while True:
        print_board(game.board)
        eval_score = move_generator.evaluate()

        if eval_score >= 1000:
            print("Wolves win! The sheep is blocked.")
            break
        if eval_score <= -1000:
            print("Sheep wins! You reached the other side.")
            break

        if game.current_player == SHEEP:
            print("\n--- Ваш ход (Овца) ---")
            possible_moves = game.get_sheep_moves()
            if not possible_moves:
                print("У овцы нет ходов. Волки победили!")
                break
            
            print("Возможные ходы:")
            for i, move in enumerate(possible_moves):
                print(f"{i}: из {to_algebraic(move[0])} в {to_algebraic(move[1])}")

            try:
                choice = int(input("Выберите номер хода: "))
                if 0 <= choice < len(possible_moves):
                    user_move = possible_moves[choice]
                    game = game.make_move(user_move)
                    print("\nПозиция после вашего хода:")
                    print_board(game.board)
                else:
                    print("Неверный номер хода. Попробуйте еще раз.")
                    continue
            except ValueError:
                print("Неверный ввод. Введите число.")
                continue
            
        else: # Wolves' turn
            print(f"--- {mode.upper()}'s turn (Wolves) ---")
        
            new_board_state = await move_generator.get_next_move(game.board)
            if new_board_state:
                game.board = new_board_state
                game.current_player *= -1
            else:
                print("Algorithm could not find a move.")

# --- Main Execution ---
if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "debug":
        mode = "ai"
        if len(sys.argv) > 2 and sys.argv[2] in ["ai", "alg"]:
            mode = sys.argv[2]
        asyncio.run(debug_game_loop(mode))
    else:
        print("Starting FastAPI server... (run with 'debug [ai|alg]' for interactive mode)")
        uvicorn.run("backend.main:app", host="0.0.0.0", port=8000, reload=True)
