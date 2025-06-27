import asyncio
import json
import sys
import copy
import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

from .helpers import *
from .constants import *
from .positions import get_positions

from .alg import AlgorithmicMoveGenerator
from .gemini import GeminiMoveGenerator

class GameState:
    def __init__(self, wolf_positions_alg, sheep_position_alg):
        self.board = [[EMPTY for _ in range(8)] for _ in range(8)]
        self.current_player = SHEEP

        for drone_id, alg_pos in wolf_positions_alg.items():
            pos = from_algebraic(alg_pos)
            if pos:
                r, c = pos
                self.board[r][c] = drone_id

        sheep_pos_coords = from_algebraic(sheep_position_alg)
        if sheep_pos_coords:
            r, c = sheep_pos_coords
            self.board[r][c] = SHEEP

    def is_valid_pos(self, r, c):
        return 0 <= r < 8 and 0 <= c < 8

    def get_sheep_moves(self):
        moves = []
        for r in range(8):
            for c in range(8):
                if self.board[r][c] == SHEEP:
                    for dr in [-1, 1]:
                        for dc in [-1, 1]:
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
        drone_positions = {}
        sheep_position = None
        wolf_ids_cycle = [151, 43, 15, 33]
        wolf_idx = 0
        for r in range(8):
            for c in range(8):
                if self.board[r][c] in DRONE_IDS:
                    drone_positions[wolf_ids_cycle[wolf_idx]] = to_algebraic((r, c))
                    wolf_idx = (wolf_idx + 1) % len(wolf_ids_cycle)
                elif self.board[r][c] == SHEEP:
                    sheep_position = to_algebraic((r,c))
        
        state_parts = [f"{id}:{pos}" for id, pos in drone_positions.items()]
        if sheep_position:
            state_parts.append(f"88:{sheep_position}")
        return ";".join(state_parts) + ";"


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


app = FastAPI()

CELL_TO_COORDS = {}
try:
    with open('aruco_map.json', 'r') as f:
        ARUCO_MAP_DATA = json.load(f)
    CELL_TO_COORDS = {item['cell']: [item['x'], item['y'], item['z']] for item in ARUCO_MAP_DATA if 'cell' in item}
except (FileNotFoundError, json.JSONDecodeError) as e:
    print(f"Warning: Could not load or parse aruco_map.json: {e}")

game_state_api = { 
    "status": "stop", 
    "drone": None, 
    "to": None,
    "moveGenerator": None,
    "state": None,
    "sheepPos": None,
}

class GameStateResponse(BaseModel):
    status: str
    drone: int | None = None
    to: list[float] | str | None = None
    sheepPos: str | None = None

@app.get("/game-state", response_model=GameStateResponse)
async def get_game_state():
    return transform_game_state(game_state_api)

@app.get("/positions")
async def read_positions():
    # URL видеопотока, можно вынести в конфигурацию
    stream_url = 'http://192.168.2.59:8080/stream?topic=/main_camera/image_raw'
    positions = get_positions(stream_url)
    if "error" in positions:
        "drone": None, 
        "to": None,
        "sheepPos": None,
    })
    return transform_game_state(game_state_api)

@app.get("/start", response_model=GameStateResponse)
async def start_game():
    if(game_state_api["status"] != 'active'):
        game_state_api["status"] = 'active'

        move_result = await game_state_api["moveGenerator"].get_next_move(game_state_api["state"].board)
        new_board_state = move_result["new_board"]
        drone_id = move_result["drone"]
        destination = move_result["to"]
        sheepPos = move_result["sheepPos"]

        if new_board_state:
            game_state_api["state"].board = new_board_state
            game_state_api["state"].current_player *= -1
            game_state_api["drone"] = drone_id
            game_state_api["to"] = destination
            game_state_api["sheepPos"] = sheepPos

            emulate_sheep()
            
        else:
            print("Algorithm could not find a move.")

    return transform_game_state(game_state_api)

async def emulate_sheep():
    if game_state_api["status"] != 'active':
        raise HTTPException(
            status_code=400, 
            detail="Game is not active. Please start the game first."
        )

    move_result = await game_state_api["moveGenerator"].get_sheep_move(game_state_api["state"].board)
    
    if move_result and move_result["new_board"]:
        new_board_state = move_result["new_board"]
        from_pos = move_result["from"]
        to_pos = move_result["to"]

        game_state_api["state"].board = new_board_state
        game_state_api["state"].current_player *= -1

        game_state_api["drone"] = None
        game_state_api["to"] = None
        
        print(f"Sheep moved from {from_pos} to {to_pos}")

    else:
        print("Sheep has no moves. Wolves win!")
        game_state_api["status"] = "stop"
        game_state_api["drone"] = None
        game_state_api["to"] = "WOLVES_WIN"

    return transform_game_state(game_state_api)

async def debug_game_loop(mode: str):
    print(f"--- Starting Interactive Debug Game (Mode: {mode}) ---")

    game = GameState(INITIAL_DRONE_POSITIONS, INITIAL_SHEEP_POSITION)
    
    initial_drone_positions_alg = INITIAL_DRONE_POSITIONS
    initial_sheep_position_alg = INITIAL_SHEEP_POSITION
    
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
            
        else:
            print(f"--- {mode.upper()}'s turn (Wolves) ---")
        
            new_board_state = await move_generator.get_next_move(game.board)
            if new_board_state:
                game.board = new_board_state
                game.current_player *= -1
            else:
                print("Algorithm could not find a move.")

def init_game_state():
    mode = 'alg'
    game_state_api["state"] = GameState(INITIAL_DRONE_POSITIONS, INITIAL_SHEEP_POSITION)
    
    initial_drone_positions_alg = {}
    initial_sheep_position_alg = {}
    
    game_state_api["moveGenerator"] = GeminiMoveGenerator(
        mode,
        api_key=API_KEY,
        drone_ids=DRONE_IDS,
        sheep_id=SHEEP_ID,
        initial_drone_positions=initial_drone_positions_alg,
        initial_sheep_position=initial_sheep_position_alg
    ) if mode  == 'ai' else AlgorithmicMoveGenerator(game_state_api["state"])

def transform_game_state(gameStateRaw):
    print(gameStateRaw)
    return {
        "status": gameStateRaw["status"], 
        "drone": gameStateRaw["drone"], 
        "to": gameStateRaw["to"],
        "sheepPos": gameStateRaw["sheepPos"]
    }

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "debug":
        mode = "ai"
        if len(sys.argv) > 2 and sys.argv[2] in ["ai", "alg"]:
            mode = sys.argv[2]
        asyncio.run(debug_game_loop(mode))


init_game_state()
