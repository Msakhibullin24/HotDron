import json
import sys
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from starlette.responses import FileResponse
from pydantic import BaseModel

from .helpers import *
from .constants import *
from .positions import *
from .game_state import GameState, init_game_state, set_sheep_pos
from starlette.responses import FileResponse 

app = FastAPI()
origins = [
    "http://localhost",
    "http://localhost:8080", # Если вы будете запускать HTML через какой-нибудь live server
    "http://127.0.0.1",
    "http://127.0.0.1:8000",
    "null"  # <-- ЭТО САМОЕ ВАЖНОЕ ДЛЯ ЛОКАЛЬНЫХ ФАЙЛОВ (file://)
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"], # Разрешить все методы (GET, POST и т.д.)
    allow_headers=["*"], # Разрешить все заголовки
)

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
    "drone_id": None,
    "to": None,
    "to_aruco": None,
    "moveGenerator": None,
    "state": None,
    "sheepPos": None,
}

def set_default_game_state_api():
    global game_state_api
    game_state_api = init_game_state({ 
        "status": "stop", 
        "drone": None, 
        "drone_id": None,
        "to": None,
        "to_aruco": None,
        "moveGenerator": None,
        "state": None,
        "sheepPos": None,
    })

set_default_game_state_api()


class GameStateResponse(BaseModel):
    status: str
    drone_id: int | None = None
    drone: str | None
    to: list[float] | None = None
    to_aruco: str | None = None
    sheepPos: str | None = None
    board: list[list[int]] | None = None

@app.get("/")
async def read_index():
    return FileResponse('./ui/index.html')

@app.get("/game-state", response_model=GameStateResponse)
async def get_game_state(board: bool = False):
    return transform_game_state(game_state_api, include_board=board)

@app.get("/positions")
async def read_positions():
    positions = get_positions()
    if "error" in positions or not positions:
        raise HTTPException(status_code=404, detail="No drone positions found")

    return positions

@app.get("/drone-positions")
async def read_positions():
    positions = get_drone_positions()
    if "error" in positions or not positions:
        raise HTTPException(status_code=404, detail="No drone positions found")

    return positions

@app.get("/start", response_model=GameStateResponse)
async def start_game():
    global game_state_api
    if game_state_api["status"] == 'active':
        return transform_game_state(game_state_api)

    game_state_api["status"] = 'active'

    # Get current sheep position and update state
    new_sheep_pos = get_sheep_position()
    if not new_sheep_pos:
        game_state_api["status"] = 'stop'  # Revert status
        raise HTTPException(status_code=404, detail="Sheep position not found")

    sheep_cell = get_cell_from_coords(new_sheep_pos) if CONNECT_TO_CAM_SHEEP else INITIAL_SHEEP_POSITION
    if not sheep_cell:
        game_state_api["status"] = 'stop'  # Revert status
        raise HTTPException(status_code=404, detail=f"Could not determine cell for sheep at coords {new_sheep_pos}")

    game_state_api = set_sheep_pos(game_state_api, sheep_cell)

    try:
        move_result = await game_state_api["moveGenerator"].get_next_move(game_state_api["state"].board)
        new_board_state = move_result["new_board"]
        drone_id = move_result["drone"]
        destination = move_result["to"]
        to_aruco = move_result["to_aruco"]
        sheepPos = move_result["sheepPos"]

        if new_board_state:
            game_state_api["state"].board = new_board_state
            game_state_api["state"].current_player *= -1
            game_state_api["drone"] = drone_id
            game_state_api["to"] = destination
            game_state_api["to_aruco"] = to_aruco
            game_state_api["sheepPos"] = sheepPos

            emulate_sheep()
            
        else:
            print("Algorithm could not find a move.")
            raise HTTPException(status_code=404, detail="Algorithm could not find a move")
    finally:
        return transform_game_state(game_state_api)


@app.get("/stop", response_model=GameStateResponse)
async def stop_game():
    if(game_state_api["status"] == 'stop'):
        return transform_game_state(game_state_api)

    game_state_api.update({
        "status": "stop", 
        "drone": None, 
        "drone_id": None,
        "to": None,
        "to_aruco": None,
        "sheepPos": None,
    })
    return transform_game_state(game_state_api)

@app.get("/circle-sheep")
async def circle_sheep():
    new_sheep_pos = get_sheep_position()
    if not new_sheep_pos:
        raise HTTPException(status_code=404, detail="Sheep position not found")

    print(f'pos {new_sheep_pos}')
    sheep_cell = get_cell_from_coords(new_sheep_pos)
    if not sheep_cell:
        raise HTTPException(status_code=404, detail=f"Could not determine cell for sheep at coords {new_sheep_pos}")
        
    print(f'cell {sheep_cell}')
    return get_block_sheep_positions(sheep_cell)

@app.get("/reset")
async def reset_game():
    set_default_game_state_api()
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

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "debug":
        mode = "ai"
        if len(sys.argv) > 2 and sys.argv[2] in ["ai", "alg"]:
            mode = sys.argv[2]
        # asyncio.run(debug_game_loop(mode))


game_state_api = init_game_state(game_state_api)
