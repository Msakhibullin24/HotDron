import json
import sys
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.concurrency import run_in_threadpool
from starlette.responses import FileResponse
from pydantic import BaseModel

import logging
from .log_handler import setup_logging

from .helpers import *
from .constants import *
from .positions import *
from .game_state import GameState, init_game_state, set_sheep_pos
from starlette.responses import FileResponse 

logger = setup_logging(drone_name="main_api")

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
    logger.warning(f"Could not load or parse aruco_map.json: {e}")

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
    logger.info('Get game-state start...')
    return transform_game_state(game_state_api, include_board=board)

@app.get("/positions")
async def read_positions():
    logger.info('Get positions start...')
    positions = get_positions()
    if "error" in positions or not positions:
        logger.error("No drone positions found")
        raise HTTPException(status_code=404, detail="No drone positions found")

    logger.info('Got positions successfully')
    return positions

@app.get("/drone-positions")
async def read_positions():
    logger.info('Get drone positions start...')
    positions = get_drone_positions()
    if "error" in positions or not positions:
        logger.error("No drone positions found")
        raise HTTPException(status_code=404, detail="No drone positions found")

    logger.info('Got drone positions successfully')
    return positions

@app.get("/start", response_model=GameStateResponse)
async def start_game():
    logger.info('Drone move started...')
    global game_state_api
    if game_state_api["status"] == 'active':
        logger.warning("Start endpoint called, but game is already active.")
        return transform_game_state(game_state_api)

    logger.info("Start controller called")
    game_state_api["status"] = 'active'

    # Get current sheep position and update state
    new_sheep_pos = get_sheep_position()
    if not new_sheep_pos and CONNECT_TO_CAM_SHEEP:
        game_state_api["status"] = 'stop'  # Revert status
        logger.error("Could not get sheep position from camera.")
        raise HTTPException(status_code=404, detail="Sheep position not found")

    sheep_cell = get_cell_from_coords(new_sheep_pos) if CONNECT_TO_CAM_SHEEP else INITIAL_SHEEP_POSITION
    print(f'sheep cell {sheep_cell}')
    if not sheep_cell:
        game_state_api["status"] = 'stop'  # Revert status
        logger.error(f"Could not determine cell for sheep at coords {new_sheep_pos}")
        raise HTTPException(status_code=404, detail=f"Could not determine cell for sheep at coords {new_sheep_pos}")

    game_state_api = set_sheep_pos(game_state_api, sheep_cell)
    logger.info(f"Sheep position set to cell: {sheep_cell}")

    try:
        move_result = await run_in_threadpool(
            game_state_api["moveGenerator"].get_next_move, 
            game_state_api["state"].board
        )
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

            logger.info(f"Move generated. Drone: {drone_id}, Destination: {destination} (Aruco: {to_aruco})")

        else:
            logger.warning("Algorithm could not find a move.")
            raise HTTPException(status_code=404, detail="Algorithm could not find a move")
    finally:
        return transform_game_state(game_state_api)


@app.get("/stop", response_model=GameStateResponse)
async def stop_game():
    logger.info('Stop game start...')
    if(game_state_api["status"] == 'stop'):
        logger.info("Stop endpoint called, but game is already stopped.")
        return transform_game_state(game_state_api)

    logger.info("Stopping game.")
    game_state_api.update({
        "status": "stop", 
        "drone": None, 
        "drone_id": None,
        "to": None,
        "to_aruco": None,
        "sheepPos": None,
    })

    logger.info('Updated game state api, stopped successfully')
    return transform_game_state(game_state_api)

@app.get("/circle-sheep")
async def circle_sheep():
    new_sheep_pos = get_sheep_position()
    if not new_sheep_pos:
        logger.error("Circle-sheep failed: Sheep position not found")
        raise HTTPException(status_code=404, detail="Sheep position not found")

    logger.info(f'Circling sheep at pos {new_sheep_pos}')
    sheep_cell = get_cell_from_coords(new_sheep_pos)
    if not sheep_cell:
        logger.error(f"Circle-sheep failed: Could not determine cell for sheep at coords {new_sheep_pos}")
        raise HTTPException(status_code=404, detail=f"Could not determine cell for sheep at coords {new_sheep_pos}")
        
    logger.info(f'Circling sheep at cell {sheep_cell}')
    return get_block_sheep_positions(sheep_cell)

@app.get("/reset")
async def reset_game():
    logger.info("Resetting game state via API.")
    set_default_game_state_api()
    return transform_game_state(game_state_api)

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "debug":
        mode = "ai"
        if len(sys.argv) > 2 and sys.argv[2] in ["ai", "alg"]:
            mode = sys.argv[2]
        # asyncio.run(debug_game_loop(mode))


logger.info('Game state initialization')
game_state_api = init_game_state(game_state_api)
