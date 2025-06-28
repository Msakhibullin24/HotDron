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
from .game_state import GameState, init_game_state

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
    positions = get_positions(stream_url)
    if "error" in positions:
        return ({
            "drone": None, 
            "to": None,
            "sheepPos": None,
        })

    return positions

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


@app.get("/stop", response_model=GameStateResponse)
async def stop_game():
    if(game_state_api["status"] == 'stop'):
        return transform_game_state(game_state_api)

    game_state_api.update({
        "status": "stop", 
        "drone": None, 
        "to": None,
        "sheepPos": None,
    })
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
        asyncio.run(debug_game_loop(mode))


game_state_api = init_game_state(game_state_api)
