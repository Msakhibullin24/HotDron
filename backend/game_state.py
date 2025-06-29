from .helpers import *
from .constants import *

from .alg import AlgorithmicMoveGenerator
from .gemini import GeminiMoveGenerator
from .positions import get_sheep_position, from_algebraic, get_drone_positions, get_cell_from_coords

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


def init_game_state(game_state_api):
    mode = 'alg'
    sheep_pos_coords = get_sheep_position()
    sheep_cell = (get_cell_from_coords(sheep_pos_coords) or INITIAL_SHEEP_POSITION) if CONNECT_TO_CAM else INITIAL_SHEEP_POSITION

    drone_pos_coords = get_drone_positions()
    
    transformed_drone_pos = {}
    if drone_pos_coords:
        for drone_id, coords in drone_pos_coords.items():
            cell = get_cell_from_coords(coords)
            if cell:
                transformed_drone_pos[drone_id] = cell

    final_drone_positions = (transformed_drone_pos if transformed_drone_pos else INITIAL_DRONE_POSITIONS) if CONNECT_TO_CAM else INITIAL_DRONE_POSITIONS

    game_state_api["state"] = GameState(final_drone_positions, sheep_cell)
    
    initial_drone_positions_alg = final_drone_positions
    initial_sheep_position_alg = {}
    
    game_state_api["moveGenerator"] = GeminiMoveGenerator(
        mode,
        api_key=API_KEY,
        drone_ids=DRONE_IDS,
        sheep_id=SHEEP_ID,
        initial_drone_positions=initial_drone_positions_alg,
        initial_sheep_position=initial_sheep_position_alg
    ) if mode  == 'ai' else AlgorithmicMoveGenerator(game_state_api["state"])

    return game_state_api

def set_sheep_pos(game_state_api, new_sheep_pos):
    """
    Updates the sheep's position on the board and in the game state.
    """
    board = game_state_api["state"].board
    
    # Find and remove the old sheep position
    for r in range(8):
        for c in range(8):
            if board[r][c] == SHEEP:
                board[r][c] = EMPTY
                break
    
    # Add the new sheep position
    new_pos_coords = from_algebraic(new_sheep_pos)
    if new_pos_coords:
        r, c = new_pos_coords
        board[r][c] = SHEEP
        game_state_api["sheepPos"] = new_sheep_pos
    else:
        print(f"Error: Invalid algebraic notation for new sheep position: {new_sheep_pos}")

    return game_state_api
