import math
import copy
from .helpers import to_algebraic, from_algebraic
from .constants import SHEEP, EMPTY, DRONE_IDS
from .game_state import GameState, minimax

WOLF = 1

class AlgorithmicMoveGenerator:
    def __init__(self, initial_game_state):
        self.game_state = initial_game_state
        self.depth = 9

    def _convert_to_internal_board(self, main_board):
        wolf_positions = []
        sheep_position = None

        for r in range(8):
            for c in range(8):
                cell = main_board[r][c]
                if cell in DRONE_IDS:
                    wolf_positions.append((r, c))
                elif cell == SHEEP:
                    sheep_position = (r, c)
        
        if not sheep_position:
            raise ValueError("Sheep not found on the board")

        return GameState(wolf_positions, sheep_position)

    async def get_next_move(self, board_state):
        internal_state = self._convert_to_internal_board(board_state)
        
        _, best_move = minimax(internal_state, self.depth, -math.inf, math.inf, True)

        if not best_move:
            return {"new_board": None, "drone": None, "to": None, "sheepPos": None}

        (from_r, from_c), (to_r, to_c) = best_move
        
        new_board = copy.deepcopy(board_state)
        drone_id = new_board[from_r][from_c]
        new_board[to_r][to_c] = drone_id
        new_board[from_r][from_c] = EMPTY
        
        destination_alg = to_algebraic((to_r, to_c))

        sheep_pos = None
        for r in range(8):
            for c in range(8):
                if internal_state.board[r][c] == SHEEP:
                    sheep_pos = (r, c)
                    break
            if sheep_pos:
                break
        
        sheep_pos_alg = to_algebraic(sheep_pos) if sheep_pos else None


        return {
            "new_board": new_board,
            "drone": drone_id,
            "to": destination_alg,
            "sheepPos": sheep_pos_alg
        }

    async def get_sheep_move(self, board_state):
        internal_state = self._convert_to_internal_board(board_state)
        
        _, best_move = minimax(internal_state, self.depth, -math.inf, math.inf, False)

        if not best_move:
            return {"new_board": None, "from": None, "to": None}

        (from_r, from_c), (to_r, to_c) = best_move
        
        new_board = copy.deepcopy(board_state)
        new_board[to_r][to_c] = SHEEP
        new_board[from_r][from_c] = EMPTY
        
        return {
            "new_board": new_board,
            "from": to_algebraic((from_r, from_c)),
            "to": to_algebraic((to_r, to_c))
        }

    def evaluate(self):
        internal_state = self._convert_to_internal_board(self.game_state.board)
        return internal_state.evaluate()
