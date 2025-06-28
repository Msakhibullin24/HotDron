import math
import copy
from .helpers import to_algebraic, from_algebraic
from .constants import SHEEP, EMPTY, DRONE_IDS

WOLF = 1

def is_valid_pos(r, c):
    return 0 <= r < 8 and 0 <= c < 8


def get_sheep_moves(board):
    """Возвращает все возможные ходы для овцы."""
    moves = []
    for r in range(8):
        for c in range(8):
            if board[r][c] == SHEEP:
                for dr in [-1, 1]:
                    for dc in [-1, 1]:
                        nr, nc = r + dr, c + dc
                        if is_valid_pos(nr, nc) and board[nr][nc] == EMPTY:
                            moves.append(((r, c), (nr, nc)))
                return moves
    return moves

def get_wolf_moves(board):
    """Gets all possible moves for all wolves."""
    moves = []
    for r in range(8):
        for c in range(8):
            if board[r][c] != SHEEP and board[r][c] != EMPTY:
                # Wolves move forward diagonally
                for dr, dc in [(1, -1), (1, 1)]:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < 8 and 0 <= nc < 8 and board[nr][nc] == EMPTY:
                        moves.append(((r, c), (nr, nc)))
    return moves

class AlgorithmicMoveGenerator:
    def __init__(self, initial_game_state):
        self.game_state = initial_game_state
        self.depth = 9

    async def get_next_move(self, current_board_state):
        possible_moves = get_wolf_moves(current_board_state)

        sheep_pos_raw = None
        for r in range(8):
            for c in range(8):
                if current_board_state[r][c] == SHEEP:
                    sheep_pos_raw = (r, c)
                    break
            if sheep_pos_raw:
                break
                
        print(sheep_pos_raw)
        sheepPos = to_algebraic(sheep_pos_raw)
        print(sheepPos)

        if not possible_moves:
            return None  # No moves available

        # Simple strategy: just pick the first move
        best_move = possible_moves[0]
        
        # Create a new state with this move
        new_board = copy.deepcopy(current_board_state)
        (from_r, from_c), (to_r, to_c) = best_move
        drone_id = current_board_state[from_r][from_c]

        new_board[to_r][to_c] = drone_id
        new_board[from_r][from_c] = EMPTY
        
        self.game_state.board = new_board

        destination_algebraic = to_algebraic((to_r, to_c)) # TODO: (to_r, to_c) преобразовать в [x, y, z] через CV

        return {
            "new_board": new_board,
            "drone": drone_id,
            "to": destination_algebraic,
            "sheepPos": sheepPos
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

def minimax(state, depth, alpha, beta, maximizing_player):
    if depth == 0 or state.evaluate() in [1000, -1000]:
        return state.evaluate(), None

    if maximizing_player:
        max_eval = -math.inf
        best_move = None
        for move in state.get_wolf_moves():
            new_state = state.make_move(move)
            evaluation, _ = minimax(new_state, depth - 1, alpha, beta, False)
            if evaluation > max_eval:
                max_eval = evaluation
                best_move = move
            alpha = max(alpha, evaluation)
            if beta <= alpha:
                break
        return max_eval, best_move
    else:
        min_eval = math.inf
        best_move = None
        for move in state.get_sheep_moves():
            new_state = state.make_move(move)
            evaluation, _ = minimax(new_state, depth - 1, alpha, beta, True)
            if evaluation < min_eval:
                min_eval = evaluation
                best_move = move
            beta = min(beta, evaluation)
            if beta <= alpha:
                break
        return min_eval, best_move
