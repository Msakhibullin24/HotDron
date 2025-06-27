import math
import copy
from .helpers import *
from .constants import *

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
            if board[r][c] != SHEEP or board[r][c] != EMPTY:
                # Wolves move forward diagonally
                for dr, dc in [(1, -1), (1, 1)]:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < 8 and 0 <= nc < 8 and board[nr][nc] == EMPTY:
                        moves.append(((r, c), (nr, nc)))
    return moves

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
               break # Альфа-бета отсечение
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
               break # Альфа-бета отсечение
       return min_eval, best_move


class AlgorithmicMoveGenerator:
    """A simple move generator that picks the first available move."""
    def __init__(self, initial_game_state):
        self.game_state = initial_game_state

    async def get_next_move(self, current_board_state):
        """
        Calculates the next move for the wolves based on a simple algorithm.
        Returns the new board state after the move.
        """
        # The board state needs to be passed in a format this class understands.
        # For simplicity, we'll assume it's the board from the GameState object.
        possible_moves = get_wolf_moves(current_board_state)

        if not possible_moves:
            return None  # No moves available

        # Simple strategy: just pick the first move
        best_move = possible_moves[0]
        
        # Create a new state with this move
        new_board = copy.deepcopy(current_board_state)
        (from_r, from_c), (to_r, to_c) = best_move
        drone_id = current_board_state[from_r][from_c]

        new_board[to_r][to_c] = WOLF
        new_board[from_r][from_c] = EMPTY
        
        self.game_state.board = new_board
        destination_algebraic = to_algebraic((to_r, to_c))

        return {
            "new_board": new_board,
            "drone": drone_id,
            "to": destination_algebraic,
        }


    def evaluate(self):
        sheep_moves = get_sheep_moves(self.game_state.board)
        
        if not sheep_moves:
            return 1000

        sheep_pos = None
        for r in range(8):
            for c in range(8):
                if self.game_state.board[r][c] == SHEEP:
                    sheep_pos = (r, c)
                    break
            if sheep_pos:
                break
        
        if sheep_pos[0] == 0:
            return -1000


        score = 0
        
        # 1. Расстояние овцы до победного края
        sheep_dist_to_win = sheep_pos[0]
        score -= sheep_dist_to_win * 25


        # 2. Мобильность овцы
        sheep_mobility = len(sheep_moves)
        score -= sheep_mobility * 15


        # 3. Характеристики позиции волков
        wolf_positions = []
        for r in range(8):
            for c in range(8):
                if self.game_state.board[r][c] == WOLF:
                    wolf_positions.append((r,c))
        
        if wolf_positions:
            # 4. Продвижение волков вперед
            avg_wolf_row = sum(r for r, c in wolf_positions) / len(wolf_positions)
            score += avg_wolf_row * 10


            # 5. Компактность ("стена" волков)
            wolf_cols = [c for r, c in wolf_positions]
            col_spread = max(wolf_cols) - min(wolf_cols)
            score -= col_spread * 5
            
            # 6. Расстояние до овцы
            dist_to_sheep = sum(abs(r - sheep_pos[0]) + abs(c - sheep_pos[1]) for r, c in wolf_positions)
            score -= dist_to_sheep * 2


        return score
