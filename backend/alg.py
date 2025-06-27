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
            if board[r][c] != SHEEP and board[r][c] != EMPTY:
                # Wolves move forward diagonally
                for dr, dc in [(1, -1), (1, 1)]:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < 8 and 0 <= nc < 8 and board[nr][nc] == EMPTY:
                        moves.append(((r, c), (nr, nc)))
    return moves

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

    # --- МЕТОД ДЛЯ ХОДА ОВЦЫ ---
    async def get_sheep_move(self, current_board_state):
        """
        Вычисляет ход для овцы.
        Стратегия: выбрать первый доступный ход.
        """
        possible_moves = get_sheep_moves(current_board_state)

        if not possible_moves:
            return {"new_board": None, "from": None, "to": None} # Ходов нет, овца проиграла

        # Выбираем первый доступный ход
        sheep_move = possible_moves[0]
        (from_r, from_c), (to_r, to_c) = sheep_move

        # Создаем новое состояние доски
        new_board = copy.deepcopy(current_board_state)
        new_board[to_r][to_c] = SHEEP
        new_board[from_r][from_c] = EMPTY

        # Возвращаем результат в формате, удобном для API
        return {
            "new_board": new_board,
            "from": to_algebraic((from_r, from_c)),
            "to": to_algebraic((to_r, to_c)),
        }
