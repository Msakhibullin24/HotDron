import math
import copy
import json
from .helpers import to_algebraic
from .constants import SHEEP, EMPTY, DRONE_ID_TO_DRONE_NAMES

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
            if board[r][c] not in [SHEEP, EMPTY]:
                # Wolves move forward diagonally
                for dr, dc in [(1, -1), (1, 1)]:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < 8 and 0 <= nc < 8 and board[nr][nc] == EMPTY:
                        moves.append(((r, c), (nr, nc)))
    return moves

def evaluate_board(board):
    """
    Calculates a heuristic score for the board state from the wolves' perspective.
    A higher score is better for the wolves.
    """
    sheep_moves = get_sheep_moves(board)
    sheep_pos = None
    wolf_positions = []
    wolf_count = 0
    wolf_row_sum = 0

    for r in range(8):
        for c in range(8):
            if board[r][c] == SHEEP:
                sheep_pos = (r, c)
            elif board[r][c] != EMPTY:
                wolf_positions.append((r, c))
                wolf_count += 1
                wolf_row_sum += r

    # Terminal state: Sheep reached the end
    if sheep_pos and sheep_pos[0] == 0:
        return -1000

    # Terminal state: Sheep is trapped
    if not sheep_moves:
        return 1000

    score = 0
    # Heuristic 1: Sheep's distance to goal (wolves want to maximize this)
    if sheep_pos:
        score += sheep_pos[0] * 10

    # Heuristic 2: Restrict sheep's movement (wolves want to minimize this)
    score -= len(sheep_moves) * 5

    # Heuristic 3: Wolves' advancement (wolves want to maximize their row sum)
    if wolf_count > 0:
        score += wolf_row_sum

    # Heuristic 4: Proximity of wolves to the sheep (wolves want to minimize distance)
    if sheep_pos and wolf_count > 0:
        dist_sum = sum(math.sqrt((r - sheep_pos[0])**2 + (c - sheep_pos[1])**2) for r, c in wolf_positions)
        score -= dist_sum

    return score

class _GameStateForMinimax:
    """A helper class to wrap a board state for the minimax algorithm."""
    def __init__(self, board):
        self.board = board

    def get_wolf_moves(self):
        return get_wolf_moves(self.board)

    def get_sheep_moves(self):
        return get_sheep_moves(self.board)

    def make_move(self, move):
        new_board = copy.deepcopy(self.board)
        (from_r, from_c), (to_r, to_c) = move
        piece = new_board[from_r][from_c]
        new_board[to_r][to_c] = piece
        new_board[from_r][from_c] = EMPTY
        return _GameStateForMinimax(new_board)

    def evaluate(self):
        return evaluate_board(self.board)

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

class AlgorithmicMoveGenerator:
    def __init__(self, initial_game_state):
        self.game_state = initial_game_state
        self.depth = 10

    def get_next_move(self, current_board_state):
        sheep_pos_raw = None
        for r in range(8):
            for c in range(8):
                if current_board_state[r][c] == SHEEP:
                    sheep_pos_raw = (r, c)
                    break
            if sheep_pos_raw:
                break
        
        sheepPos = to_algebraic(sheep_pos_raw) if sheep_pos_raw else ""

        # Этот вызов minimax - самая долгая операция
        game_for_minimax = _GameStateForMinimax(current_board_state)
        _, best_move = minimax(game_for_minimax, self.depth, -math.inf, math.inf, True)

        if not best_move:
            # Fallback
            possible_moves = get_wolf_moves(current_board_state)
            if not possible_moves:
                # Это означает, что ходов нет, нужно вернуть пустой результат
                return {
                    "new_board": None, "drone_id": None, "drone": None,
                    "to": None, "to_aruco": None, "sheepPos": sheepPos
                }
            best_move = possible_moves[0]
        
        (from_r, from_c), (to_r, to_c) = best_move
        drone_id = current_board_state[from_r][from_c]

        new_board = copy.deepcopy(current_board_state)
        new_board[to_r][to_c] = drone_id
        new_board[from_r][from_c] = EMPTY
        
        self.game_state.board = new_board  # Этот класс не должен менять состояние напрямую
        destination_algebraic = to_algebraic((to_r, to_c))
        
        to_aruco = None
        to_coords = None
        try:
            with open('aruco_map.json', 'r') as f:
                aruco_map = json.load(f)
            
            for marker in aruco_map:
                # Добавил .lower() для надежности сравнения
                if marker.get('cell') and marker.get('cell').lower() == destination_algebraic.lower():
                    to_aruco = f"aruco_{marker['id']}"
                    to_coords = [marker['x'], marker['y'], marker['z']]
                    break
        except (FileNotFoundError, json.JSONDecodeError):
            print("Warning: Could not load or parse aruco_map.json to find aruco id.")

        return {
            "new_board": new_board,
            "drone_id": drone_id,
            "drone": DRONE_ID_TO_DRONE_NAMES.get(drone_id),
            "to": to_coords,
            "to_aruco": to_aruco,
            "sheepPos": sheepPos
        }

    # --- ИЗМЕНЕНИЕ 2: Убираем 'async' ---
    def get_sheep_move(self, board_state):
        game_for_minimax = _GameStateForMinimax(board_state)
        # Этот вызов minimax - самая долгая операция
        _, best_move = minimax(game_for_minimax, self.depth, -math.inf, math.inf, False)

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