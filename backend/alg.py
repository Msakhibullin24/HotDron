import math
import copy
from .helpers import to_algebraic, from_algebraic
from .constants import SHEEP, EMPTY, DRONE_IDS

WOLF = 1

class AlgorithmicMoveGenerator:
    def __init__(self, initial_game_state):
        self.game_state = initial_game_state
        self.depth = 9

    def _convert_to_internal_board(self, main_board):
        internal_board = [[EMPTY for _ in range(8)] for _ in range(8)]
        wolf_positions = []
        sheep_position = None

        for r in range(8):
            for c in range(8):
                cell = main_board[r][c]
                if cell in DRONE_IDS:
                    internal_board[r][c] = WOLF
                    wolf_positions.append((r, c))
                elif cell == SHEEP:
                    internal_board[r][c] = SHEEP
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

        sheep_pos_coords = internal_state.get_sheep_moves()[0][0]
        sheep_pos_alg = to_algebraic(sheep_pos_coords)


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

class GameState:
    def __init__(self, wolf_positions, sheep_position):
        self.board = [[EMPTY for _ in range(8)] for _ in range(8)]
        for r, c in wolf_positions:
            self.board[r][c] = WOLF
        if sheep_position:
            self.board[sheep_position[0]][sheep_position[1]] = SHEEP
        self.current_player = SHEEP

    def is_valid_pos(self, r, c):
        return 0 <= r < 8 and 0 <= c < 8

    def get_sheep_moves(self):
        moves = []
        for r in range(8):
            for c in range(8):
                if self.board[r][c] == SHEEP:
                    for dr in [-1]:
                        for dc in [-1, 1]:
                            nr, nc = r + dr, c + dc
                            if self.is_valid_pos(nr, nc) and self.board[nr][nc] == EMPTY:
                                moves.append(((r, c), (nr, nc)))
                    return moves
        return moves

    def get_wolf_moves(self):
        moves = []
        for r in range(8):
            for c in range(8):
                if self.board[r][c] == WOLF:
                    dr = 1
                    for dc in [-1, 1]:
                        nr, nc = r + dr, c + dc
                        if self.is_valid_pos(nr, nc) and self.board[nr][nc] == EMPTY:
                            moves.append(((r, c), (nr, nc)))
        return moves

    def make_move(self, move):
        new_state = copy.deepcopy(self)
        (from_r, from_c), (to_r, to_c) = move
        piece = new_state.board[from_r][from_c]
        new_state.board[to_r][to_c] = piece
        new_state.board[from_r][from_c] = EMPTY
        new_state.current_player *= -1
        return new_state

    def evaluate(self):
        sheep_moves = self.get_sheep_moves()
        if not sheep_moves:
            return 1000

        sheep_pos = None
        for r in range(8):
            for c in range(8):
                if self.board[r][c] == SHEEP:
                    sheep_pos = (r, c)
                    break
            if sheep_pos: break
        
        if not sheep_pos: return 1000

        if sheep_pos[0] == 0:
            return -1000

        wolf_positions = []
        for r in range(8):
            for c in range(8):
                if self.board[r][c] == WOLF:
                    wolf_positions.append((r, c))

        sheep_row = sheep_pos[0]
        wolf_rows = [r for r, c in wolf_positions]
        if any(sheep_row < r for r in wolf_rows):
            return -900

        avg_wolf_row = sum(wolf_rows) / len(wolf_positions)
        row_variance = sum((r - avg_wolf_row) ** 2 for r in wolf_rows)
        wall_cohesion_penalty = row_variance * 25
        wall_advancement_bonus = avg_wolf_row * 20
        mobility_penalty = len(sheep_moves) * 10
        dist_to_sheep = sum(abs(r - sheep_row) for r in wolf_rows)
        proximity_penalty = dist_to_sheep * 5

        score = wall_advancement_bonus - wall_cohesion_penalty - proximity_penalty - mobility_penalty
        return score

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
