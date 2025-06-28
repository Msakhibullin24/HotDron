import math
import copy


# Константы для представления фигур
WOLF = 1
SHEEP = -1
EMPTY = 0


class GameState:
   def __init__(self, wolf_positions, sheep_position):

       self.board = [[EMPTY for _ in range(8)] for _ in range(8)]
       for r, c in wolf_positions:
           self.board[r][c] = WOLF
       self.board[sheep_position[0]][sheep_position[1]] = SHEEP
       self.current_player = SHEEP


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
       from_r, from_c = move[0]
       to_r, to_c = move[1]
      
       piece = new_state.board[from_r][from_c]
       new_state.board[to_r][to_c] = piece
       new_state.board[from_r][from_c] = EMPTY
       new_state.current_player *= -1
       return new_state


   def evaluate(self):

       sheep_moves = self.get_sheep_moves()
      
       if not sheep_moves:
           return 1000 # Победа волков


       sheep_pos = None
       for r in range(8):
           for c in range(8):
               if self.board[r][c] == SHEEP:
                   sheep_pos = (r, c)
                   break
           if sheep_pos:
               break
      
       if sheep_pos[0] == 0:
           return -1000 # Победа овцы


       wolf_positions = []
       for r in range(8):
           for c in range(8):
               if self.board[r][c] == WOLF:
                   wolf_positions.append((r,c))


       # Критически важная проверка: если овца прорвалась за линию хотя бы одного волка
       sheep_row = sheep_pos[0]
       wolf_rows = [r for r, c in wolf_positions]
       if any(sheep_row < r for r in wolf_rows):
           return -900 # Почти поражение для волков, этого нужно избегать любой ценой


       avg_wolf_row = sum(wolf_rows) / len(wolf_positions)


       # 1. Штраф за нарушение строя (разброс по вертикали)
       row_variance = sum((r - avg_wolf_row) ** 2 for r in wolf_rows)
       wall_cohesion_penalty = row_variance * 25 # Очень высокий штраф


       # 2. Бонус за продвижение стены вперед
       wall_advancement_bonus = avg_wolf_row * 20


       # 3. Штраф за мобильность овцы
       mobility_penalty = len(sheep_moves) * 10


       # 4. Штраф за расстояние до овцы
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


def to_algebraic(pos):
   r, c = pos
   return f"{chr(ord('a') + c)}{8 - r}"

def init_game_state(game_state_api):
    from backend.alg import AlgorithmicMoveGenerator
    
    initial_wolf_pos = [(0, 1), (0, 3), (0, 5), (0, 7)]
    initial_sheep_pos = (7, 0) 
    
    game_state = GameState(initial_wolf_pos, initial_sheep_pos)
    
    game_state_api["state"] = game_state
    game_state_api["moveGenerator"] = AlgorithmicMoveGenerator(game_state)
    
    return game_state_api

def set_sheep_pos(game_state_api, sheep_cell):
    from .helpers import from_algebraic
    if not game_state_api or "state" not in game_state_api:
        return

    sheep_pos = from_algebraic(sheep_cell)
    if sheep_pos is None:
        return

    # Find old sheep position and remove it
    old_sheep_pos = None
    for r in range(8):
        for c in range(8):
            if game_state_api["state"].board[r][c] == SHEEP:
                old_sheep_pos = (r, c)
                break
        if old_sheep_pos:
            break
    
    if old_sheep_pos:
        game_state_api["state"].board[old_sheep_pos[0]][old_sheep_pos[1]] = EMPTY

    # Set new sheep position
    game_state_api["state"].board[sheep_pos[0]][sheep_pos[1]] = SHEEP
    game_state_api["sheepPos"] = sheep_cell
