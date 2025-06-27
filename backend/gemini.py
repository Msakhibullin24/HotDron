import json
import asyncio
import urllib.request
import urllib.error
import ssl
import certifi

def generate_game_context_prompt(drone_ids: list[int], sheep_id: int, initial_drone_positions: dict[int, str], initial_sheep_position: str) -> str:
    drone_ids_str = ", ".join(map(str, drone_ids))

    initial_drones_parts = []
    for d_id in drone_ids:
        if d_id in initial_drone_positions:
            initial_drones_parts.append(f"{d_id}:{initial_drone_positions[d_id]}")
    
    initial_input_str = ";".join(initial_drones_parts)
    if sheep_id is not None and initial_sheep_position:
        if initial_input_str:
            initial_input_str += ";"
        initial_input_str += f"{sheep_id}:{initial_sheep_position}"
    initial_input_str += ";"

    MOVE_INITIAL_PROMPT = f"""
привет, твоя задача решать задачу игры в волки и овцы(с дронами), на входе в каждом шаге тебе будет приходить текущее состояние в формате {{id дрона}}:{{клетка}};..., например 151:D4;43:B3; id волков - {drone_ids_str}. id овцы - {sheep_id}. задача, поймать овцу. волки ходят только вперед. ходим как в шашках, по черным клеткам, по диагонали. 1 дрон за ход. отвечай в таком же формате как приходит только с другими (новыми ячейками). изначальный ввод: {initial_input_str} не пиши ничего кроме строки хода. ходи волками так чтобы они были своего рода "роем" и работали совместно, но по очереди
"""
    return MOVE_INITIAL_PROMPT.strip()

class GeminiGameAgent:
    def __init__(self, api_key: str,
                 drone_ids: list[int], sheep_id: int, 
                 initial_drone_positions: dict[int, str], initial_sheep_position: str):
        self._api_key = api_key
        self._conversation_history: list[dict] = []
        
        initial_context_prompt = generate_game_context_prompt(
            drone_ids, sheep_id, initial_drone_positions, initial_sheep_position
        )
        self.add_message("user", initial_context_prompt)
        print("\n--- Инициализация агента с начальным промптом ---")
        print(f"Первое сообщение в истории: {self._conversation_history[0]['content'][:200]}...")

    def add_message(self, role: str, content: str):
        valid_role = 'model' if role == 'model' else 'user'
        self._conversation_history.append({"role": valid_role, "content": content})

    def get_full_history(self) -> list[dict]:
        return list(self._conversation_history)

    async def _make_api_call(self) -> dict:
        base_url_text = 'https://generativelanguage.googleapis.com'
        model = 'gemini-2.5-flash'
        url = f"{base_url_text}/v1beta/models/{model}:generateContent?key={self._api_key}"
        
        print(f"COMMUNICATE WITH GEMINI URL: {url}")

        gemini_api_formatted_content = []
        for message in self._conversation_history:
            role = 'model' if message['role'] == 'model' else 'user'
            gemini_api_formatted_content.append({
                'role': role,
                'parts': [{'text': message['content']}],
            })

        body = {
            'contents': gemini_api_formatted_content,
            'generationConfig': {
                'temperature': 1.2
            }
        }

        headers = {
            'Content-Type': 'application/json',
        }

        request_body_bytes = json.dumps(body).encode('utf-8')
        
        ssl_context = ssl.create_default_context(cafile=certifi.where())
        req = urllib.request.Request(url, data=request_body_bytes, headers=headers, method='POST')

        response_text = ""
        try:
            with await asyncio.to_thread(urllib.request.urlopen, req, context=ssl_context) as response:
                status_code = response.getcode()
                if status_code >= 400:
                    error_data = await asyncio.to_thread(response.read().decode, 'utf-8')
                    raise urllib.error.HTTPError(url, status_code, f"Ошибка Gemini API: {status_code} {error_data}", headers, None)
                
                response_text = await asyncio.to_thread(response.read().decode, 'utf-8')
                data = json.loads(response_text)
                return data
        except urllib.error.HTTPError as e:
            error_body = await asyncio.to_thread(e.read().decode, 'utf-8')
            print(f'Ошибка Gemini API (HTTP {e.code}): {error_body}')
            raise RuntimeError(f"Ошибка Gemini API: {e.code} {error_body}")
        except json.JSONDecodeError:
            print(f"Ошибка декодирования JSON ответа от Gemini API. Ответ: {response_text}")
            raise Exception("Получен некорректный JSON ответ от Gemini API")
        except Exception as e:
            print(f'Непредвиденная ошибка при общении с Gemini API: {e}')
            raise

    async def send_current_state_and_get_response(self, current_state_str: str) -> str:
        self.add_message("user", f"Текущее состояние: {current_state_str}")
        
        print("\n--- Отправка запроса в Gemini (с полной историей) ---")
        try:
            gemini_response = await self._make_api_call()

            if gemini_response and 'candidates' in gemini_response and len(gemini_response['candidates']) > 0:
                first_candidate = gemini_response['candidates'][0]
                if 'content' in first_candidate and 'parts' in first_candidate['content'] and len(first_candidate['content']['parts']) > 0:
                    llm_response_text = first_candidate['content']['parts'][0]['text'].strip()
                    self.add_message("model", llm_response_text)
                    return llm_response_text
                else:
                    print("Предупреждение: Не удалось найти текстовое содержимое (parts) в ответе Gemini.")
                    return ""
            else:
                print("Предупреждение: Пустой или некорректный ответ 'candidates' от Gemini API.")
                return ""

        except Exception as e:
            print(f"Ошибка при получении ответа от Gemini: {e}")
            return ""

class GeminiMoveGenerator:
    def __init__(self, mode, api_key=None, drone_ids=None, sheep_id=None, initial_drone_positions=None, initial_sheep_position=None):
        self.mode = mode
        if not all([api_key, drone_ids, sheep_id, initial_drone_positions, initial_sheep_position]):
            raise ValueError("AI mode requires api_key, drone_ids, sheep_id, and initial positions.")
        self.agent = GeminiGameAgent(api_key, drone_ids, sheep_id, initial_drone_positions, initial_sheep_position)

    async def get_next_move(self, game_state):
        current_state_str = game_state.get_board_state_str()
        ai_move_str = await self.agent.send_current_state_and_get_response(current_state_str)
        print(f"AI response: {ai_move_str}")

        try:
            moved_part = ai_move_str.split(';')[0]
            moved_drone_id, new_cell_alg = moved_part.split(':')
            original_pos_alg = initial_drone_positions_alg.get(int(moved_drone_id))
            from_pos = from_algebraic(original_pos_alg)
            to_pos = from_algebraic(new_cell_alg)

            if from_pos and to_pos:
                print(f"AI moves from {to_algebraic(from_pos)} to {to_algebraic(to_pos)}")
                game = game.make_move((from_pos, to_pos))
                initial_drone_positions_alg[int(moved_drone_id)] = new_cell_alg

            else:
                print("Error: Could not parse AI move.")
        except Exception as e:
            print(f"Error processing AI move: {e}")
