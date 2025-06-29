#!/usr/bin/env python3
import socket
import datetime
import json
import os
from collections import defaultdict

class UDPLogServer:
    def __init__(self, host='127.0.0.1', port=9090):
        self.host = host
        self.port = port
        self.drone_logs = defaultdict(list)
        self.running = True
        
        log_dir = 'backend_logs'
        os.makedirs(log_dir, exist_ok=True)
        log_filename = os.path.join(log_dir, f"log_server_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
        
        self.log_file = open(log_filename, 'w', encoding='utf-8')
        
    def __del__(self):
        if self.log_file:
            self.log_file.close()

    def process_log(self, message, addr):
        print(f'msg: {message}')
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        try:
            parts = message.split('|', 2)
            if len(parts) >= 3:
                drone_id, log_type, log_message = parts
            else:
                drone_id = f"unknown_{addr[0]}"
                log_type = "unknown"
                log_message = message
        except:
            drone_id = f"unknown_{addr[0]}"
            log_type = "error"
            log_message = message
        
        log_entry = {
            'timestamp': timestamp,
            'drone_id': drone_id,
            'log_type': log_type,
            'message': log_message,
            'source_ip': addr[0]
        }
        
        self.drone_logs[drone_id].append(log_entry)
        self.display_log(log_entry)
        self.write_to_file(log_entry)

    def write_to_file(self, log_entry):
        log_line = (f"[{log_entry['timestamp']}] {log_entry['drone_id']} "
                    f"({log_entry['log_type']}): {log_entry['message']}\n")
        self.log_file.write(log_line)
        self.log_file.flush()

    def display_log(self, log_entry):
        color_codes = {
            'drone': '\033[92m',
            'landing': '\033[93m',
            'error': '\033[91m',
            'info': '\033[94m',
            'warning': '\033[93m',
            'critical': '\033[91m',
            'unknown': '\033[95m'
        }
        
        color = color_codes.get(log_entry['log_type'].lower(), '\033[0m')
        reset = '\033[0m'
        
        print(f"{color}[{log_entry['timestamp']}] {log_entry['drone_id']} "
              f"({log_entry['log_type']}): {log_entry['message']}{reset}")
    
    def start_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
            server_socket.bind((self.host, self.port))
            
            print(f"🚁 UDP Drone Log Server started on {self.host}:{self.port}")
            print(f"📝 Logging to file: {self.log_file.name}")
            print("=" * 60)
            
            try:
                while self.running:
                    data, addr = server_socket.recvfrom(1024)
                    message = data.decode('utf-8').strip()
                    self.process_log(message, addr)
            except KeyboardInterrupt:
                print("\n🛑 Server shutting down...")
                self.running = False
            except Exception as e:
                print(f"Server error: {e}")
    
    def get_summary(self):
        summary_str = "\n" + "=" * 60 + "\n"
        summary_str += "📊 DRONE LOG SUMMARY\n"
        summary_str += "=" * 60 + "\n"
        print(summary_str)
        self.log_file.write(summary_str)

        for drone_id, logs in self.drone_logs.items():
            drone_summary = f"\n🚁 {drone_id}: {len(logs)} log entries\n"
            print(drone_summary)
            self.log_file.write(drone_summary)
            for log_type in ['drone', 'landing', 'error', 'info', 'warning', 'critical']:
                count = len([l for l in logs if l['log_type'] == log_type])
                if count > 0:
                    type_summary = f"  - {log_type}: {count}\n"
                    print(type_summary, end='')
                    self.log_file.write(type_summary)
        self.log_file.flush()

if __name__ == "__main__":
    import sys
    # Default port is now 9090 to match the class definition
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 9090
    
    server = UDPLogServer(port=port)
    try:
        server.start_server()
    finally:
        server.get_summary()
        server.log_file.close()
