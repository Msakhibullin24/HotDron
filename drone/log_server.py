#!/usr/bin/env python3
import socket
import datetime
import json
from collections import defaultdict

class UDPLogServer:
    def __init__(self, host='0.0.0.0', port=9999):
        self.host = host
        self.port = port
        self.drone_logs = defaultdict(list)
        self.running = True
        
    def process_log(self, message, addr):
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
        print("\n" + "=" * 60)
        print("📊 DRONE LOG SUMMARY")
        print("=" * 60)
        for drone_id, logs in self.drone_logs.items():
            print(f"\n🚁 {drone_id}: {len(logs)} log entries")
            for log_type in ['drone', 'landing', 'error', 'info', 'warning', 'critical']:
                count = len([l for l in logs if l['log_type'] == log_type])
                if count > 0:
                    print(f"  - {log_type}: {count}")

if __name__ == "__main__":
    import sys
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 9999
    
    server = UDPLogServer(port=port)
    try:
        server.start_server()
    finally:
        server.get_summary()
