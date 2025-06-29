import logging
import socket

class UDPLogHandler(logging.Handler):
    """Custom logging handler that sends logs to UDP server"""
    def __init__(self, server_ip='127.0.0.1', server_port=9090, drone_name='unknown_drone'):
        super().__init__()
        self.server_ip = server_ip
        self.server_port = server_port
        self.drone_name = drone_name
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
    def emit(self, record):
        try:
            # Map Python logging levels to our custom log types
            level_mapping = {
                'DEBUG': 'info',
                'INFO': 'drone',
                'WARNING': 'warning',
                'ERROR': 'error',
                'CRITICAL': 'critical'
            }
            log_type = level_mapping.get(record.levelname, 'info')
            
            # Format message: drone_id|log_type|message
            message = f"{self.drone_name}|{log_type}|{record.getMessage()}"
            
            # Send to UDP server
            print(f'SENDING {message}')
            self.sock.sendto(message.encode('utf-8'), (self.server_ip, self.server_port))
        except Exception:
            # Don't raise exceptions in logging handler
            pass
    
    def close(self):
        if hasattr(self, 'sock'):
            self.sock.close()
        super().close()


def setup_logging(drone_name, log_level=logging.INFO):
    logger = logging.getLogger()
    logger.setLevel(log_level)
    logger.handlers.clear()
    
    # Console handler
    console_handler = logging.StreamHandler()
    console_formatter = logging.Formatter(f'[{drone_name}] %(asctime)s - %(levelname)s - %(message)s', '%H:%M:%S')
    console_handler.setFormatter(console_formatter)
    logger.addHandler(console_handler)
    
    # UDP handler
    udp_handler = UDPLogHandler(drone_name=drone_name)
    logger.addHandler(udp_handler)
    
    return logger
