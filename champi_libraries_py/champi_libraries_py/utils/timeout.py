import time


class Timeout:
    
    def __init__(self, duration):
        self.duration = duration
        self.start_time = None

    def start(self):
        self.start_time = time.time()

    def is_elapsed(self):
        return time.time() - self.start_time >= self.duration
    
    def reset(self):
        self.start_time = None
    
    def is_running(self):
        return self.start_time is not None