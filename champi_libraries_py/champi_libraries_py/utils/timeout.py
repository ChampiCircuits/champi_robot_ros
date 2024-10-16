import time
import diagnostic_msgs


class Timeout:
    
    def __init__(self):
        self.duration = None
        self.start_time = None

    def start(self, duration):
        self.duration = duration
        self.start_time = time.time()

    def is_elapsed(self):
        return time.time() - self.start_time >= self.duration
    
    def reset(self):
        self.start_time = None
    
    def is_running(self):
        return self.start_time is not None
    
    def get_remaining_time(self):
        return self.duration - (time.time() - self.start_time)
    

    def produce_diagnostics(self, stat):
        if self.duration is None or self.start_time is None:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'No timeout set')
            stat.add("Remaining time (s)", 'N/A')
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Running')
            stat.add("Remaining time (s)", str(self.get_remaining_time()))
        return stat
