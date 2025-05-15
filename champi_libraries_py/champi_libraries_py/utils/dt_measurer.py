import time


class DtMeasurer:
    """Very simple class. Intesest is that at the first call to get_dt(), it will return the expected dt,
    not 0 or a big value."""
    def __init__(self, expected_dt):
        self.last_tick = None
        self.dt = expected_dt

    def tick(self):
        """Call once per loop iteration"""
        if self.last_tick is not None:
            self.dt = time.time() - self.last_tick
        self.last_tick = time.time()
        
    
    def get_dt(self):
        """Get the time elapsed since the last tick"""
        return self.dt