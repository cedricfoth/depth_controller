import numpy as np

class live_fir_filter():

    def __init__(self, h):
        
        self.order = len(h) - 1
        self.h = h
        self.prev_x = np.full(self.order, np.nan)
        
    def __call__(self, x):
        y = self.h[0] * x
        
        for i in range(self.order):
            if np.isnan(self.prev_x[i]):
                y += self.h[i + 1] * x
            else:
                y += self.h[i + 1] * self.prev_x[i]
                
        self.prev_x = np.roll(self.prev_x, 1)
        self.prev_x[0] = x
        
        return y