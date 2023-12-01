import numpy as np

class live_iir_filter():

    def __init__(self, b, a):
        if len(a) != len(b):
            return None
        self.order = len(a) - 1
        self.a = a
        self.b = b
        self.prev_x = np.full(self.order, np.nan)
        self.prev_y = np.full(self.order, np.nan)
        
    def __call__(self, x):
        y = self.b[0] * x

        for n in range(self.order):
            if np.isnan(self.prev_x).any() or np.isnan(self.prev_y).any():
                y = x
            else:
                y += self.b[n + 1] * self.prev_x[n] - self.a[n + 1] * self.prev_y[n]
        
        y /= self.a[0]

        self.prev_x = np.roll(self.prev_x, 1)
        self.prev_y = np.roll(self.prev_y, 1)
        self.prev_x[0] = x
        self.prev_y[0] = y
        
        return y