import time

class PID:
    def __init__(self, Kp: float, Ki: float,
                 Kd: float, offset: float, e_threshold: float, i_reset: bool):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.offset = offset
        self.e_threshold = e_threshold
        self.i_reset = i_reset

        self.integral = 0.0

        self.prev_e = 0.0
        self.prev_t = time.time()

    def get_integral(self):
        return self.integral
        #t = time.time()
        #return self.Ki * self.prev_e * (t - self.prev_t)
    
    def get_time_diff(self):
        t = time.time()
        return (t - self.prev_t)

    def control(self, e: float) -> float:
        t = time.time()
        if (e >= 0 >= self.prev_e or e <= 0 <= self.prev_e) and self.i_reset:
             self.reset_integral()

        self.proportional = self.Kp * e

        if abs(e) <= self.e_threshold or self.e_threshold == 0:
            self.integral = self.integral + self.Ki * e * (t - self.prev_t)
        else:
            self.integral = 0

        self.differential = self.Kd * (e - self.prev_e) / (t - self.prev_t)

        output = self.offset + self.proportional \
            + self.integral + self.differential
        
        self.prev_t = t
        self.prev_e = e

        return output

    def reset_integral(self) -> None:
        self.integral = 0

    def change_params(self,  Kp: float, Ki: float,
                 Kd: float, offset: float, e_threshold: float, i_reset: bool):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.offset = offset
        self.e_threshold = e_threshold
        self.i_reset = i_reset

        self.integral = 0

        self.prev_e = 0
        self.prev_t = time.time()