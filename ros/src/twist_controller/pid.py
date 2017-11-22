
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM, min_i=-50., max_i=50.):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx
        self.min_i = min_i
        self.max_i = max_i
        self.int_val = self.last_int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0

    def step(self, error, sample_time):
        self.last_int_val = self.int_val

        # clip the integral sum to prevent excessive integral error and gain
        integral = max(self.min_i, min(self.int_val + error * sample_time, self.max_i))
        derivative = (error - self.last_error) / sample_time

        y = self.kp * error + self.ki * self.int_val + self.kd * derivative
        val = max(self.min, min(y, self.max))

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        #print("error: ", error)
        #print("d_error: ", derivative)
        #print("i_error: ", self.int_val)
        return val
