class PID(object):
    def __init__(self, Kp, Kd, Ki):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.target_value = 0.0
        self.current_value = 0.0
        self.p_error = 0.0
        self.last_p_error = 0.0
        self.d_error = 0
        self.i_error = 0


    def update(self, current_value):
        self.current_value = current_value
        self.p_error = self.target_value - current_value
        self.d_error = self.p_error - self.last_p_error
        self.i_error += self.p_error
        self.last_p_error = self.d_error
        return self.Kp * self.p_error + self.Kd * self.d_error + self.Ki * self.i_error


    def set_target(self, target):
        self.target = target;
        self.d_error

