

class PIDHover:
    def __init__(self, k_c, k_p, k_i, k_d, init_alt, target_alt, rate):
        self.k_c = k_c
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.prev_errors = []
        self.prev_alt = init_alt
        self.target_alt = target_alt
        self.rate = rate

    def get_throttle(self, altitude):
        # PID components
        integral = sum(self.prev_errors) * self.rate
        proportion = altitude - self.target_alt
        differential = (altitude - self.prev_alt) * self.rate

        # adjust throttle
        throttle = self.k_c + self.k_p * proportion + self.k_i * integral + self.k_d * differential

        # Keep pushing / pulling the integral buffer
        self.prev_errors.append(altitude - self.target_alt)
        if len(self.prev_errors) > 1 / self.rate:
            self.prev_errors = self.prev_errors[1:]

        self.prev_alt = altitude
        return throttle
