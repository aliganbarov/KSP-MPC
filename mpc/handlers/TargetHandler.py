

class TargetHandler:

    def __init__(self, upper_alt, lower_alt, velocity):
        self.ascending = True
        self.upper_alt = upper_alt
        self.lower_alt = lower_alt
        self.velocity = velocity

    @staticmethod
    def landing_sliding_target(status):
        if status['Altitude'] > 5000:
            target_alt = status['Altitude'] - 500
            target_vel = -200
        elif status['Altitude'] > 1000:
            target_alt = status['Altitude'] - 800
            target_vel = 0
        else:
            target_alt = -2
            target_vel = 0
        return target_alt, target_vel

    def data_gather_target(self, status):
        target_alt = self.upper_alt
        target_vel = self.velocity
        if status['Altitude'] < self.upper_alt and self.ascending:
            target_alt = status['Altitude'] + 200
            target_vel = self.velocity
        else:
            self.ascending = False
        if status['Altitude'] > self.lower_alt and not self.ascending:
            target_alt = status['Altitude'] - 200
            target_vel = -1 * self.velocity
        else:
            self.ascending = True

        return target_alt, target_vel

