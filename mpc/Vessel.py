import math


class Vessel:

    def __init__(self, conn):
        self.status = {
            'Altitude': 0.,
            'Direction X': 0,
            'Direction Y': 0,
            'Direction Z': 0,
        }
        self.conn = conn
        self.vessel = self.conn.space_center.active_vessel
        self.altitude = conn.add_stream(getattr, self.vessel.flight(), 'surface_altitude')
        self.direction = conn.add_stream(getattr, self.vessel.flight(), 'direction')
        self.stages = ['Launch', 'Ascending', 'Descending']
        self.stage = self.stages[0]

        # cheating
        # self.vessel.auto_pilot.engage()
        # self.vessel.auto_pilot.target_direction = (1, 0, 0)

    def next_stage(self):
        self.vessel.control.activate_next_stage()
        self.stage = self.stages[1]

    def update_status(self):
        self.status['Altitude'] = self.altitude()
        direction = self.direction()
        self.status['Direction X'] = direction[2]
        self.status['Direction Y'] = direction[1]
        self.status['Direction Z'] = direction[0]

    def get_status(self):
        self.update_status()
        return self.status

    def set_throttle(self, throttle):
        self.vessel.control.throttle = throttle

    def set_pitch(self, pitch):
        self.vessel.control.pitch = pitch

    def set_yaw(self, yaw):
        self.vessel.control.yaw = yaw

    def set_roll(self, roll):
        self.vessel.control.roll = roll
