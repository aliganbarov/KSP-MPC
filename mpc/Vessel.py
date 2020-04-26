
class Vessel:

    def __init__(self, conn):
        self.status = {
            'Orbit speed': 0.,
            'Surface speed': 0.,
            'Altitude': 0.,
            'Error': 0.,
        }
        self.conn = conn
        self.vessel = self.conn.space_center.active_vessel
        obt_frame = self.vessel.orbit.body.non_rotating_reference_frame
        srf_frame = self.vessel.orbit.body.reference_frame
        self.orb_speed = conn.add_stream(getattr, self.vessel.flight(obt_frame), 'speed')
        self.srf_speed = conn.add_stream(getattr, self.vessel.flight(srf_frame), 'speed')
        self.altitude = conn.add_stream(getattr, self.vessel.flight(), 'surface_altitude')
        self.stages = ['Launch', 'Ascending', 'Descending']
        self.stage = self.stages[0]
        self.vessel.auto_pilot.engage()

    def next_stage(self):
        self.vessel.control.activate_next_stage()
        self.stage = self.stages[1]

    def update_status(self):
        self.status['Orbit speed'] = round(self.orb_speed(), 2)
        self.status['Surface speed'] = round(self.srf_speed(), 2)
        self.status['Altitude'] = round(self.altitude(), 2)

    def get_status(self):
        self.update_status()
        return self.status

    def set_throttle(self, throttle):
        self.vessel.control.throttle = throttle

    def set_target_direction(self, direction):
        self.vessel.auto_pilot.target_direction = direction
