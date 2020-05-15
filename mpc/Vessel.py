

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
        self.vertical_velocity = conn.add_stream(getattr, self.vessel.flight(self.vessel.orbit.body.reference_frame),
                                              'vertical_speed')
        self.throttle = conn.add_stream(getattr, self.vessel.control, 'throttle')
        self.thrust = conn.add_stream(getattr, self.vessel, 'thrust')
        self.drag = conn.add_stream(getattr, self.vessel.flight(), 'drag')
        self.yaw = conn.add_stream(getattr, self.vessel.control, 'yaw')
        self.pitch = conn.add_stream(getattr, self.vessel.control, 'pitch')
        self.mass = conn.add_stream(getattr, self.vessel, 'mass')
        self.stage = 0

        # cheating
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.target_direction = (1, 0, 0)

    def next_stage(self):
        if self.stage == 0:
            self.vessel.control.activate_next_stage()
        self.stage += 1

    def get_stage(self):
        return self.stage

    def update_status(self):
        self.status['Altitude'] = self.altitude()
        direction = self.direction()
        self.status['Direction X'] = direction[2]
        self.status['Direction Y'] = direction[1]
        self.status['Direction Z'] = direction[0]
        self.status['Throttle'] = self.throttle()
        self.status['Thrust'] = self.thrust()
        self.status['Drag X'] = self.drag()[2]
        self.status['Drag Y'] = self.drag()[1]
        self.status['Drag Z'] = self.drag()[0]
        self.status['Vertical Velocity'] = self.vertical_velocity()
        self.status['Yaw'] = self.yaw()
        self.status['Pitch'] = self.pitch()

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

    def get_mass(self):
        return self.mass()

    def get_available_thrust(self):
        return self.vessel.max_thrust

