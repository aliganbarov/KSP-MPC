import numpy as np

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

        ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
            position=self.vessel.orbit.body.reference_frame,
            rotation=self.vessel.surface_reference_frame)
        self.direction = conn.add_stream(getattr, self.vessel.flight(ref_frame), 'direction')
        # self.retrograde = conn.add_stream(getattr, self.vessel.flight(ref_frame), 'retrograde')
        self.velocity = conn.add_stream(getattr, self.vessel.flight(ref_frame), 'velocity')
        conn.drawing.add_direction((1, 0, 0), ref_frame)
        conn.drawing.add_direction((0, 0.5, 0), ref_frame)
        conn.drawing.add_direction((0, 0, 0.2), ref_frame)

        self.vertical_velocity = conn.add_stream(getattr, self.vessel.flight(self.vessel.orbit.body.reference_frame),
                                                 'vertical_speed')
        self.throttle = conn.add_stream(getattr, self.vessel.control, 'throttle')
        self.thrust = conn.add_stream(getattr, self.vessel, 'thrust')
        self.drag = conn.add_stream(getattr, self.vessel.flight(), 'drag')
        self.yaw = conn.add_stream(getattr, self.vessel.control, 'yaw')
        self.pitch = conn.add_stream(getattr, self.vessel.flight(), 'pitch')
        self.roll = conn.add_stream(getattr, self.vessel.flight(self.vessel.orbit.body.reference_frame), 'roll')
        self.mass = conn.add_stream(getattr, self.vessel, 'mass')
        self.fuel = conn.add_stream(getattr, self.vessel.parts.engines[0], 'has_fuel')
        self.stage = 0

        # cheating
        # self.vessel.auto_pilot.engage()
        # self.vessel.auto_pilot.target_direction = (1, 0, 0)

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
        # self.status['Throttle'] = self.throttle()
        self.status['Thrust'] = self.thrust()
        self.status['Drag X'] = self.drag()[2]
        self.status['Drag Y'] = self.drag()[1]
        self.status['Drag Z'] = self.drag()[0]
        self.status['Vertical Velocity'] = self.vertical_velocity()
        # self.status['Yaw'] = self.yaw()
        # self.status['Pitch'] = self.pitch()
        self.status['Roll'] = self.roll()
        self.status['Mass'] = self.mass()
        self.status['Has Fuel'] = self.fuel()
        # get custom retrograde as inverse of normalized velocity
        velocity = self.velocity()
        retrograde = (-velocity[0], -velocity[1], -velocity[2]) / np.linalg.norm(velocity)
        # print(f"Custom retrograde: {retrograde}")
        # print(f"Direction: {direction}")
        self.status['Retrograde X'] = retrograde[2]
        self.status['Retrograde Y'] = retrograde[1]

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

    def set_control_value(self, key, value):
        if key == 'Throttle':
            self.vessel.control.throttle = value
        elif key == 'Pitch':
            self.vessel.control.pitch = value
        elif key == 'Yaw':
            self.vessel.control.yaw = value
        elif key == 'Roll':
            self.vessel.control.roll = value

    def set_autopilot(self):
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.target_direction = (1, 0, 0)
