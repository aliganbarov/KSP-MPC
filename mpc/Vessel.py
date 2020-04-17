import krpc


class Vessel:

    def __init__(self):
        self.status = {
            'stage': '',
            'speed': ''
        }
        self.conn = krpc.connect()
        self.vessel = self.conn.space_center.active_vessel

    def next_stage(self):
        self.vessel.control.activate_next_stage()
