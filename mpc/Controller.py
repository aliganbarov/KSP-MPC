import krpc
from mpc.Vessel import Vessel
from mpc.Panel import Panel
from mpc.controllers.PIDHover import PIDHover
import time


class Controller:

    def __init__(self):
        self.conn = krpc.connect()
        self.vessel = Vessel(self.conn)
        self.panel = Panel(self.conn)
        self.panel.init_panel(self.vessel.status)

    def run(self):
        status = self.vessel.get_status()
        target_alt = 100
        # init hover PID
        pid_hover = PIDHover(.25, -.03, .01, -5.5, status['Altitude'], target_alt, .1)
        while True:
            self.vessel.set_target_direction((1, 0, 0))

            if self.vessel.stage == 'Launch':
                self.vessel.next_stage()
            status = self.vessel.get_status()
            status['Error'] = status['Altitude'] - target_alt
            self.panel.update_panel(status)

            self.vessel.set_throttle(pid_hover.get_throttle(status['Altitude']))

