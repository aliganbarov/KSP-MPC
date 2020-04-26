import krpc
from mpc.Vessel import Vessel
from mpc.Panel import Panel
from mpc.controllers.PID import PID
import time


class Controller:

    def __init__(self):
        self.conn = krpc.connect()
        self.vessel = Vessel(self.conn)
        self.panel = Panel(self.conn)
        self.error_status = {
            'Altitude Error': 0,
            'Direction X Error': 0,
            'Direction Y Error': 0,
            # 'Roll Error': 0,
        }
        self.panel.init_panel(self.error_status)

    def run(self):
        status = self.vessel.get_status()
        target_alt = 100
        target_direction_y = 0
        target_direction_x = 0
        target_roll = 0
        # init hover PID
        pid_hover = PID(.25, -.03, .01, -0.55, status['Altitude'], target_alt)
        pid_yaw = PID(0, -10, 5, -5, status['Direction X'], target_direction_x)
        pid_pitch = PID(0, 10, -5, -5, status['Direction Y'], target_direction_y)
        # pid_roll = PID(0, .01, 0.001, -1, status['Roll'], target_roll, rate)
        while True:
            if self.vessel.stage == 'Launch':
                self.vessel.next_stage()
            status = self.vessel.get_status()
            print(status)
            self.error_status['Altitude Error'] = status['Altitude'] - target_alt
            self.error_status['Direction X Error'] = status['Direction X'] - target_direction_x
            self.error_status['Direction Y Error'] = status['Direction Y'] - target_direction_y
            # self.error_status['Roll Error'] = status['Roll'] - target_roll
            self.panel.update_panel(self.error_status)

            new_throttle = pid_hover.get_val(status['Altitude'])
            new_pitch = pid_pitch.get_val(status['Direction Y'])
            new_yaw = pid_yaw.get_val(status['Direction X'], True)
            # new_roll = pid_roll.get_val(status['Roll'])

            print({
                'New Throttle': new_throttle,
                'New Yaw': new_yaw,
                'New Pitch': new_pitch,
                # 'New Roll': new_roll,
            })

            self.vessel.set_throttle(new_throttle)
            self.vessel.set_pitch(new_pitch)
            self.vessel.set_yaw(new_yaw)
            # self.vessel.set_roll(new_roll)

            '''
            self.vessel.set_throttle(pid_hover.get_val(status['Altitude']))
            self.vessel.set_pitch(pid_pitch.get_val(status['Pitch']))
            self.vessel.set_yaw(pid_yaw.get_val(status['Heading']))
            self.vessel.set_roll(pid_roll.get_val(status['Roll']))
            '''
            time.sleep(0.01)
