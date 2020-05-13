import krpc
from mpc.Vessel import Vessel
from mpc.Panel import Panel
from mpc.controllers.PID import PID
from mpc.controllers.MPC import MPC
from datetime import datetime
import pandas as pd


class Controller:

    def __init__(self):
        self.conn = krpc.connect()
        self.vessel = Vessel(self.conn)
        self.panel = Panel(self.conn)
        self.error_status = {
            'Altitude Error': 0,
            'Direction X Error': 0,
            'Direction Y Error': 0,
            'Net Force Error': 0,
        }
        self.panel.init_panel(self.error_status)
        self.log_filename = str(datetime.now()) + '.csv'
        self.logs = pd.DataFrame(columns=['Input Throttle', 'Actual Throttle', 'Altitude',
                                          'Vertical Speed', 'Thrust', 'Drag', 'Yaw', 'Pitch'])


    def run(self):
        status = self.vessel.get_status()
        target_alt = 500
        target_direction_y = 0
        target_direction_x = 0
        target_net_force = 0
        target_roll = 0
        # init hover PID
        # pid_hover = PID(.25, -.03, .01, -0.55, status['Altitude'], target_alt)
        # pid_yaw = PID(0, -10, 5, -5, status['Direction X'], target_direction_x)
        # pid_pitch = PID(0, 10, -5, -5, status['Direction Y'], target_direction_y)
        pid_yaw = PID(0, -1, .5, -5, status['Direction X'], target_direction_x)
        pid_pitch = PID(0, 1, -.5, -5, status['Direction Y'], target_direction_y)
        if self.vessel.stage == 'Launch':
            self.vessel.next_stage()
        throttle_mpc = MPC(self.vessel, horizon=15)
        while True:
            if self.vessel.altitude() > 100:
                target_alt = 0
            status = self.vessel.get_status()
            self.error_status['Altitude Error'] = status['Altitude'] - target_alt
            self.error_status['Direction X Error'] = status['Direction X'] - target_direction_x
            self.error_status['Direction Y Error'] = status['Direction Y'] - target_direction_y
            self.panel.update_panel(self.error_status)

            current_state = [self.vessel.altitude(), self.vessel.get_vertical_speed()]
            print("Current state")
            print(current_state)
            t1 = datetime.now()
            new_throttle = throttle_mpc.get_optimal_throttle(current_state, [target_alt, 0])
            t2 = datetime.now()
            print("Time: ", t2-t1)
            self.write_log(new_throttle)
            # new_pitch = pid_pitch.get_val(status['Direction Y'])
            # new_yaw = pid_yaw.get_val(status['Direction X'])
            print({
                'New Throttle': new_throttle,
                # 'New Yaw': new_yaw,
                # 'New Pitch': new_pitch,
            })
            self.vessel.set_throttle(new_throttle)
            # self.vessel.set_pitch(new_pitch)
            # self.vessel.set_yaw(new_yaw)

    def write_log(self, inp_throttle):
        self.logs[-1] = [
            inp_throttle,
            self.vessel.get_throttle(),
            

        ]
        self.logs.to_csv(self.log_filename)