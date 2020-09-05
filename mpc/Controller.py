import krpc
from mpc.Vessel import Vessel
from mpc.Panel import Panel
from mpc.controllers.PID import PID
from mpc.controllers.MPC import MPC
from mpc.handlers.LogHandler import LogHandler
from mpc.handlers.TerminationHandler import TerminationHandler
from mpc.handlers.TargetHandler import TargetHandler
from datetime import datetime
import time
import pandas as pd


class Controller:

    def __init__(self, saved_filename='20K'):
        self.conn = krpc.connect()
        self.conn.space_center.load(saved_filename)
        self.vessel = Vessel(self.conn)
        self.vessel.stage = 2
        # self.panel = Panel(self.conn)

    def run(self, params, target_handler, termination_handler, log_handler=None):
        times = []
        # get initial status
        status = self.vessel.get_status()
        # Init controllers
        pid_roll = PID(0, params['Roll P'], params['Roll I'], params['Roll D'],
                       status['Roll'], params['Target Roll'])
        pid_pitch = PID(0, params['Pitch P'], params['Pitch I'], params['Pitch D'],
                        status['Direction Y'], params['Target Direction Y'])
        pid_yaw = PID(0, params['Yaw P'], params['Yaw I'], params['Yaw D'],
                      status['Direction X'], params['Target Direction X'])
        throttle_mpc = MPC(self.vessel, horizon=params['horizon'], dT=params['dT'], dt=params['dt'])
        while True:
            t1 = datetime.now()
            status = self.vessel.get_status()

            # get targets
            target_alt, target_vel = target_handler(status)

            # check for termination condition
            if termination_handler(target_alt, status):
                self.vessel.set_throttle(0)
                break

            # get new input values
            new_throttle = throttle_mpc.get_optimal_throttle([status['Altitude'], status['Vertical Velocity']],
                                                             [target_alt, target_vel])
            new_pitch = pid_pitch.get_val(status['Direction Y'])
            new_yaw = pid_yaw.get_val(status['Direction X'])
            new_roll = pid_roll.get_val(status['Roll'])

            # set new values
            self.vessel.set_throttle(new_throttle)
            self.vessel.set_pitch(new_pitch)
            self.vessel.set_yaw(new_yaw)
            self.vessel.set_roll(new_roll)

            # update panel
            # self.panel.update_panel(status)

            # calculate average time for cycle
            t2 = datetime.now()
            times.append((t2-t1).total_seconds())
            # print("Avg total time: ", sum(times)/len(times))

            if log_handler:
                inputs = {
                    'New Throttle': new_throttle,
                    'New Pitch': new_pitch,
                    'New Yaw': new_yaw,
                    'New Roll': new_roll
                }
                log_handler.write(params, inputs, status)

        if log_handler:
            log_handler.save()

    def run_landing(self):
        # set controller parameters
        params = {
            'Roll P': -0.14, 'Roll I': -0.011, 'Roll D': -0.56,
            'Pitch P': 10, 'Pitch I': 1, 'Pitch D': 100,
            'Yaw P': -10, 'Yaw I': -1, 'Yaw D': -100,
            'horizon': 10, 'dT': 0.5, 'dt': 0.5,
            'Target Direction X': 0, 'Target Direction Y': 0, 'Target Roll': -90
        }
        self.run(params, TargetHandler.landing_sliding_target, TerminationHandler.landing_termination)

    def run_data_gathering(self, lower_alt, upper_alt, velocity):
        # get init status
        status = self.vessel.get_status()
        # init log handler
        log_filename = 'logs/data_collection/' + time.strftime("%Y%m%d-%H%M%S") + '.csv'
        log_handler = LogHandler(log_filename, status)
        # init target handler
        target_handler = TargetHandler(upper_alt, lower_alt, velocity)
        # set controller parameters
        params = {
            'Roll P': -0.14, 'Roll I': -0.011, 'Roll D': -0.56,
            'Pitch P': 10, 'Pitch I': 1, 'Pitch D': 100,
            'Yaw P': -10, 'Yaw I': -1, 'Yaw D': -100,
            'horizon': 5, 'dT': 0.5, 'dt': 0.5,
            'Target Direction X': 0, 'Target Direction Y': 0, 'Target Roll': -90
        }
        self.run(params, target_handler.data_gather_target, TerminationHandler.fuel_termination, log_handler)

    def run_model_validation(self):
        log_filename = 'logs/model_validation/' + time.strftime("%Y%m%d-%H%M%S") + '.csv'
        df = pd.DataFrame(columns=['horizon', 'dt', 'Input Altitude', 'Input Velocity', 'Model Altitude', 'Model Velocity',
                                   'Model Thrust', 'Model Drag', 'Model Mass', 'Model Weight', 'Model Acceleration',
                                   'Actual Altitude', 'Actual Velocity', 'Actual Thrust', 'Actual Drag',
                                   'Actual Mass', 'Input Throttle'])
        horizons = [0, 5, 10, 15, 20, 1000]
        dts = [0.1, 0.2, 0.5, 1]
        inputs = [(0, 0, 10), (0, 1, 10), (0, 0, 10), (0, 1, 10), (0, 0, 10), (0, 1, 10), (0, 0, 15), (0.3, 0.3, 20)]
        for horizon in horizons:
            for dt in dts:
                self.conn.space_center.load('launch_stage')
                self.vessel = Vessel(self.conn)
                if self.vessel.get_stage() == 0:
                    self.vessel.next_stage()
                    self.vessel.set_throttle(0.5)
                    time.sleep(4)
                throttle_mpc = MPC(self.vessel, horizon=horizon, dt=dt)
                print('h: ', horizon, 't: ', dt)
                data = throttle_mpc.model_validation(inputs)
                for item in data:
                    df.loc[len(df)] = [horizon, dt] + item
        df.to_csv(log_filename, index=False)

