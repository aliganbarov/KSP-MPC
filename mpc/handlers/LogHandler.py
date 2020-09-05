import pandas as pd
import time


class LogHandler:

    def __init__(self, filename, status):
        self.logs = pd.DataFrame(columns=['Timestamp',
                                          'Input Throttle', 'horizon', 'dT', 'dt',
                                          'Input Roll', 'Roll P', 'Roll I', 'Roll D',
                                          'Input Pitch', 'Pitch P', 'Pitch I', 'Pitch D',
                                          'Input Yaw', 'Yaw P', 'Yaw I', 'Yaw D'] +
                                         [x for x in status.keys()])
        self.filename = filename

    def write(self, params, inputs, status):
        self.logs.loc[len(self.logs)] = [time.time(),
                                         inputs['New Throttle'], params['horizon'], params['dT'], params['dt'],
                                         inputs['New Roll'], params['Roll P'], params['Roll I'], params['Roll D'],
                                         inputs['New Pitch'], params['Pitch P'], params['Pitch I'], params['Pitch D'],
                                         inputs['New Yaw'], params['Yaw P'], params['Yaw I'], params['Yaw D']] + \
                                        [status[x] for x in status.keys()]

    def save(self):
        self.logs.to_csv(self.filename, index=False)
