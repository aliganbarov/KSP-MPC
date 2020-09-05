

class TerminationHandler:

    @staticmethod
    def landing_termination(target_alt, status):
        if abs(target_alt - status['Altitude']) < 7:
            print("Reached Target. Altitude: " + str(status['Altitude']))
            return True
        return False

    @staticmethod
    def fuel_termination(target_alt, status):
        if not status['Has Fuel']:
            return True
        return False
