class unit_module:
    commander_direction = 0
    soldiers = []

    def __init__(self, solds):
        self.soldiers = solds

    def getCommandDirec(self):
        #TODO: implement uniform distribution
        #after getting a value, need to check together with avg_velocity val
        # whether the new location is valid. In other words, no buildings there
        #if not, get a new value from the uniform distribution and repeat
        return

    def getSoldiers(self):
        return self.soldiers
    
    def getSoldiersNum(self):
        return len(self.getSoldiers())
