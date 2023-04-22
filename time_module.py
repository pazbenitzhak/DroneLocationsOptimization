class time_module:
    current_time = 0
    interval_size = 1

    def __init__(self,curr_time,interv_size):
        self.current_time = curr_time
        self.interval_size=interv_size

    def getTime(self):
        return self.current_time

    def updateTime(self):
        self.current_time+=self.interval_size
        return

    def setTime(self,t):
        self.current_time = t
        return

    def setTimeInterval(self,delta_t):
        self.interval_size = delta_t

    def getTimeInterval(self):
        return self.interval_size

#to run it need python time_module.py