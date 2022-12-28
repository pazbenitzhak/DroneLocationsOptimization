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

    def setTime(self):
        self.current_time = 0
        return

    def setTimeInterval(self,delta_t):
        interval_size = delta_t

t = time_module(0,1)
t.updateTime()
t.updateTime()
t.updateTime()
t.updateTime()
t.updateTime()
print(t.getTime())

