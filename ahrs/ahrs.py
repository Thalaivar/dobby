import time
import dobby_imu

class ahrs(MPU9250):
    self.gyro_time = 0
    self.gyro_prevtime = 0
    self.gyro_prevdata=np.zeros((3,))
    self.gyro_euler  = np.zeros((3,))
    self.accel_euler = np.zeros((3,))
    self.mag_euler   = np.zeros((3,))

    def __init__(self):
        #define initialisation functions

    def get_euler(self):
        #Function updates Euler Angles from all three Sensors
        #get_gyro_euler()
        #get_accel_euler()
        #get_mag_euler()

    def get_gyro_euler(self):
        self.gyro_time = time.clock()
        dt_time = self.gyro_time - self.gyro_prevtime
        di = np.zeros((3,))
        if dt_time == 0:
            dt_time = 0.0033 #Assuming 300Hz Frequency. This is a failsafe that needs to be discussed
        di = dt_time * (self.gyro_data - self.gyro_prevdata)

        self.gyro_euler += di

        self.gyro_prevdata = self.gyro_data
        self.gyro_prevtime = self.gyro_time
