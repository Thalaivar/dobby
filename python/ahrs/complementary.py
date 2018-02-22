import time

class COMPLEMENTARY():
	__GYRO_CONSTANT = 0.98
	__ACCEL_CONSTANT = 0.02

	def __init__(self):
		self.gyro_euler  = np.zeros((3,))
		self.accel_euler = np.zeros((3,))
		self.mag_euler   = np.zeros((3,))
		self.euler       = np.zeros((3,))
		self.dt 		 = 0
		self.prevtime = 0
		self.nowtime = 0
		self.gyro_prevdata=np.zeros((3,))

	def get_accel_euler(self, accel_data):
		self.accel_euler[1] = math.asin(-accel_data[0]/self.norm(accel_data))
		self.accel_euler[0] = math.atan2(accel_data[1], accel_data[2])

	def get_gyro_euler(self, gyro_data):
		self.gyro_euler[0] = gyro_data[0] + math.sin(self.euler[0]*math.pi/180.0)*math.tan(self.euler[1]*math.pi/180.0)*gyro_data[1] - math.sin(self.euler[1]*math.pi/180.0)*gyro_data[2]
		self.gyro_euler[1] = math.cos(self.euler[0]*math.pi/180.0)*gyro_data[1] - math.sin(self.euler[0]*math.pi/180.0)*gyro_data[2]
		self.gyro_euler[2] = (math.sin(self.euler[0]*math.pi/180.)*gyro_data[1] + math.cos(self.euler[0]*math.pi/180.)*gyro_data[2])/math.cos(self.euler[1]*math.pi/180.)

	def norm(self, array):
		temp = 0

		for i in range(np.size(array)):
			temp = temp + array[i]*array[i]

		temp = math.sqrt(temp)
		return temp

	def euler_comp_update(self):
		self.nowtime = time.clock()
		dt = self.nowtime - self.prevtime
		self.get_accel_euler()
		self.get_gyro_euler()
		self.euler[0] = self.__GYRO_CONSTANT*(self.gyro_euler[0]*dt + self.euler[0]) + self.__ACCEL_CONSTANT*self.accel_euler[0]*180.0/math.pi
		self.euler[1] = self.__GYRO_CONSTANT*(self.gyro_euler[1]*dt + self.euler[1]) + self.__ACCEL_CONSTANT*self.accel_euler[1]*180.0/math.pi
		self.euler[2] = self.__GYRO_CONSTANT*(self.gyro_euler[2]*dt + self.euler[2]) + self.__ACCEL_CONSTANT*self.accel_euler[2]*180.0/math.pi
		self.gyro_prevdata = self.gyro_data
		self.prevtime = self.nowtime


	def debug_calc_euler(self, choice):
		if choice == "accel":
			self.get_accel_euler()
			self.euler[0] = self.accel_euler[0]
			self.euler[1] = self.accel_euler[1]
			self.euler[2] = self.accel_euler[2]

		elif choice == "gyro":
			self.get_gyro_euler()
			self.euler[0] = self.gyro_euler[0]
			self.euler[1] = self.gyro_euler[1]
			self.euler[2] = self.gyro_euler[2]
