import numpy as np
from numpy.linalg import norm
import time
import math

class DCM():

	def __init__(self):
		self.dcm_matrix = np.identity(3)
		self.accel_correct_gain = 1.2
		self.mag_correct_gain   = 0.56
		self.p_gain				= 1.5
		self.i_gain				= 0.78
		self.i_correction       = 0
		self.now_time			= 0
		self.prev_time			= 0

	def matrix_update(self, accel_data, gyro_data, mag_data):
		# calculate dt #
		self.now_time = time.clock()
		dt = self.now_time - self.prev_time
		self.prev_time = self.now_time
		print("*******************************")
		print(self.dcm_matrix)
		if input("type: ") == 1:
			# calculate roll and pitch correction
			accel_correct = self.accel_correction(accel_data)
			print(accel_correct)

			# calculate yaw correction
			mag_correct = self.mag_correction(mag_data/1000)
			print(mag_correct)

			# calculate total correction
			gyro_correction = self.accel_correct_gain*accel_correct + self.mag_correct_gain*mag_correct
			print(gyro_correction)

			# run PI controller
			p_correction = self.p_gain*gyro_correction
			self.i_correction = self.i_gain*gyro_correction*dt + self.i_correction
			correction = p_correction + self.i_correction
			print(correction)

			# first calculate rotation from gyro #
			rot_vec = self.rotation_vector(gyro_data, dt)
			print(rot_vec)

			# corrected rotation vector #
			rot_vec = rot_vec - correction
			print(rot_vec)

			# calculate change in rotation matrix due to rotation of its rows (step 2 of DCM IMU) #
			self.dcm_matrix = np.matmul(self.dcm_matrix, rot_vec)
			print(self.dcm_matrix)

			# last step, renormalize
			self.renormalize()

	def to_euler(self):

		# roll is dot of pitch axis with earth vertical axis #
		roll = math.asin(self.dcm_matrix[2,1])*180.0/math.pi
		
		# pitch is dot of roll axis with earth vertical axis #
		pitch = math.asin(self.dcm_matrix[2,0])*180.0/math.pi
		
		# yaw is dot of roll axis with original earth X axis #
		yaw = math.acos(self.dcm_matrix[0,0])*180.0/math.pi
		
		return np.array([roll, pitch, yaw])

	def accel_correction(self, accel_data):
			wa_x = self.dcm_matrix[2,1]*accel_data[2] - self.dcm_matrix[2,2]*accel_data[1]
			wa_y = self.dcm_matrix[2,2]*accel_data[0] - self.dcm_matrix[2,0]*accel_data[2]
			wa_z = self.dcm_matrix[2,0]*accel_data[1] - self.dcm_matrix[2,1]*accel_data[0]

			return np.array([wa_x, wa_y, wa_z])

	def mag_correction(self, mag_data):
			wm_x = self.dcm_matrix[1,1]*mag_data[2] - self.dcm_matrix[1,2]*mag_data[1]
			wm_y = self.dcm_matrix[1,2]*mag_data[0] - self.dcm_matrix[1,0]*mag_data[2]
			wm_z = self.dcm_matrix[1,0]*mag_data[1] - self.dcm_matrix[1,1]*mag_data[0]

			return np.array([wm_x, wm_y, wm_z])

	# ensure orhtogonality and normalize (last step of DCM IMU`)
	def renormalize(self):
		print("!#@!@#!@#!@#!@##!@#!@#!@")
		# calculate error due to rotation matrix rows "leaning in" #
		error = np.dot(self.dcm_matrix[0,], self.dcm_matrix[1,])
		print(error)
		# correct all three row vectors #
		self.dcm_matrix[0,] = self.dcm_matrix[0,] - (error/2)*self.dcm_matrix[1,]
		self.dcm_matrix[1,] = self.dcm_matrix[1,] - (error/2)*self.dcm_matrix[0,]
		
		print(self.dcm_matrix[0,])
		row3_x = self.dcm_matrix[0,1]*self.dcm_matrix[1,2] - self.dcm_matrix[0,2]*self.dcm_matrix[1,1]
		row3_y = self.dcm_matrix[0,2]*self.dcm_matrix[1,0] - self.dcm_matrix[0,0]*self.dcm_matrix[1,2]
		row3_z = self.dcm_matrix[0,0]*self.dcm_matrix[1,1] - self.dcm_matrix[0,1]*self.dcm_matrix[1,0]
		self.dcm_matrix[2,] = np.array(([row3_x, row3_y, row3_z]))

		# normalize all three vectors #
		self.dcm_matrix[0,] = (0.5)*(3 - np.dot(self.dcm_matrix[0,], self.dcm_matrix[0,]))*self.dcm_matrix[0,]
		self.dcm_matrix[1,] = (0.5)*(3 - np.dot(self.dcm_matrix[1,], self.dcm_matrix[1,]))*self.dcm_matrix[1,]
		self.dcm_matrix[2,] = (0.5)*(3 - np.dot(self.dcm_matrix[2,], self.dcm_matrix[2,]))*self.dcm_matrix[2,]


	# calculate rotation vector that updates dcm matrix , i.e. (omega)*dt #
	def rotation_vector(self, gyro_data, dt):

		rot_vec    = np.zeros((3, 3))
		rot_vec[0,] = np.array([1,   -gyro_data[2]*dt, gyro_data[1]*dt])
		rot_vec[1,] = np.array([gyro_data[2]*dt, 1, -gyro_data[0]*dt])
		rot_vec[2,] = np.array([-gyro_data[1]*dt, gyro_data[0]*dt, 1])

		return rot_vec
	
	def return_row_norm(self):
		
		a = norm(self.dcm_matrix[0,])
		b = norm(self.dcm_matrix[1,])
		c = norm(self.dcm_matrix[2,])

		return np.array([a, b, c])


