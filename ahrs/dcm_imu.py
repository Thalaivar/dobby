import numpy as np
import sys
sys.path.insert(0, '../imu/')
from dobby_imu import MPU9250
class DCM(MPU9250):

	def __init__(self):
		self.dcm_matrix = np.zeros((3, 3))
		self.accel_correct_gain = 0
		self.mag_correct_gain   = 0
		self.p_gain				= 0
		self.i_gain				= 0
		self.i_correction       = 0

	def matrix_update(self, dt):

		# first calculate rotation from gyro #
		rot_vec = self.rotation_vector(dt)

		# calculate change in rotation matrix due to rotation of its rows (step 2 of DCM IMU) #
		self.dcm_matrix = self.dcm_matrix @ rot_vec

		# calculate roll and pitch correction
		accel_correct = self.accel_correction()

		# calculate yaw correction
		mag_correct = self.mag_correction()

		# calculate total correction
		gyro_correction = self.accel_correct_gain*accel_correct + self.mag_correct_gain*mag_correct

		# run PI controller
		p_correction = self.p_gain*gyro_correction
		self.i_correction = self.i_gain*gyro_correction*dt + self.i_correction
		correction = p_correction + self.i_correction

		# last step, renormalize
		self.renormalize()

	def to_euler(self):

		# roll is dot of pitch axis with earth vertical axis #
		roll = math.acos(self.dcm_matrix[2,1])

		# pitch is dot of roll axis with earth vertical axis #
 		pitch = math.acos(self.dcm_matrix[2,0])

		# yaw is dot of roll axis with original earth X axis #
		yaw = math.acos(self.dcm_matrix[0,0])

		return np.array(([roll, pitch, yaw]))
	def accel_correction(self):
			wa_x = self.dcm_matrix[2,1]*MPU9250.accel_data[2] - self.dcm_matrix[2,2]*MPU9250.accel_data[1]
			wa_y = self.dcm_matrix[2,2]*MPU9250.accel_data[0] - self.dcm_matrix[2,0]*MPU9250.accel_data[2]
			wa_z = self.dcm_matrix[2,0]*MPU9250.accel_data[1] - self.dcm_matrix[2,1]*MPU9250.accel_data[0]

			return np.array(([wa_x, wa_y, wa_z]))

	def mag_correction(self):
			wm_x = self.dcm_matrix[1,1]*MPU9250.mag_data[2] - self.dcm_matrix[1,2]*MPU9250.mag_data[1]
			wm_y = self.dcm_matrix[1,2]*MPU9250.mag_data[0] - self.dcm_matrix[1,0]*MPU9250.mag_data[2]
			wm_z = self.dcm_matrix[1,0]*MPU9250.mag_data[1] - self.dcm_matrix[1,1]*MPU9250.mag_data[0]

			return np.array(([wa_x, wa_y, wa_z]))

	# ensure orhtogonality and normalize (last step of DCM IMU`)
	def renormalize(self):

		# calculate error due to rotation matrix rows "leaning in" #
		error = self.dcm_matrix[0,] * self.dcm_matrix[1,]

		# correct all three row vectors #
		self.dcm_matrix[0,] = self.dcm_matrix[0,] - (error/2)*self.dcm_matrix[1,]
		self.dcm_matrix[1,] = self.dcm_matrix[1,] - (error/2)*self.dcm_matrix[0,]

		row3_x = self.dcm_matrix[0,1]*self.dcm_matrix[1,2] - self.dcm_matrix[0,2]*self.dcm_matrix[1,1]
		row3_y = self.dcm_matrix[0,2]*self.dcm_matrix[1,0] - self.dcm_matrix[0,0]*self.dcm_matrix[1,2]
		row3_z = self.dcm_matrix[0,0]*self.dcm_matrix[1,1] - self.dcm_matrix[0,1]*self.dcm_matrix[1,0]
		self.dcm_matrix[2,] = np.array(([row3_x, row3_y, row3_z]))

		# normalize all three vectors #
		self.dcm_matrix[0,] = (0.5)*(3 - self.dcm_matrix[0,]*self.dcm_matrix[0,])*self.dcm_matrix[0,]
		self.dcm_matrix[1,] = (0.5)*(3 - self.dcm_matrix[1,]*self.dcm_matrix[1,])*self.dcm_matrix[1,]
		self.dcm_matrix[2,] = (0.5)*(3 - self.dcm_matrix[2,]*self.dcm_matrix[2,])*self.dcm_matrix[2,]


	# calculate rotation vector that updates dcm matrix , i.e. (omega)*dt #
	def rotation_vector(self, dt):

		rot_vec    = np.zeros((3, 3))
		rot_vec[0,] = array('f', [1,   -MPU9250.gyro_data[2]*dt, MPU9250.gyro_data[1]*dt])
		rot_vec[1,] = array('f', [MPU9250.gyro_data[3]*dt, 1, -MPU9250.gyro_data[0]*dt])
		rot_vec[2,] = array('f', [-MPU9250.gyro_data*dt, MPU9250.gyro_data[0]*dt, 1])

		return rot_vec
