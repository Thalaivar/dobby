from ti.icss import Icss
import sys
sys.path.insert(0, '/home/debian/dobby/ahrs/')
from dobby_ahrs import AHRS

class MOTORS(AHRS):
		
		#servo motor pins are conencted to pru1
		__PRU_NUM = 1
		__FIRMWARE = '../../bin/dobby_pwm.bin'
		__EQUIV_CYCLES = 100/12

		def __init__(self):
			self.pruss = Icss( "/dev/uio/pruss/module" )
			self.motor1 = None
			self.motor2 = None
			self.motor3 = None
			self.motor4 = None
			self.data = None
		
		def init_pwm_pru(self):
			self.pruss.cfg.intc = 0
			self.pruss.cfg.idlemode = 'auto'
			self.pruss.cfg.standbymode = 'auto'
			self.pruss.cfg.standbyreq = True  #False -> OCP master port enable
			self.pruss.core1.full_reset()
			self.data = pruss.dram1.map(ctypes.c_uint32 * 4, offset=0)
	
		def load_pwm_firmware(self):
			with open(self.__FIRMWARE, 'rb') as f:
				self.pruss.iram0.write( f.read() )
		
		def calibrate_esc(self):
			motor_outputs = [self.motor1, self.motor2, self.motor3, self.motor4]
			
			print("Make sure all props are removed before calibrating ESCs!")
			time.sleep(4)
			print("Sending 2000 uS pulse")
			
			motor_outputs = [ x - x + 1000 for x in motor_outputs]
			self.write()

			time.sleep(1)
			print("Sending 1000 uS pulse")

			motor_outputs = [ x - x + 1000 for x in motor_outputs]
			self.write()

			time.sleep(2)
			print("ESC calibrated!")

		def update(self):
		
		
		
		def write(self):
			self.data = [self.motor1, self.motor2, self.motor3, self.motor4]

		def pwm_to_cycles(self, pwm):
			return self.__EQUIV_CYCLES*pwm
