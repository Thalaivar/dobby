import rcpy
import sys
sys.path.insert(0, '../ahrs/rcpy_ahrs')
from rcpy_ahrs import AHRS

class SMC(AHRS):
	def __init__(self):
		self.sliding_surface_constants = np.zeros((3,))
		self.eta					   = np.zeros((3,))
	
	def control_smc(self):
		
