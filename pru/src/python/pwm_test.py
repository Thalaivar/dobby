from ti.icss import Icss
import ctypes

pruss = Icss( "/dev/uio/pruss/module" )

pruss.cfg.intc = 0
pruss.cfg.idlemode = 'auto'
pruss.cfg.standbymode = 'auto'
pruss.cfg.standbyreq = False

pruss.core1.full_reset()

with open('../../bin/dobby_pwm.bin', 'rb') as f:
	pruss.iram1.write( f.read() )

channel_width = [8*int(x) for x in input("Enter 4 Ch vals: ").split()]

data = pruss.dram1.map(ctypes.c_uint32 * 4, offset=0)
data[:] = channel_width
pruss.core1.run()
	
while True:
	try:
		channel_width = [8*int(x) for x in input("Enter 4 Ch vals: ").split()]
		data[:] = channel_width

	except KeyboardInterrupt:
		break
		print("DONE!")
