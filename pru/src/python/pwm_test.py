from ti.icss import Icss

pruss = Icss( "/dev/uio/pruss/module" )

pruss.cfg.intc = 0
pruss.cfg.idlemode = 'auto'
pruss.cfg.standbymode = 'auto'
pruss.cfg.standbyreq = False

pruss.core1.full_reset()

with open('../../bin/pwm.bin', 'rb') as f:
	pruss.iram1.write( f.read() )

while True:
	try:
		channel_width = [int(x) for x in input("Enter 4 Ch vals: ").split()]
		for i in range(4):
			pruss.core1.r[i] = int(channel_width[i])

		pruss.core1.run()
	
	except KeyboardInterrupt:
		break
		print("DONE!")
