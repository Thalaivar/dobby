import numpy as np
import matplotlib.pyplot as plt

roll, pitch, yaw = 57.272727*np.loadtxt("attitude_data_file.txt", delimiter=',', unpack=True)
encoder_pitch = 360*np.loadtxt("encoder.txt", delimiter='\n', unpack=True)/4000

print(len(pitch))
print(len(encoder_pitch))
#plt.plot(pitch)
#plt.plot(encoder_pitch)
plt.grid()
plt.show()
