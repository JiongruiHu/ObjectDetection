import numpy as np
theta_x = 45
theta_y = 60
height = 100
fov_x = np.tan(np.deg2rad(theta_x/2))*height*2
fov_y = np.tan(np.deg2rad(theta_y/2))*height*2
p_x = 172
p_y = 224

print "fov: ", fov_x, fov_y
print "pixel size: ", float(80)/p_x, float(110)/p_y