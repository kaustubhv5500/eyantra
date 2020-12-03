import matplotlib
import matplotlib.pyplot as plt
import pandas as pd

fig = plt.figure()
ax = fig.add_subplot(111)

position_list = pd.read_csv('odom.csv')
buffer = 0.6
waypts = [(-9.1,-1.2),(10.7,10.5),(12.6,-1.9),(18.2,-1.4),(-2,4)]

plt.plot(position_list['pose.pose.position.x'],position_list['pose.pose.position.y'])

#buffer is half of the diagonal of the square, then length can be calculated as... 
lenght = (((2.0*buffer)**2)/2.0)**0.5 

for pt in waypts:
      rect = matplotlib.patches.Rectangle((pt[0]-buffer,pt[1]),lenght,lenght,angle=-45.0,edgecolor='r',facecolor='none')
      ax.add_patch(rect)

plt.grid()
plt.xlim([-10,20]) 
plt.ylim([-5, 15]) 
plt.show()