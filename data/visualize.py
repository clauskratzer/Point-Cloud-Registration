import numpy as np
import matplotlib.pyplot as plt

data=np.load('lepard/split/4DMatch/bear9AK_Drink/cam1_0008_cam2_0020.npz')
dat_keys=data.keys()
print(list(dat_keys))
xyz1=data['s_pc']
xyz2=data['t_pc']

fig=plt.figure()
ax=fig.add_subplot(projection='3d')
ax.scatter(xyz1[:,0],xyz1[:,1],xyz1[:,2])
ax.scatter(xyz2[:,0],xyz2[:,1],xyz2[:,2])
plt.show()