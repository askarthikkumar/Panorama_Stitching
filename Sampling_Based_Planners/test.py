import csv
import numpy as np
import matplotlib.pyplot as plt

reader = csv.reader(open("nodes.txt"), delimiter=' ')
a=[i[:-1] for i in reader]
arr=np.array(a).astype(float)
# print(arr.shape)
# print(arr[0,:])
fig=plt.figure()
ax=fig.add_subplot(111)
ax.scatter(arr[:-2,0],arr[:-2,1])
ax.scatter(arr[-2,0],arr[-2,1],color='r')
ax.scatter(arr[-1,0],arr[-1,1],color='g')
ax.set_xlabel('X')
ax.set_ylabel('Y')
plt.show()
