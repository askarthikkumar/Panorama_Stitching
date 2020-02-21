'''
Q4.2:
    1. Integrating everything together.
    2. Loads necessary files from ../data/ and visualizes 3D reconstruction using scatter
'''
import numpy as np
import importlib
import submission as sub
import helper
import findM2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# importlib.reload(sub)

im1 = plt.imread('../data/im1.png')
im2 = plt.imread('../data/im2.png')
pts = np.load('../data/some_corresp.npz')
pts1 = pts['pts1']
pts2 = pts['pts2']



F = sub.eightpoint(pts1, pts2, max(im1.shape[0], im1.shape[1]))

Karr = np.load('../data/intrinsics.npz')
K1 = Karr['K1']
K2 = Karr['K2']
temple_pts = np.load('../data/templeCoords.npz')
x1 = temple_pts['x1']
y1 = temple_pts['y1']

x2_arr = []
y2_arr = []
for x, y in zip(x1.flatten(), y1.flatten()):
    x2, y2 = sub.epipolarCorrespondence(im1, im2, F, x, y)
    x2_arr.append(x2)
    y2_arr.append(y2)
temple_pts2 = np.stack((np.asarray(x2_arr).reshape(
    1, -1), np.asarray(y2_arr).reshape(1, -1)), axis=0).squeeze().T
# print(temple_pts2.shape)
temple_pts1 = np.stack((x1, y1), axis=0).squeeze().T
# print(temple_pts1.shape)

M2, C2, P = findM2.test_M2_solution(pts1, pts2, Karr, max(im1.shape))

M1 = np.hstack((np.eye(3), np.zeros((3, 1))))
C1 = K1@M1
C2 = K2@M2
P, err = sub.triangulate(C1, temple_pts1, C2, temple_pts2)

# print(err)
# print("error is ", err)
# print(P)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(P[:, 0], P[:, 1], P[:, 2])
plt.show()

np.savez('q4_2.npz', F=F,M1=M1,M2=M2,C1=C1,C2=C2)