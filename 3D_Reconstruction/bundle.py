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
def get_world(pts1, pts2, intrinsics, M,F):
	'''
	Estimate all possible M2 and return the correct M2 and 3D points P
	:param pred_pts1:
	:param pred_pts2:
	:param intrinsics:
	:return: M2, the extrinsics of camera 2
			 C2, the 3x4 camera matrix
			 P, 3D points after triangulation (Nx3)
	'''
	K1 = intrinsics['K1']
	K2 = intrinsics['K2']

	E = sub.essentialMatrix(F,K1,K2)
	M1 = np.hstack((np.eye(3),np.zeros((3,1))))
	M2_arr = helper.camera2(E)
	P_res = None
	err_res = None
	C2_res = None
	M2_res = None
	count = pts1.shape[0]+2
	for i in range(4):
		# choose one C2
		M2 = M2_arr[:,:,i]
		# multiply with intrinsics
		C1 = K1@M1
		C2 = K2@M2
		P,err = sub.triangulate(C1,pts1,C2,pts2)
		z = P[:,2]
		neg = z[z<0]
		print("len is ",len(neg))
		if len(neg) < count:
			P_res = P
			e_res = err
			C2_res = C2
			M2_res = M2
			count = len(neg)
			print(err)
			# print(len(neg))
		
	
	return M2_res, C2_res, P_res

def reprojection_error(C1,C2,P,pts1,pts2):
    total = 0
    P_homo = np.concatenate((P, np.ones((P.shape[0],1))),axis=1)
    print(P_homo[0].shape)
    for i in range(pts1.shape[0]):
        x1_proj = C1@P_homo[i]
        x1_proj = x1_proj/x1_proj[-1]
        x2_proj = C2@P_homo[i]
        x2_proj = x2_proj/x2_proj[-1]

        error1 = np.sum((pts1[i]-x1_proj[:2])**2)
        error2 = np.sum((pts2[i]-x2_proj[:2])**2)
        error = error1+error2
        total += error
    return total


im1 = plt.imread('../data/im1.png')
im2 = plt.imread('../data/im2.png')
pts = np.load('../data/some_corresp_noisy.npz')
pts1 = pts['pts1']
pts2 = pts['pts2']
Karr = np.load('../data/intrinsics.npz')
K1 = Karr['K1']
K2 = Karr['K2']
M = np.max(im1.shape)

# F_eight = sub.eightpoint(pts1,pts2, np.max(im1.shape))
# helper.displayEpipolarF(im1,im2,F_eight)
F, inliers = sub.ransacF(pts1, pts2, M)
# helper.displayEpipolarF(im1,im2,F)
p1 = pts1[inliers.flatten()]
p2 = pts2[inliers.flatten()]
M1 = np.hstack((np.eye(3), np.zeros((3, 1))))
M2_init, C2, P_init = get_world(p1, p2, Karr, max(im1.shape),F)
z_max = np.median(P_init[:,2])
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(P_init[:, 0]/z_max, P_init[:, 1]/z_max, P_init[:, 2]/z_max, c='b')

C1 = K1@M1
C2 = K2@M2_init
print("error before bundle ",reprojection_error(C1,C2,P_init,p1,p2))

M2_opt, P_opt = sub.bundleAdjustment(K1,M1,p1,K2,M2_init, p2, P_init)
C1 = K1@M1
C2 = K2@M2_opt
print("error after bundle ", reprojection_error(C1,C2,P_opt,p1,p2))

z_max = np.median(P_opt[:,2])
ax.scatter(P_opt[:, 0]/z_max, P_opt[:, 1]/z_max, P_opt[:, 2]/z_max, c='r')
ax.legend(['Before Bundle Adjustment', 'After Bundle Adjustment'])
plt.show()
