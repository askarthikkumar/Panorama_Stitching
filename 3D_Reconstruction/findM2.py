import numpy as np
import submission as sub
import helper
import matplotlib.pyplot as plt
'''
Q3.3:
    1. Load point correspondences
    2. Obtain the correct M2
    3. Save the correct M2, C2, and P to q3_3.npz
'''


def test_M2_solution(pts1, pts2, intrinsics, M):
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

	F = sub.eightpoint(pts1, pts2, M)
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
		# print("len is ",len(neg))
		if len(neg) < count:
			P_res = P
			err_res = err
			C2_res = C2
			M2_res = M2
			count = len(neg)
		# print(err_res)
	return M2_res, C2_res, P_res


if __name__ == '__main__':
	data = np.load('../data/some_corresp.npz')
	pts1 = data['pts1']
	pts2 = data['pts2']
	intrinsics = np.load('../data/intrinsics.npz')
	im1 = plt.imread('../data/im1.png')
	M2, C2, P = test_M2_solution(pts1, pts2, intrinsics, max(im1.shape))

	np.savez('q3_3', M2=M2, C2=C2, P=P)
