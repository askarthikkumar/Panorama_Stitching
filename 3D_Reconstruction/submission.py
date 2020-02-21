"""
Homework4.
Replace 'pass' by your implementation.
"""

# Insert your package here
import numpy as np
import helper
import matplotlib.pyplot as plt
import scipy.optimize
'''
Q2.1: Eight Point Algorithm
    Input:  pts1, Nx2 Matrix
            pts2, Nx2 Matrix
            M, a scalar parameter computed as max (imwidth, imheight)
    Output: F, the fundamental matrix
'''


def eightpoint(pts1, pts2, M):
    # Replace pass by your implementation
    # scale the points

    # insert scaling code segment
    pts2_ = pts2/M
    pts1_ = pts1/M

    x2 = pts2_[:, 0].reshape(-1, 1)
    y2 = pts2_[:, 1].reshape(-1, 1)

    x1 = pts1_[:, 0].reshape(-1, 1)
    y1 = pts1_[:, 1].reshape(-1, 1)

    A = np.hstack([x1*x2, x2*y1, x2, y2*x1, y2*y1,
                   y2, x1, y1, np.ones(x2.shape)])

    u, s, vh = np.linalg.svd(A)
    v = vh.T

    f = v[:, -1]
    f = f.reshape((3, 3))

    # rank2 constraint
    u_f, s_f, vh_f = np.linalg.svd(f)
    s_f[-1] = 0
    f_new = u_f@np.diag(s_f)@vh_f

    # refine F
    f_refine = helper.refineF(f_new, pts1_, pts2_)
    # f_refine = f_new
    # offset normalisation
    scale = np.diag([1/M, 1/M, 1])
    f_unorm = scale.T@f_refine@scale

    return f_unorm


'''
Q2.2: Seven Point Algorithm
    Input:  pts1, Nx2 Matrix
            pts2, Nx2 Matrix
            M, a scalar parameter computed as max (imwidth, imheight)
    Output: Farray, a list of estimated fundamental matrix.
'''


def sevenpoint(pts1, pts2, M, ref=True):
    pts2 = pts2/M
    pts1 = pts1/M

    x2 = pts2[:, 0].reshape(-1, 1)
    y2 = pts2[:, 1].reshape(-1, 1)

    x1 = pts1[:, 0].reshape(-1, 1)
    y1 = pts1[:, 1].reshape(-1, 1)

    A = np.hstack([x1*x2, x2*y1, x2, y2*x1, y2*y1,
                   y2, x1, y1, np.ones(x2.shape)])

    u, s, vh = np.linalg.svd(A)

    f1 = vh.T[:, -1].reshape(3, 3)
    f2 = vh.T[:, -2].reshape(3, 3)

    scale = np.diag([1/M, 1/M, 1])
    def fun(a): return np.linalg.det(a * f1 + (1 - a) * f2)
    a0 = fun(0)
    a1 = 2*(fun(1)-fun(-1))/3 - (fun(2) - fun(-2))/12
    a2 = 0.5*fun(1) + 0.5*fun(-1) - fun(0)
    a3 = fun(1) - (a0+a1+a2)
    coeff = [a3, a2, a1, a0]
    roots = np.roots(coeff)
    # print(roots)
    roots = roots[np.logical_not(np.iscomplex(roots))]
    farr_dup = [root*f1+(1-root)*f2 for root in roots]
    if ref is True:
        farr_refine = [helper.refineF(f, pts1, pts2) for f in farr_dup]
        f_unorm = [scale.T@f@scale for f in farr_refine]
    else:
        f_unorm = [scale.T@f@scale for f in farr_dup]

    return f_unorm


'''
Q3.1: Compute the essential matrix E.
    Input:  F, fundamental matrix
            K1, internal camera calibration matrix of camera 1
            K2, internal camera calibration matrix of camera 2
    Output: E, the essential matrix
'''


def essentialMatrix(F, K1, K2):
    # Replace pass by your implementation
    return K2.T@F@K1


'''
Q3.2: Triangulate a set of 2D coordinates in the image to a set of 3D points.
    Input:  C1, the 3x4 camera matrix
            pts1, the Nx2 matrix with the 2D image coordinates per row
            C2, the 3x4 camera matrix
            pts2, the Nx2 matrix with the 2D image coordinates per row
    Output: P, the Nx3 matrix with the corresponding 3D points per row
            err, the reprojection error.
'''


def triangulate(C1, pts1, C2, pts2):
    # Replace pass by your implementation
    Xarr = []
    i = 0
    for i in range(pts1.shape[0]):
        xC3 = pts1[i, 0]*C1[2, :]
        yC3 = pts1[i, 1]*C1[2, :]
        xC3_ = pts2[i, 0]*C2[2, :]
        yC3_ = pts2[i, 1]*C2[2, :]
        rC1 = C1[0, :]
        rC2 = C1[1, :]
        rC1_ = C2[0, :]
        rC2_ = C2[1, :]

        A = np.vstack([(rC1-xC3).reshape(1, 4), (yC3-rC2).reshape(1, 4),
                       (rC1_-xC3_).reshape(1, 4), (yC3_-rC2_).reshape(1, 4)])
        u, s, vh = np.linalg.svd(A)
        X = vh.T[:, -1]
        X = X/X[-1]
        Xarr.append(np.copy(X))

    total = 0
    for i in range(pts1.shape[0]):
        x1_proj = C1@Xarr[i]
        x1_proj = x1_proj/x1_proj[-1]
        x2_proj = C2@Xarr[i]
        x2_proj = x2_proj/x2_proj[-1]

        error1 = np.sum((pts1[i]-x1_proj[:2])**2)
        error2 = np.sum((pts2[i]-x2_proj[:2])**2)
        error = error1+error2
        total += error

    P = np.stack(Xarr, axis=1).T[:, :3]
    assert(P.shape == (pts1.shape[0], 3))
    return P, total


'''
Q4.1: 3D visualization of the temple images.
    Input:  im1, the first image
            im2, the second image
            F, the fundamental matrix
            x1, x-coordinates of a pixel on im1
            y1, y-coordinates of a pixel on im1
    Output: x2, x-coordinates of the pixel on im2
            y2, y-coordinates of the pixel on im2

'''


def epipolarCorrespondence(im1, im2, F, x1, y1):
    # l2 = x1.T@F
    x1_homo = np.array([x1, y1, 1]).reshape(3, 1)
    l = ((x1_homo.T)@(F.T)).flatten()
    slope = -(l[0]/l[1])
    if slope < 1:
        wsize = 5
        s = max(x1-50, 0)
        e = min(x1+50, im1.shape[1])
        start = s + (wsize-1)/2
        end = e - (wsize-1)/2
        x_arr = np.arange(start, end)
        y_arr = np.round(-(l[0]*x_arr+l[2])/l[1]).astype(np.int32)

    if slope >= 1:
        wsize = 5
        s = max(y1-50, 0)
        e = min(y1+50, im1.shape[0])
        start = s + (wsize-1)/2
        end = e - (wsize-1)/2
        y_arr = np.arange(start, end)
        x_arr = np.round(-(l[1]*y_arr+l[2])/l[0]).astype(np.int32)

    d = (wsize-1)/2
    r1 = im1[int(y1-d):int(y1+d+1), int(x1-d):int(x1+d+1)]
    err = []
    for x_can, y_can in zip(x_arr, y_arr):
        r2 = im2[int(y_can-d):int(y_can+d+1), int(x_can-d):int(x_can+d+1), :]
        err.append((np.sum((r1-r2)**2)))

    pos = np.argmin(np.asarray(err))
    return x_arr[pos], y_arr[pos]


'''
Q5.1: RANSAC method.
    Input:  pts1, Nx2 Matrix
            pts2, Nx2 Matrix
            M, a scaler parameter
    Output: F, the fundamental matrix
            inliers, Nx1 bool vector set to true for inliers
'''


def ransacF(pts1, pts2, M):
    # Replace pass by your implementation

    mismatches_min = pts1.shape[0]+2
    num_iter = 1500
    pts1_homo = np.concatenate((pts1, np.ones((pts1.shape[0], 1))), axis=1).T
    # print(pts1_homo.shape)
    pts2_homo = np.concatenate((pts2, np.ones((pts2.shape[0], 1))), axis=1).T
    pts1_exp = pts1_homo[:, None].transpose(2, 0, 1)
    pts1_exp_ = pts1_homo[:, None].transpose(2, 1, 0)
    pts2_exp = pts2_homo[:, None].transpose(2, 1, 0)
    pts2_exp_ = pts2_homo[:, None].transpose(2, 0, 1)
    F_best = None
    mask_best = None
    thresh = 1e-3
    np.random.seed(0) 
    for i in range(num_iter):
        # print("Iter is " + str(i))
        select = np.random.choice(pts1.shape[0], 7)
        p1 = pts1[select,:]
        p2 = pts2[select,:]
        Farr = sevenpoint(p1, p2, M, ref=False)
        for F in Farr:
            # rewrite this part
            results = (pts2_exp)@F@pts1_exp
            results = np.abs(results.squeeze())
            mask_idx = np.flatnonzero(results > thresh)
            if(len(mask_idx) < mismatches_min):
                F_best = F
                # print("min is " + str(mismatches_min))
                mask_best = (results <= thresh)
                mismatches_min = len(mask_idx)

    # print("global min is " + str(mismatches_min))
    # get best estimate from inliers
    mask_best = mask_best.reshape(-1, 1)
    p1 = pts1[mask_best.flatten()]
    p2 = pts2[mask_best.flatten()]
    F_best = helper.refineF(F_best,p1,p2)

    assert(mask_best.shape == (pts1.shape[0], 1))
    assert(F_best.shape == (3, 3))

    return F_best, mask_best


'''
Q5.2: Rodrigues formula.
    Input:  r, a 3x1 vector
    Output: R, a rotation matrix
'''


def rodrigues(r):
    # Replace pass by your implementation
    rmag = np.sqrt(np.sum(r**2))
    r_vec = r/rmag if rmag != 0 else r
    if rmag == 0:
        return np.eye(3)

    A = np.array([[0 ,-r_vec[2, 0], r_vec[1, 0]], [r_vec[2, 0], 0, -r_vec[0, 0]], [-r_vec[1, 0], r_vec[0, 0], 0]])
    I = np.eye(3)
    R = I + np.sin(rmag)*A + (1-np.cos(rmag))*(A@A)
    return R


'''
Q5.2: Inverse Rodrigues formula.
    Input:  R, a rotation matrix
    Output: r, a 3x1 vector
'''


def invRodrigues(R):
    # Replace pass by your implementation
    trace = R[0, 0]+R[1, 1]+R[2, 2]
    theta = np.arccos((trace-1)/2)
    if theta == 0:
        return np.array([0, 0, 0])
    
    r_vec = (1/(2*np.sin(theta)))*(np.array([R[2, 1]-R[1, 2], R[0, 2]-R[2, 0], R[1, 0]-R[0, 1]]))
    r = theta*r_vec.reshape(3,1)
    return r


'''
Q5.3: Rodrigues residual.
    Input:  K1, the intrinsics of camera 1
            M1, the extrinsics of camera 1
            p1, the 2D coordinates of points in image 1
            K2, the intrinsics of camera 2
            p2, the 2D coordinates of points in image 2
            x, the flattened concatenationg of P, r2, and t2.
    Output: residuals, 4N x 1 vector, the difference between original and estimated projections
'''


def rodriguesResidual(K1, M1, p1, K2, p2, x):
    # Replace pass by your implementation
    P_flat = x[:-6]
    r = x[-6:-3].reshape(3,1)
    t = x[-3:].reshape(3,1)
    P = P_flat.reshape(-1,3)
    R = rodrigues(r)
    M2 = np.concatenate((R,t),axis=1)
    C1 = K1@M1
    C2 = K2@M2
    P_homo = np.concatenate((P,np.ones((P.shape[0],1))),axis=1).T
    assert(P_homo.shape == (4,P.shape[0]))
    p1_proj = C1@P_homo
    p1_proj = p1_proj/p1_proj[-1]
    p2_proj = C2@P_homo
    p2_proj = p2_proj/p2_proj[-1]
    p1_proj = p1_proj[:2,:].T
    p2_proj = p2_proj[:2,:].T
    residuals = np.concatenate([(p1-p1_proj).reshape([-1]), (p2-p2_proj).reshape([-1])]).reshape(-1,1)
    
    return residuals

'''
Q5.3 Bundle adjustment.
    Input:  K1, the intrinsics of camera 1
            M1, the extrinsics of camera 1
            p1, the 2D coordinates of points in image 1
            K2,  the intrinsics of camera 2
            M2_init, the initial extrinsics of camera 1
            p2, the 2D coordinates of points in image 2
            P_init, the initial 3D coordinates of points
    Output: M2, the optimized extrinsics of camera 1
            P2, the optimized 3D coordinates of points
'''

def wrapper(x, p2,K2,p1,M1,K1):
    result = rodriguesResidual(K1,M1,p1,K2,p2,x)
    return result.flatten()

def bundleAdjustment(K1, M1, p1, K2, M2_init, p2, P_init):
    # Replace pass by your implementation
    P = P_init.flatten()
    R = M2_init[:3,:3]
    t = M2_init[:,-1]
    r = invRodrigues(R)
    x = np.concatenate((P,r,t),axis=None)
    x_opt, _ = scipy.optimize.leastsq(wrapper,x, args = (p2, K2, p1, M1, K1))
    P_flat = x_opt[:-6]
    r_opt = x_opt[-6:-3].reshape(3,1)
    t_opt = x_opt[-3:].reshape(3,1)
    P_opt = P_flat.reshape(-1,3)
    # P_opt = P_opt/(P_opt[:,2].reshape(-1,1))
    R_opt = rodrigues(r_opt)
    M2_opt = np.concatenate((R_opt,t_opt),axis=1)
    return M2_opt, P_opt

if __name__ == '__main__':

    im1 = plt.imread('../data/im1.png')
    im2 = plt.imread('../data/im2.png')
    pts = np.load('../data/some_corresp.npz')
    pts1 = pts['pts1']
    pts2 = pts['pts2']

    '''
    # Q2.1

    F = eightpoint(pts1, pts2, max(im1.shape[0], im1.shape[1]))
    print(F)
    # helper.epipolarMatchGUI(im1,im2,F)
    # helper.displayEpipolarF(im1,im2,F)
    np.savez('q2_1.npz',F=F,M=max(im1.shape[0],im1.shape[1]))
    '''

    '''
    # Q2.2
    select = [ 99,  42,  24,   1,  82, 105,  22]
    
    F = sevenpoint(pts1[select,:], pts2[select,:], max(im1.shape[0],im1.shape[1]), ref=False)
    # print([np.linalg.matrix_rank(f) for f in F])
    print(F)
    # helper.displayEpipolarF(im1,im2,F[0])
    np.savez('q2_2.npz',F=F,M=max(im1.shape[0],im1.shape[1]), pts1=pts1[select,:], pts2=pts2[select,:])
    '''

    '''
    # Q3.1
    Karr = np.load('../data/intrinsics.npz')
    K1 = Karr['K1']
    K2 = Karr['K2']
    E = essentialMatrix(F,K1,K2)
    # print(E)
    '''

    '''
    M1 = np.hstack((np.eye(3),np.zeros((3,1))))
    M2arr = helper.camera2(E)
    # choose one C2
    M2 = M2arr[:,:,0]
    # multiply with intrinsics
    C1 = K1@M1
    C2 = K2@M2
    P,err = triangulate(C1,pts1,C2,pts2)
    print(err)
    '''

    '''
    # Q4.1
    F = eightpoint(pts1, pts2, max(im1.shape[0], im1.shape[1]))
    # helper.epipolarMatchGUI(im1, im2, F)
    # x = [123 , 232, 454, 417, 497, 318, 359, 259]
    # y = [207, 194, 392, 265, 229, 150, 280, 356]
    np.savez('q4_1.npz', F=F, pts1=pts1, pts2=pts2)
    '''

    