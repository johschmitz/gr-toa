import numpy as np

def chan_3rx(pos_rx, d):
    r21 = d[0]
    r31 = d[1]

    # Set receivers position
    print("------------------")
    print(pos_rx)
    x1 = pos_rx[0][0]
    y1 = pos_rx[0][1]
    x2 = pos_rx[1][0]
    y2 = pos_rx[1][1]
    x3 = pos_rx[2][0]
    y3 = pos_rx[2][1]
    
    center_triangle = ((x1+x2+x3)/3,(y1+y2+y3)/3)

    # Parameters of equation (5) from the reference paper
    K1 = x1**2 + y1**2
    K2 = x2**2 + y2**2
    K3 = x3**2 + y3**2

    # Define position differences
    x21 = x2 - x1
    x31 = x3 - x1
    y21 = y2 - y1
    y31 = y3 - y1
    rmax21 = np.sqrt(x21**2+y21**2)
    rmax31 = np.sqrt(x31**2+y31**2)
    # If a TDOA value that leads to distances greater than the real distance between 2 receivers occurs, clip it to maximum value.
    if r21 > rmax21:
        r21 = rmax21
        print("Warning: invalid TDOA between r2 & r1. Set to greatest value possible.")
    if r21 < -rmax21:
        r21 = -(rmax21)
        print("Warning: invalid TDOA between r2 & r1. Set to greatest value possible.")
    if r31 > rmax31:
        r31 = rmax31 
        print("Warning: invalid TDOA between r3 & r1. Set to greatest value possible.")
    if r31 < -rmax31:
        r31 = -(rmax31 ) 
        print("Warning: invalid TDOA between r3 & r1. Set to greatest value possible.")
        
    # Solve equations system
    A = (y21*r31-y31*r21)/(y31*x21-y21*x31)
    B = (y21*(0.5*(pow(r31,2)-K3+K1))-y31*(0.5*(pow(r21,2)-K2+K1)))/(y31*x21-y21*x31)
    C = (x31*r21-x21*r31)/(y31*x21-y21*x31)
    D = (x31*(0.5*(pow(r21,2)-K2+K1))-x21*(0.5*(pow(r31,2)-K3+K1)))/(y31*x21-y21*x31)

    alpha = pow(A,2)+pow(C,2)-1
    beta = 2*(A*B+C*D-A*x1-C*y1)
    gamma = pow(x1,2)+pow(y1,2)-2*x1*B-2*y1*D+pow(B,2)+pow(D,2)

    r1 = [(-beta+np.sqrt(pow(beta,2)-4*alpha*gamma+0j))/(2*alpha),(-beta-np.sqrt(pow(beta,2)-4*alpha*gamma+0j))/(2*alpha)]
    valid_roots = []
    for root in r1:
        if root >= 0:
            valid_roots.append(root)
    valid_roots = np.array(valid_roots)
    if 0 < len(valid_roots):
        xy = np.squeeze(np.real((valid_roots*A+B,valid_roots*C+D)).T)
        # Calculate position with the solution in the roi
        if xy.ndim > 1:
            # Choose solution closer to center
            if np.linalg.norm(xy[0]-center_triangle) < np.linalg.norm(xy[1]-center_triangle):
                xy = xy[0]
            else:
                xy = xy[1]
        return xy
    else:
        return center_triangle


# Solution for 4 or more receivers with source in near field
def chan_tdoa(pos, d, Q):
    pos = np.array(pos)
    d = np.array(d)
    print(pos)
    M,D = pos.shape # number of receiver stations M and dimensions D
    K = np.zeros((M,1))

    for i in range(M):
        K[i] = pos[i,0]**2 + pos[i,1]**2
    r_i1 = np.zeros((M-1,1)) # range difference in reference to receiver 1
    x_i1 = np.zeros((M-1,1))
    y_i1 = np.zeros((M-1,1))
    for i in range(M-1):
        r_i1[i] = d[i]
        x_i1[i] = pos[i+1,0] - pos[0,0]
        y_i1[i] = pos[i+1,1] - pos[0,1]
        
        r_i1_max = np.sqrt(x_i1[i]**2+y_i1[i]**2)
        if r_i1[i] > r_i1_max:
            r_i1[i] = r_i1_max
            print("Warning: invalid TDOA")
        elif r_i1[i] < -r_i1_max:
            r_i1[i] = - r_i1_max
            print("Warning: invalid TDOA")

    # ############################ First step (14b) ############################
    # For near-field source use (14b) first to give an approximation of B
    h = 0.5*(r_i1**2 - K[1:] + K[0]) # (11)
    G = -np.concatenate((x_i1, y_i1, r_i1),1) # (11)
    z1 = chan14b(h, G, Q) # first position estimate
    # ########################## Second step (14a) ############################
    cov_z, z2 = chan14a(h, G, Q, pos, z1)
    # ########################### Third step (22a) ############################
    z3 = chan22a(cov_z, pos, z2)
    xy = (z3[0].item(0,0),z3[1].item(0,0))
    return xy


def chan14b(h, G, Q):
    '''
    Be adviced!: For use of chan14b() the number of TDoA-receivers should be at
                 least D+2, with D the number of dimensions to be estimated for
                 the TX position (else risk error due to singular matrix).
    '''
    h = np.asmatrix(h)
    G = np.asmatrix(G)
    Q_inv = np.asmatrix(np.linalg.pinv(Q))
    return np.linalg.pinv(np.transpose(G)*Q_inv*G)*np.transpose(G)*Q_inv*h # (14b)


def chan14a(h, G, Q, pos, est):
    '''
    pos: RX positions
    est:  an estimate of the TX position
    Q:   covariance matrix of TDoA or DoA measurements
    Returns also cov_z from (17) which is needed in (21)
    '''
    M,D = pos.shape
    h = np.asmatrix(h)
    G = np.asmatrix(G)
    Q = np.asmatrix(Q)
    r_i0 = np.zeros((M-1,1)) # 'true' distances
    for i in range(M-1):
        r_i0[i] = np.sqrt( (pos[i+1,0]-est[0])**2 + (pos[i+1,1]-est[1])**2 )
    B = np.asmatrix( np.diag( np.transpose(r_i0)[0] ) ) # (12)
    Psi_inv = np.linalg.pinv( B*Q*B )
    cov_z = np.linalg.pinv(np.transpose(G)*Psi_inv*G) # (17)
    new_est = cov_z*np.transpose(G)*Psi_inv*h # (14a)
    # gives same position result as (14b) always for TDoA-only case
    return cov_z, new_est


def chan22a(cov_z, pos, est):
    '''
    s. chan14a()
    cov_z needed for (21)
    '''

    M,D = pos.shape
    xy_1 = np.reshape( np.concatenate((pos[0], np.asarray([0]))), (D+1,1) )
    zd = est - np.asmatrix( xy_1 ) # (19)
    h_hat = np.asmatrix( np.square(zd) ) # (19)
    G_hat = np.asmatrix( np.vstack( (np.eye(2), np.ones(2)) ) ) # (19)
    B_hat = np.asmatrix(np.diag( np.transpose(np.asarray(zd))[0] )) # (21)
    Psi_hat_inv = np.linalg.pinv( 4*B_hat*cov_z*B_hat ) # (21)
    z_hat = np.linalg.pinv(np.transpose(G_hat)*Psi_hat_inv*G_hat)*np.transpose(G_hat)*Psi_hat_inv*h_hat # (22a)
    # for the following sign correction the first estimate has to be good enough!
    new_est = np.multiply( np.sqrt( np.abs(z_hat)), np.sign(zd[:2])) + np.reshape(pos[0], (D,1)) # (24)
    return new_est


def locate(pos_rx, tdoas):
    if len(pos_rx) == 3:
        return list(chan_3rx(pos_rx, tdoas))
    else:
        # see chan, ho: A simple and efficient estimator for hyperbolic location
        # first construct covariance matrix
        Q_shape = 0.5 *np.ones(shape=(len(pos_rx)-1,len(pos_rx)-1)) + 0.5 * np.eye(len(pos_rx)-1)
        # scale with noise power
        P_noise = 1
        Q = P_noise*Q_shape
        return list(chan_tdoa(pos_rx, tdoas, Q))