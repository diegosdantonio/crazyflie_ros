import numpy as np
from scipy import optimize
from trajectory import traj_data


# function catenary
def f(c,l,x_bar):
    return (- l/2 + c * np.sinh(x_bar/(2*c)))  # only one real root at x = 1

# Rotation matrix
def Rotz(th):
    Rot = np.array([[np.cos(th), -np.sin(th), 0],
                  [np.sin(th), np.cos(th), 0],
                  [0, 0, 1]])
    return Rot

# Hat_map
def hat_map(aux):
    A = np.array([[0, -aux[2], aux[1]], [aux[2], 0, -aux[0]], [-aux[1], aux[0], 0]])
    return A


# function for compute the position of the robots
# here the minimum point change with the time and the rotation is fixed
# and increasing and decreasing the span when is close to one object.

def trajectoryB(t):

    # Minimum point change with the time

    dst_xCd = np.array([t/2.5-1., 0., 0.0], dtype='f')
    # dst_xCd = np.array([0.,0, 0.1], dtype='f')
    dst_dxCd = np.array([1./10, 0., 0.], dtype='f')
    dst_ddxCd = np.array([0., 0., 0.], dtype='f')

    dst_Omega = np.array([0, 0, 0], dtype='f')

    # the rotation is fixed
    dst_yaw = 90*np.pi/180
    # t/5

    dst_R = Rotz(dst_yaw)
    dst_dR = dst_R.dot(hat_map(dst_Omega))

    # after one time the span increase a decrease

    if t>np.pi/2 and t < np.pi:
        dst_x_bar_d = 0.3 + np.abs(0.5 * np.sin(2*t))
    else:
        dst_x_bar_d = 0.3

    dst_dx_bar_d = 0

    sol = optimize.bisect(f, 0.01, 5, args=(1.05, dst_x_bar_d))  #(f, x0=[0.15], args=(1.5, dst_x_bar_d), method='hybr', tol=1e-6)
    #print(sol)
    dst_c = np.absolute(sol)
    dst_dc = 0

    # Relative span

    dst_zABd = dst_c*(np.cosh(dst_x_bar_d/(2*dst_c)) - 1)

    # dst_dzABd = dst_dc*np.cosh(dst_x_bar_d/(2*dst_c) - 1)\
    #                      + (dst_dx_bar_d/2 - (dst_x_bar_d*dst_dc)/(2*dst_c))\
    #                      * np.sinh(dst_x_bar_d/(2*dst_c))

    #  xA, xB and zAB desired
    xAdv = np.array([-dst_x_bar_d/2, 0, dst_zABd], dtype='f')
    xBdv = np.array([dst_x_bar_d/2, 0, dst_zABd], dtype='f')

    # dxAdv = np.array([- dst_dx_bar_d / 2, 0, dst_dzABd], dtype='f')
    # dxBdv = np.array([dst_dx_bar_d / 2, 0, dst_dzABd], dtype='f')

    ## Desired positions to vectors

    dst_xAd = dst_xCd + dst_R.dot(xAdv)
    dst_xBd = dst_xCd + dst_R.dot(xBdv)

   # dst_dxAd = dst_dxCd + dst_dR.dot(xAdv) + dst_R.dot(dxAdv)
   # dst_dxBd = dst_dxCd + dst_dR.dot(xBdv) + dst_R.dot(dxBdv)

    return dst_xAd, dst_xBd, dst_yaw

