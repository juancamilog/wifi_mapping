#!/usr/bin/env python2
import numpy as np

class ap_simulator:
    def __init__(self, f=None, params=None):
        self.f = f
        self.params = params
        if f is not None:
          self.f = staticmethod(f)
        pass

    def get_values(self,x):
        if self.f is None:
            return 0
        else:
           return self.f(self.params, x)

    def set_function(self, f , params):
        self.f = staticmethod(f)
        self.params = params

def path_loss_dbm(params, x):
    x0 = params[0]
    P0 = params[1] 
    a = params[2]
    return P0 - 10.0*math.sqrt( np.power((x -  x0), a) )

def path_loss_w(params, x):
    return np.power(10.0,(path_loss_dbm(params, x))/10.0)

def rbf_kernel(params, x, add_noise=False):
    x0 = params[0]
    sigma_f = params[1]
    length_scale = params[2]
    return (sigma_f**2)*np.exp(-0.5*(np.multiply((x-x0)**2,length_scale)).sum())

def trained_gaussian_process(params, x):
    # the parameters correspond to the invese trained kernel matrix K^-1,
    # the matrix of training inputs X,
    # the vector of training outputs y,
    # the trained kernel parameters,
    # and a reference to the kernel function
    invK = params[0]
    X = params[1]
    y = params[2]
    kernel_params = params[3]
    kernel_func = params[4]

    # compute k(X,x)
    k = numpy.fromiter(kernel_func([col, kernel_params], x) for col in X.T, float, X.shape[1])

    # compute output
    return k*invK*y

