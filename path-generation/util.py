from math import sqrt

eps = 7./3 - 4./3 - 1

def h(x):
    if x is 0:
        # I actually don't know what to return here
        # The derivative at 0 will never be that crucial
        return 0.001

    return sqrt(eps) * x

def diff(f,x):
    return (f(x+h(x)) - f(x-h(x)))/(2*h(x))
