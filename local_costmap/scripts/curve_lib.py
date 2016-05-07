#!/usr/bin/env python
#
# http://devosaurus.blogspot.com/2013/10/exploring-b-splines-in-python.html

import numpy as np
import matplotlib.pyplot as plt
import time

def memoize(f):
    """ Memoization decorator for functions taking one or more arguments. """
    class memodict(dict):
        def __init__(self, f):
            self.f = f
        def __call__(self, *args):
            return self[args]
        def __missing__(self, key):
            ret = self[key] = self.f(*key)
            return ret        
    return memodict(f)

def C_factory(P, n=2, V_type="clamped"):
    """ Returns a b-spline curve C(t) configured with P, V_type and n.
    The knot vector will be created according to V_type
    
    Parameters
    ==========
    - P (list of D-tuples of reals) : List of de Boor points of dimension D.
    - n (int) : degree of the curve
    - V_type (str): name of the knit vector type to create.
    
    Returns
    =======
    A D-dimensionnal B-Spline Curve.
    """
    
    # TODO: check that p_len is ok with the degree and > 0
    m = len(P)    # the number of points in P    
    D = len(P[0]) # the dimension of a point (2D, 3D)
        
    # Create the knot vector
    V = make_knot_vector(n, m, V_type)
    # TODO: check the validity of the input knot vector.
    # TODO: create an initial Vector Point.
    
    #############################################################################
    # The following line will be detailed later.                                #
    # We create the highest degree basis spline function, aka. our entry point. #
    # Using the recursive formulation of b-splines, this b_n will call          #
    # lower degree basis_functions. b_n is a function.                          #
    #############################################################################
    b_n = basis_factory(n)
    
    @memoize
    def S(t, d):
        """ The b-spline funtion, as defined in eq. 3. """
        out = 0.
        for i in range(m): #: Iterate over 0-indexed point indices
            out += P[i][d]*b_n(t, i, V)
        return out
    
    def C(t):
        """ The b-spline curve, as defined in eq. 4. """
        out = [0.]*D           #: For each t we return a list of D coordinates
        for d in range(D):     #: Iterate over 0-indexed dimension indices
            out[d] = S(t,d)
        return out
    
    ####################################################################
    # "Enrich" the function with information about its "configuration" #
    ####################################################################  
    C.P = P                   #: The control polygone
    C.V = V                   #: The knot vector used by the function
    C.spline = S              #: The spline function.
    C.basis = b_n             #: The highest degree basis function. Useful to do some plotting.
    C.min = V[0]              #: The domain of definition of the function, lower bound for t
    C.max = V[-1]             #: The domain of definition of the function, upper bound for t
    C.endpoint = C.max!=V[-1] #: Is the upper bound included in the domain.
    return C

def make_knot_vector(n, m, style="clamped"):
    """
    Create knot vectors for the requested vector type.
    
    Parameters
    ==========
    - n (int) : degree of the bspline curve that will use this knot vector
    - m (int) : number of vertices in the control polygone
    - style (str) : type of knot vector to output
    
    Returns
    =======
    - A knot vector (tuple)
    """
    if not style == "clamped":
        raise NotImplementedError
        
    total_knots = m+n+2                           # length of the knot vector, this is exactly eq.6
    outer_knots = n+1                             # number of outer knots at each of the vector.
    inner_knots = total_knots - 2*(outer_knots)   # number of inner knots
    # Now we translate eq. 5:
    knots  = [0]*(outer_knots)
    knots += [i for i in range(1, inner_knots)]
    knots += [inner_knots]*(outer_knots)
    
    return tuple(knots) # We convert to a tuple. Tuples are hashable, required later for memoization

@memoize
def basis_factory(degree):
    """ Returns a basis_function for the given degree """
    if degree == 0:
        @memoize        
        def basis_function(t, i, knots):
            """The basis function for degree = 0 as per eq. 7"""
            t_this = knots[i]
            t_next = knots[i+1]
            out = 1. if (t>=t_this and t<t_next) else 0.         
            return out
        
    else:
        @memoize
        def basis_function(t, i, knots):
            """The basis function for degree > 0 as per eq. 8"""
            out = 0.
            t_this = knots[i]
            t_next = knots[i+1]
            t_precog  = knots[i+degree]
            t_horizon = knots[i+degree+1]            

            top = (t-t_this)
            bottom = (t_precog-t_this)
     
            if bottom != 0:
                out  = top/bottom * basis_factory(degree-1)(t, i, knots)
                
            top = (t_horizon-t)
            bottom = (t_horizon-t_next)
            if bottom != 0:
                out += top/bottom * basis_factory(degree-1)(t, i+1, knots)
         
            return out
        
    ####################################################################
    # "Enrich" the function with information about its "configuration" #
    ####################################################################         
    basis_function.lower = None if degree==0 else basis_factory(degree-1)
    basis_function.degree = degree
    return basis_function


def draw_bspline(C=None, P=None, n=None, V_type=None, endpoint_epsilon=0.00001):
    """Helper function to draw curves."""
    if P and n and V_type:
        C = C_factory(P, n, V_type)
    if C:
        # Use 2D or 3D
        is3d = True if len(C.P[0])==3 else False
        from mpl_toolkits.mplot3d import Axes3D

        # Regularly spaced samples
        sampling = [t for t in np.linspace(C.min, C.max, 100, endpoint=C.endpoint)]
        # Hack to sample close to the endpoint
        sampling.append(C.max - endpoint_epsilon)
        # Sample the curve!!!!
        curvepts = [ C(s) for s in sampling ]
        
        # Create a matplotlib figure
        fig = plt.figure()
        fig.set_figwidth(12)
        if is3d:
            fig.set_figheight(10)
            ax = fig.add_subplot(111, projection='3d')
        else:
            ax  = fig.add_subplot(111)
            
        # Draw the curve points
        ax.scatter( *zip(*curvepts), marker="o", c=sampling, cmap="jet", alpha=0.5 )
        # Draw the control cage.
        ax.plot(*zip(*C.P), alpha=0.3)
        # Draw the knots
        knotspos = [C(s) for s in C.V if s!= C.max]
        knotspos.append( C(C.max - endpoint_epsilon) )
        ax.scatter( *zip(*knotspos), marker="*", c=sampling, alpha=1, s=100 )
        
        # Here we annotate the knots with their values
        prev = None
        occurences = 1
        for i, curr in enumerate(C.V):
            if curr == C.max:
                kpos = C(curr-endpoint_epsilon)
            else:
                kpos = C(curr)
            if curr == prev:
                occurences += 1
            else:
                occurences = 1
            kpos[0] -= 0.3*occurences
            ax.text( *kpos, s="t="+str(curr), fontsize=12 )
            prev = curr

if __name__ == '__main__':

    # straight path:
    P2 = [( 0 , 0 ),( 0.850849609305 , -0.378822652946 ),( 1.25084960931 , -0.378822652946 ),( 1.64591739623 , -0.554718164325 ),( 2.1219753027 , -0.766672800154 ),( 2.67053118522 , -1.37590582837 ),( 3.65773110606 , -1.81543555126 ),( 4.19303559114 , -2.40995141164 ),( 5.18666731717 , -2.40735164768 )]
    P3 = [( 0 , 0 ),( 1.1416407865 , 0.0 ),( 1.87247715261 , -0.325389314461 ),( 2.72332676192 , -0.704211967407 ),( 4.04727543666 , -1.29367189528 ),( 5.94074442611 , -2.63221376384 )]

    # We decide to draw a quadratic curve
    n = 2
    # Next we define the control points of our curve
    P = [(3,-1), (2.5,3), (0, 1), (-2.5,3), (-3,-1)]
    # Create the knot vector
    # V = make_knot_vector(n, len(P), "clamped")
    # Create the Curve function
    # C = C_factory(P, V, n)
    C = C_factory(P2, n)

    # Regularly spaced samples
    sampling = [t for t in np.linspace(C.min, C.max, 100, endpoint=C.endpoint)]
    # Sample the curve!!!!
    curvepts = [ C(s) for s in sampling ]
    # Create a matplotlib figure
    fig = plt.figure()
    fig.set_figwidth(16)
    ax  = fig.add_subplot(111)
    # Draw the curve points
    ax.scatter( *zip(*curvepts), marker="o", c=sampling, cmap="jet", alpha=0.5 )
    # Draw the control cage.
    ax.plot(*zip(*P2), alpha=0.3)
    plt.show()

    # # Next we define the control points of our 2D curve
    # P = [(3 , 1), (2.5, 4), (0, 1), (-2.5, 4), (-3, 0), (-2.5, -4), (0, -1), (2.5, -4), (3, -1)]
    # # Create the a quadratic curve function
    # draw_bspline(P=P, n=2, V_type="clamped")

    # # Next we define the control points of our 3D curve
    # P3D = [(3, 1, 8), (2.5, 3, 7), (0, 1, 6), (-2.5, 3, 5), (-3, 0, 4), (-2.5, -3, 3), (0, -1, 2), (2.5, -3, 1), (3, -1, 0)]
    # # Create the a quadratic curve function
    # draw_bspline(P=P3D, n=2, V_type="clamped")
