#!/usr/bin/env python
"""
Fit a curve to the [max speed vs turning radius] dynamics data.
This file is not to be imported.
It is just a test bed for getting a good fit.
"""
import matplotlib.pyplot as plt
import numpy as np
import sys
import scipy.optimize

# Speeds (control value).
speeds = [0.5, 1, 1.5, 1.7, 1.9, 2.3, 2, 3, 4, 0.9, 1.5, 1.7, 1.9, 2.3, 2.6, 1.3, 1.8, 2, 2.3, 2.5, 2.7, 2.9, 3, 3.2, 1.3, 1.7, 2.1, 2.2, 2.3, 2.5, 2.6, 5, 0.6, 0.6, 0.7, 1.3, 0.8, 0.9, 1.1, 1.5, 1.7, 1.9, 2.2, 2.4, 2.6, 2.7, 3.1, 5, 0.7, 0.9, 1.1, 1.4, 1.7, 2.1, 2.3, 2.5, 3, 1.3, 1.9, 2.1, 2.3, 2.8, 2.9, 3, 0.7, 1.8, 2.2, 2.8, 2.9, 3.2]

# Turning radii (meters).
turning_radii = [1.903501969, 1.903501969, 1.903501969, 1.919724719, 1.949239987, 2.068391976, 1.7, 2.114, 3.085, 1.585705803, 1.600918043, 1.620038764, 1.637831258, 1.676906886, 1.720991344, 1.382804848, 1.383547007, 1.401932341, 1.443226093, 1.492169146, 1.567983248, 1.690659198, 1.749530024, 1.896304199, 1.147981044, 1.152974535, 1.173409097, 1.188758504, 1.214716174, 1.264799595, 1.309199854, 4.5, 0.9895127606, 0.9670989729, 0.9398625226, 0.9322307594, 0.9232603762, 0.9437353174, 0.9478162712, 0.9330682429, 0.9357271147, 0.9562024396, 1.015881643, 1.102754575, 1.207924484, 1.288321243, 1.673112479, 4.5, 0.8241976124, 0.8193434995, 0.8291874776, 0.8370864859, 0.8496517115, 0.9484411631, 1.054831759, 1.109675213, 1.523525629, 0.822500608, 0.9024676287, 0.9470940479, 1.010415738, 1.356563907, 1.432012249, 1.514624846, 0.8115466822, 0.8242799969, 0.9398168302, 1.339302527, 1.436290477, 1.72163466]

assert(len(speeds) == len(turning_radii))

# Format data into a table.
data = np.column_stack((turning_radii, speeds))
# Sort the data by turning radius.
data = data[np.argsort(data[:,0])]
turning_radii = data[:,0]
speeds = data[:,1]

if False:
    # Show a scatter plot of the data.
    plt.scatter(turning_radii, speeds)
    plt.xlabel("Turning radius")
    plt.ylabel("Speed")
    plt.show()

if True:
    # Filter for the monotonically increasing (speed) portions of the data.
    increasing_indices = np.maximum.accumulate(data[:,1]) == data[:,1]
    data = data[increasing_indices]
    assert np.all(np.diff(data[:,1]) >= 0.)
    turning_radii = data[:,0]
    speeds = data[:,1]

if False:
    # Filter out the high datapoints.
    data = data[data[:,0] < 2.5]
    turning_radii = data[:,0]
    speeds = data[:,1]

if False:
    # Show a scatter plot of increasing data.
    plt.scatter(turning_radii, speeds)
    plt.plot(turning_radii, speeds)
    plt.xlabel("Turning radius")
    plt.ylabel("Speed")
    plt.show()

if False:
    # Fit a polynomial to the data.
    coeffs = np.polyfit(turning_radii, speeds, deg=2)
    poly = np.poly1d(coeffs)
    print poly
    xs = np.linspace(np.min(turning_radii), np.max(turning_radii))
    ys = np.polyval(poly, xs)
    plt.scatter(turning_radii, speeds)
    plt.plot(xs, ys)
    plt.show()

if False:
    # Fit an arbitrary curve to the data.
    def func(x, a, b, c):
        return a * np.exp(-b * x) + c

    opt_xs = data[:,0]
    opt_ys = data[:,1]
    popt, pcov = scipy.optimize.curve_fit(func, opt_xs, opt_ys)
    print popt

    xs = np.linspace(np.min(turning_radii)-.2, np.max(turning_radii)+.2, num=400)
    ys = func(xs, *popt)
    plt.scatter(turning_radii, speeds)
    plt.plot(xs, ys)
    plt.show()

# def max_speed_at_turning_radius(speed):
#     """Return the maximum achievable speed at a given turning radius."""
#     if speed < .81:
#         return .65
#     elif speed < 1.95:
#         return np.log(speed+.89) * 4 - 1.
#         # x = speed
#         # return -0.14*x*x + 1.38*x +1.151
#     else:
#         return speed * .7 + 1.9

def max_speed_at_turning_radius(speed):
    x = speed
    return -0.14*x*x + 1.38*x +1.151

if True:
    xs = np.linspace(np.min(turning_radii)-.2, np.max(turning_radii)+.2, num=400)
    ys = np.vectorize(max_speed_at_turning_radius)(xs)
    plt.scatter(turning_radii, speeds)
    plt.plot(xs, ys)
    plt.show()

def regression_test():
    """Test whether there has been a regression in this code."""
    return True

