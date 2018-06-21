#!/usr/bin/env python3
"""Script for generating GC joysticks response curve tables.

 --Jacques Gagnon <darthcloud@gmail.com>
"""

from collections import namedtuple
from scipy import stats
from scipy.interpolate import interp1d
from scipy.special import betainc
import matplotlib.pyplot as plt
import numpy as np

def sigmoid(k, t):
    """Source: https://dinodini.wordpress.com/2010/04/05/normalized-tunable-sigmoid-functions/ """
    return (k*abs(t))/(k-abs(t)+1)

def sigmoid_unbond(k, t):
    if (t > 1):
        return 1;
    if (t < -1):
        return -1;
    if (t < 0):
        return -sigmoid(k, t)
    else:
        return sigmoid(k, t)

def scurve(k, t):
    if (t > 1):
        return 1;
    if (t < -1):
        return -1;
    if (t < 0):
        return -betainc(k, k, abs(t))
    else:
        return betainc(k, k, t)

def get_gc_js_curve_tables(joysticks):
    """ Build the curve table for GC joysticks. """
    k = 0.5 * np.exp(3.5) / 7
    gc_curves = []
    for joystick in joysticks['GC']:
        gc_curves.append([int(round(sigmoid_unbond(-1.2, val/joystick.max_int.origin)*joystick.max_int.origin)) for j in (range(128), range(-128, 0)) for val in j])
    for joystick in joysticks['GC']:
        gc_curves.append([int(round(sigmoid_unbond(0.2, val/joystick.max_int.origin)*joystick.max_int.origin)) for j in (range(128), range(-128, 0)) for val in j])
    for joystick in joysticks['GC']:
        gc_curves.append([int(round(sigmoid_unbond(0.1, val/joystick.max_int.origin)*joystick.max_int.origin)) for j in (range(128), range(-128, 0)) for val in j])
    for joystick in joysticks['GC']:
        gc_curves.append([int(round(scurve(k, val/joystick.max_int.origin)*joystick.max_int.origin)) for j in (range(128), range(-128, 0)) for val in j])

    return gc_curves

def main():
    """ Main """
    x = np.linspace(0, 1, num=400)
    k = 0.5 * np.exp(3.5) / 7
    fig, ax = plt.subplots(figsize=(10, 10))
    fig.suptitle('Joystick Response Curves', fontsize=16)
    ax.set(xlim=[0, 1], ylim=[0, 1], aspect=1)
    ax.plot(x, x, '-')
    ax.plot(x, sigmoid(-1.2, x), '-')
    ax.plot(x, sigmoid(0.2, x), '-')
    ax.plot(x, sigmoid(0.1, x), '-')
    ax.plot(x, betainc(k, k, x), '-')
    ax.legend(['Linear', 'Agressive', 'Relaxed', 'Wide', 'S Curve'], loc='lower right')
    fig.savefig('figure/js_response_curves.png', bbox_inches='tight')

if __name__ == "__main__":
    main()

### The End ###
