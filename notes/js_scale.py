#!/usr/bin/env python3
"""Script for generating GC joysticks scaling tables.

 --Jacques Gagnon <darthcloud@gmail.com>
"""

from collections import namedtuple
from scipy import stats
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import numpy as np
from controller_data import CTRL_DATA, Maximum

Joystick = namedtuple('Joystick', 'max_int max_float function')

def get_js_abs_maximum(data):
    """ Aggregate maximum values from the data set. """
    abs_max = {}
    for system, controllers in data.items():
        abs_max[system] = []
        for controller in controllers:
            for joystick, maximum in enumerate(controller):
                try:
                    abs_max[system][joystick]
                except IndexError:
                    abs_max[system].append(Maximum([], []))
                abs_max[system][joystick].origin.extend([abs(value) for point in maximum.origin for value in point if value != 0])
                abs_max[system][joystick].diagonal.extend([abs(value) for point in maximum.diagonal for value in point])
    return abs_max

def get_joysticks():
    """ Return average maximums and function for each joystick in the data set. """
    abs_max_sample = get_js_abs_maximum(CTRL_DATA)

    abs_max_float = {}
    abs_max_int = {}
    for system, controller in abs_max_sample.items():
        abs_max_float[system] = []
        abs_max_int[system] = []
        for joystick, maximum in enumerate(controller):
            origin_mean = stats.hmean(maximum.origin)
            diagonal_mean = stats.hmean(maximum.diagonal)
            abs_max_float[system].append(Maximum(origin_mean, diagonal_mean))
            abs_max_int[system].append(Maximum(int(round(origin_mean)), int(round(diagonal_mean))))

    joysticks = {}
    for system, controller in abs_max_int.items():
        joysticks[system] = []
        for joystick, maximum in enumerate(controller):
            print("{} origins max: {} diagonals max: {}".format(
                system + ("" if len(controller) == 1 else " Main" if joystick == 0 else " C"),
                maximum.origin, maximum.diagonal))
            plist = [0, maximum.diagonal, maximum.origin]
            joysticks[system].append(Joystick(maximum, abs_max_float[system][joystick], interp1d(plist, plist[::-1], bounds_error=False)))

    return joysticks

def get_gc_js_scale_tables(joysticks):
    """ Build the scaling table for GC joysticks in 1.7 fixed point format. """
    gc_scale = []
    for joystick in joysticks['GC']:
        gc_scale.append(interp1d([0, joystick.max_int.diagonal, joystick.max_int.origin],
                                 [joysticks['N64'][0].max_float.origin/joystick.max_float.origin,
                                  joysticks['N64'][0].max_float.diagonal/joystick.max_float.diagonal,
                                  joysticks['N64'][0].max_float.origin/joystick.max_float.origin],
                                 bounds_error=False, fill_value=joysticks['N64'][0].max_float.origin/joystick.max_float.origin))

    gc_scale_fp = []
    for function in gc_scale:
        gc_scale_fp.append([int(round(128 * function(abs(val)))) for j in (range(128), range(-128, 0)) for val in j])

    return gc_scale_fp

def main():
    """ Main """
    # Get functions and maximums for each joystick.
    js = get_joysticks()

    # Plot N64 controller maximum values vs unscaled GC controller values via Cube64 (dead zone + sign).
    x = np.linspace(0, 128, num=512)
    fig, ax = plt.subplots(figsize=(10, 10))
    fig.suptitle('N64 & GC Joysticks Maximum', fontsize=16)
    ax.set(xlim=[0, 128], ylim=[0, 128], aspect=1)
    for _, joysticks in js.items():
        for joystick in joysticks:
            ax.plot(x, joystick.function(x), '-')
    ax.legend(['N64', 'GC', 'GC C'], loc='upper right')
    fig.savefig('figure/n64_gc_joysticks_max.png', bbox_inches='tight')

    # Compute the GC joysticks scaling tables.
    scale_tables = get_gc_js_scale_tables(js)

    # Plot validation of the scaling tables.
    for joystick, table in enumerate(scale_tables):
        js_name = "Main" if joystick == 0 else "C"
        fig, ax = plt.subplots(figsize=(10, 10))
        fig.suptitle("GC {} Joystick Scaling".format(js_name), fontsize=16)
        ax.set(xlim=[0, 128], ylim=[0, 128], aspect=1)
        ax.plot(x, js['N64'][0].function(x), '-', label='N64')
        ax.plot(x, js['GC'][joystick].function(x), '-', label='GC')
        x_scale = [(table[val]/128 * val) for val in range(js['GC'][joystick].max_int.origin + 1)]
        y_scale = [(table[val]/128 * js['GC'][joystick].function(val)) for val in range(js['GC'][joystick].max_int.origin + 1)]
        ax.plot(x_scale, y_scale, 'r--', label='GC scale')
        ax.legend(loc='upper right')
        fig.savefig("figure/gc_{}_scaling.png".format(js_name.lower()), bbox_inches='tight')

if __name__ == "__main__":
    main()

### The End ###
