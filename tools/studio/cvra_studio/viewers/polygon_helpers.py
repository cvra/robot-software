import numpy as np

def rect(x, y, w, h):
    return [x + w/2, x - w/2, x - w/2, x + w/2], \
           [y + h/2, y + h/2, y - h/2, y - h/2]

def polygon(x, y, radius, samples, offset=0):
    return [x + radius * np.cos(2 * np.pi * i / samples + offset) for i in range(samples)],\
           [y + radius * np.sin(2 * np.pi * i / samples + offset) for i in range(samples)]

def close_polygon(polygon):
    x, y = polygon
    x.append(x[0])
    y.append(y[0])
    return x, y
