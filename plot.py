import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

TABLE_SIZE = {'x': 3000, 'y': 2000}
ROBOT_SIZE = 245
ARM_LENGTHS = (100, 98)
ARM_OFFSET = {'x': -100, 'y': 0, 'a': np.pi}

def discrete_circle(center, radius, samples, offset=0):
    return [center[0] + radius * np.cos(2 * np.pi * i / samples + offset) for i in range(samples)],\
           [center[1] + radius * np.sin(2 * np.pi * i / samples + offset) for i in range(samples)]

def rectangle(a, b):
    return [a[0], b[0], b[0], a[0]], [a[1], a[1], b[1], b[1]]

def rot(angle):
    return np.array([[np.cos(angle), -np.sin(angle)],
                     [np.sin(angle), np.cos(angle)]])

def lever(length, angle):
    return length * np.array([np.cos(angle), np.sin(angle)])

def draw_link(ax, x0, x1, thickness=5):
    ax.plot([x0[0], x1[0]], [x0[1], x1[1]], linewidth=thickness)

def draw_polygon(ax, polygon, fill=True):
    x, y = polygon
    x.append(x[0])
    y.append(y[0])
    if fill: ax.fill(x, y)
    else:    ax.plot(x, y)

def draw_base(ax, pose, robot_size):
    draw_polygon(ax, discrete_circle((pose['x'], pose['y']), robot_size / 2, 6, pose['a']))

def draw_arm(ax, pose, arm, arm_offset):
    shoulder = np.array([pose['x'], pose['y']]) + rot(pose['a']) @ [arm_offset['x'], arm_offset['y']]
    elbow = shoulder + lever(ARM_LENGTHS[0], pose['a'] + arm_offset['a'] + arm['a'])
    hand = elbow + lever(ARM_LENGTHS[1], pose['a'] + arm_offset['a'] + arm['a'] + arm['b'])

    draw_link(ax, shoulder, elbow)
    draw_link(ax, elbow, hand)

def draw_robot(ax, pose={'x': 0, 'y': 0, 'a': 0}, arm={'a': 0, 'b': 0}):
    draw_base(ax, pose, ROBOT_SIZE)
    draw_arm(ax, pose, arm, ARM_OFFSET)

def draw_table(margin=200):
    fig, ax = plt.subplots()
    ax.set_xlim((- margin, TABLE_SIZE['x'] + margin))
    ax.set_ylim((- margin, TABLE_SIZE['y'] + margin))
    draw_polygon(ax, rectangle((0, 0), (TABLE_SIZE['x'], TABLE_SIZE['y'])), fill=False)
    return fig, ax

fig, ax = draw_table()
draw_robot(ax, {'x': 1000, 'y': 1000, 'a': np.pi/2}, {'a': np.pi/4, 'b': -np.pi/2})
fig.savefig("test.png")
