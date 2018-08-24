# path following 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

#Initial parameter values
x_0         = 0.0
y_0         = 5.0
theta_0     = 1
v           = 5
omega       = 0

#Initial robot parameters
p = np.array([x_0, y_0])
theta      = theta_0

#Line vectors
r = np.array([0, 0])
q = np.array([1, 1])
#Ensure q is unit vector
norm = np.linalg.norm(q) * q
q = 1.0 / norm
#Total number of iterations and storage of x and y
ITERATIONS     = 1000
delta_t     = 0.01
x_total = np.zeros((ITERATIONS, 1))#[0 for i in range(0, ITERATIONS + 1)]
y_total = np.zeros((ITERATIONS, 1))#[0 for i in range(0, ITERATIONS + 1)]

#Governing parameters of the controller
K_theta          = 1
K_path      = 1
theta_inf     = np.pi/2

## Controller
def controller(theta, theta_d):
    #Update our angle to the desired course, limited by K_theta
    omega = K_theta * (theta_d - theta)
    
    return omega

## Path Follow
def PathFollow(x, y, theta):
    #First compute theta_q
    theta_q = np.arctan2(q[1], q[0])
    #Ensure we turn in an efficient direction
    if theta_q - theta < -np.pi:
       theta_q = theta_q + 2 * np.pi
    if theta_q - theta > np.pi:
        theta_q = theta_q - 2 * np.pi

    #Compute the lateral error to the path
    e_y = -np.sin(theta_q) * (x - r[0]) + np.cos(theta_q) * (y - r[1])
    #Now compute the desired course
    theta_d = theta_q - theta_inf * (2 / np.pi) * np.arctan(K_path * e_y)

    return theta_d

def finitedifference(v, w, x, y, theta):
    x = delta_t * v * np.cos(theta) + x
    y = delta_t * v * np.sin(theta) + y
    theta = delta_t * w + theta
    
    return x, y, theta

## main Loop
for i in range(ITERATIONS):
    x_total[i] = p[0]
    y_total[i] = p[1]
    
    theta_d = PathFollow(p[0], p[1], theta)
    omega = controller(theta, theta_d)
    p[0], p[1], theta = finitedifference(v, omega, p[0], p[1], theta)


x_total[i] = p[0]
y_total[i] = p[1]

#Try to do an animation
fig = plt.figure()
ax = plt.axes(xlim = (0, 2), ylim = (-2, 2))
line, = ax.plot([], [], lw = 2)

def init():
    line.set_data([], [])
    return line,
def animate(i):
    x = x_total[:i]
    y = y_total[:i]
    line.set_data(x, y)
    return line,

q = q * norm
plt.xlim(min(x_total) - 1, max(x_total) + 1)
plt.ylim(min(y_total) - 1, max(y_total) + 1)
plt.plot([r[0], q[0] * max(x_total)], [r[1], q[1] * max(y_total)])
anim = animation.FuncAnimation(fig, animate, init_func = init, frames = len(x_total), interval = 10, blit = True)
plt.show()

plt.figure()
plt.plot(x_total, y_total)
plt.plot([r[0], q[0] * max(x_total)], [r[1], q[1] * max(y_total)])
plt.show(False)
