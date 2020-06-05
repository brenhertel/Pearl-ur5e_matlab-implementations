import mlfd
import numpy as np
import matplotlib.pyplot as plt
import similaritymeasures

shape = (5, 2)

#arr1 = np.zeros(shape)
#arr2 = np.ones(shape)

#x = np.linspace(1, 10, 10000)
#
#y1 = np.abs(np.cos(x))
#y2 = np.abs(np.sin(x))
#
#temp1 = np.hstack((y1, np.flipud(y2)))
#temp2 = np.hstack((y2, np.flipud(y1)))
#
#x2 = np.linspace(10, 1, 10000)
#
#x = np.hstack((x, x2))
#
#y1 = temp1
#y2 = temp2

x1 = np.linspace(-5, 10, 100)
y1 = np.power(x1, 2)
y2 = np.linspace(-5, 10, 100)
x2 = np.power(y2, 2)

#traj1 = np.hstack((np.transpose(x), np.transpose(y1)))
#traj2 = np.hstack((np.transpose(x), np.transpose(y2)))

traj1 = np.hstack((np.reshape(x1, (len(x1), 1)), (np.reshape(y1, (len(y1), 1)))))
#x = x + 0.5
#x[5] = x[5] - 2
traj2 = np.hstack((np.reshape(x2, (len(x2), 1)), (np.reshape(y2, (len(y2), 1)))))
print(traj1)

print(traj2)

#ans = mlfd.area_evaluation(traj1, traj2)

#print(ans)

ans2 = similaritymeasures.curve_length_measure(traj1, traj2)

print(ans2)

plt.plot(traj1[:, 0], traj1[:, 1], 'r')
plt.plot(traj2[:, 0], traj2[:, 1], 'g--')
plt.show()