import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from sklearn import mixture
import h5py

#n_samples = 300

# generate random sample, two components
#np.random.seed(0)

# generate spherical data centered on (20, 20)
#shifted_gaussian = np.random.randn(n_samples, 2) + np.array([20, 20])
#print('Shifted gaussian')
#print(shifted_gaussian)
# generate zero centered stretched Gaussian data
#C = np.array([[0., -0.7], [3.5, .7]])
#stretched_gaussian = np.dot(np.random.randn(n_samples, 2), C)
#print('Stretched Gaussian')
#print(stretched_gaussian)
# concatenate the two datasets into the final training set
#X_train = np.vstack([shifted_gaussian, stretched_gaussian])

#open the file
filename = '../h5 files/xy_fd_hd_grid10.h5'
hf = h5py.File(filename, 'r')
#navigate to necessary data and store in numpy arrays
x = hf.get('x')
x = np.array(x)
y = hf.get('y')
y = np.array(y)
ja_fd = hf.get('ja_fd')
ja_fd = np.array(ja_fd)
ja_hd = hf.get('ja_hd')
ja_hd = np.array(ja_hd)
lte_fd = hf.get('lte_fd')
lte_fd = np.array(lte_fd)
lte_hd = hf.get('lte_hd')
lte_hd = np.array(lte_hd)
dmp_fd = hf.get('dmp_fd')
dmp_fd = np.array(dmp_fd)
dmp_hd = hf.get('dmp_hd')
dmp_hd = np.array(dmp_hd)
#close out file
hf.close()

grid_size = np.shape(x)[0];

n_samples = 300

# generate random sample, two components
np.random.seed(0)

# generate spherical data centered on (20, 20)
#shifted_gaussian = np.random.randn(n_samples, 2) + np.array([20, 20])
#print('Shifted gaussian')
#print(shifted_gaussian)
# generate zero centered stretched Gaussian data
#C = np.array([[0., -0.7], [3.5, .7]])
#stretched_gaussian = np.dot(np.random.randn(n_samples, 2), C)
#print('Stretched Gaussian')
#print(stretched_gaussian)
# concatenate the two datasets into the final training set
#X_train = np.vstack([shifted_gaussian, stretched_gaussian])

# fit a Gaussian Mixture Model with two components
clf = mixture.GaussianMixture(n_components=2, covariance_type='full')
clf.fit(ja_fd[0:2, :])

# display predicted scores by the model as a contour plot
x = np.linspace(-20., 30.)
y = np.linspace(-20., 40.)
X, Y = np.meshgrid(x, y)
XX = np.array([X.ravel(), Y.ravel()]).T
Z = -clf.score_samples(XX)
Z = Z.reshape(X.shape)

CS = plt.contour(X, Y, Z, norm=LogNorm(vmin=1.0, vmax=1000.0),
                 levels=np.logspace(0, 3, 10))
CB = plt.colorbar(CS, shrink=0.8, extend='both')
plt.scatter(X_train[:, 0], X_train[:, 1], .8)

plt.title('Negative log-likelihood predicted by a GMM')
plt.axis('tight')
plt.show()