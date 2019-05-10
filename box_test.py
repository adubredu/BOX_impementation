import numpy as np

train = np.random.rand(10,15)
test = np.random.rand(1,15)
C = 0.6
n_samples = 5

(evaled_thetas,avgs,CIs) = GaussianBOX(train, test, n_samples,C)
