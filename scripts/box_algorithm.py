from openravepy import *
import numpy as np, time
from math import fabs, sqrt
import pdb
import pickle
from scipy.stats import t 
from math import sqrt



def GaussianBOX(w_n,C,k,Theta,D):
	mu = np.mean(D)
	cov = np.cov(D)


	for i in range(0,k):
		theta = np.


























































'''
def GaussianBox(train,test,n_samples,C):
	test = np.array(test); train = np.array(train)

	n_thetas = np.shape(test)
	n_row, n_col = np.shape(train)

	data_mu = np.mean(train)
	data_cov = np.cov(train)

	evaled_thetas = []
	ts = t.ppf([C],n_row-1)

	print(ts)
	CI = np.transpose(data_mu) + ts*(1/sqrt(n_row))*sqrt(np.diag(data_cov))
	# print CI
	t_idx = np.maximum

train = np.ones((4,8))
test = np.ones(8)
k = 4


GaussianBox(train=train, test=test,n_samples=k, C=1)
'''