from openravepy import *
import numpy as np, time
from math import fabs, sqrt
import pdb
import pickle
from scipy.stats import t 
from math import sqrt



def GaussianBox(train,test,n_samples,C):
	test = np.array(test); train = np.array(train)

	n_thetas = np.shape(test)[1]
	n_row, n_col = np.shape(train)
	data_mu = np.mean(train,axis=1)
	data_cov = np.cov(train)
	evaled_thetas = []
	ts = t.ppf([C],n_row-1)[0]

	CI = np.transpose(data_mu) + ts*(1/sqrt(n_row))*np.sqrt(np.diag(data_cov))
	t_idx = np.argmax(CI)
	evaled_thetas.append(t_idx)
	evaled_theta = t_idx

	t_shape = train.shape
	remain_cols_idxs = range(0,t_shape[1])
	intersect = np.intersect1d(remain_cols_idxs,evaled_thetas)

	remain_cols_idxs = np.delete(remain_cols_idxs, intersect,None)

	selected_cols = train[evaled_thetas,:]
	remain_cols = train[:,remain_cols_idxs]
	train_mean_vals= np.mean(train,axis=1)

	n_thetas = np.shape(train)[0]
	avgs = np.zeros((n_samples,n_thetas))
	CIs = 0
	avgs[0,:] = data_mu
	evaled_thetas = np.array(evaled_thetas)


	for k in range(1,n_samples):
		dd = data_cov[evaled_theta]; dd = dd+np.eye(np.shape(dd)[0], np.shape(dd)[0])
		DD = data_cov
		dD = DD; dD = np.transpose(dD[evaled_theta,:])
		data_mu = data_mu + np.transpose(dD * np.linalg.inv(dd) * np.transpose(test[evaled_theta] - 
			data_mu[evaled_theta]))
		data_conv = DD - dD * np.linalg.inv(dd)*np.transpose(dD)

		CI = np.transpose(data_mu) + ts*(1/sqrt(n_row))*np.sqrt(np.diag(data_cov))
		CI = np.mean(CI,axis=1)
		# print len(CI)
		CI[evaled_thetas] = test[evaled_thetas]
		# avgs[k,:] = data_mu
		# print(np.shape(avgs))

		t_idx_list = (-CI).argsort()
		# print t_idx_list
		t_idx = t_idx_list[0]; idx = 0
		# print t_idx

		while (t_idx in evaled_thetas):
			# print (evaled_thetas,t_idx)
			idx = idx+1
			# if idx < len(t_idx_list):
			t_idx = t_idx_list[idx]

		evaled_thetas = np.append(evaled_thetas,t_idx)
		# print t_idx
		# print type(evaled_thetas)
		intersect = np.intersect1d(remain_cols_idxs,evaled_thetas)
		remain_cols_idxs = np.delete(remain_cols_idxs, intersect,None)
		selected_cols = train[evaled_thetas,:]
		remain_cols = train[remain_cols_idxs,:]
		evaled_theta = t_idx
		# print evaled_thetas
	# print t_idx_list
	return evaled_thetas






train = np.random.rand(15,10)
test = np.random.rand(15,1)
k = 5


print GaussianBox(train=train, test=test,n_samples=k, C=0.5)
