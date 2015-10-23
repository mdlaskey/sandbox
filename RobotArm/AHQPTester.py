import random
import numpy as np
import IPython
import cPickle as pickle 
from numpy import linalg as LA
from sklearn import svm 
from sklearn import preprocessing  
from sklearn import linear_model
from sklearn import metrics 
from scipy.sparse import csr_matrix
from scipy.sparse import vstack
from scipy.spatial.distance import pdist,squareform
from cvxopt import matrix, solvers
from AHQP import AHQP
import matplotlib.pyplot as plt
from sklearn.neighbors import KernelDensity

import matplotlib.pyplot as plt
import matplotlib.font_manager

import time
from sklearn import preprocessing  

def test_sparse_implementations(ahqp, data, labels):
	print "Original"
	start = time.time()
	a = ahqp.assembleKernel(data,labels)
	end = time.time()
	print end - start

	print "Implementation 1"
	start = time.time()
	b = ahqp.assembleKernelSparse1(data,labels)
	end = time.time()
	print end - start

	print "Implementation 2"
	start = time.time()
	c = ahqp.assembleKernelSparse2(data,labels)
	end = time.time()
	print end - start

	print a, b, c

DIM = 22
NUM_SAMPLES = 6000



# data = pickle.load(open('states.p','rb'))

#Sample from a GMM 
mean_1 = np.zeros(DIM)#+np.array([-2,2])
mean_2 = np.zeros(DIM)#+np.array([2,-2])
mean_3 = np.zeros(DIM)#+np.array([2,2])

cov = np.eye(DIM)*0.1
cov_p = np.eye(DIM)

data_1 = np.random.multivariate_normal(mean_1,cov,NUM_SAMPLES)
data_2 = np.random.multivariate_normal(mean_2,cov,NUM_SAMPLES)
data_3 = np.random.multivariate_normal(mean_3,cov,NUM_SAMPLES)
data_idx = np.random.multinomial(NUM_SAMPLES,[0.3,0.2,0.5])
data = np.zeros([NUM_SAMPLES,DIM])

data[0:0.3*NUM_SAMPLES,:] = data_1[0:0.3*NUM_SAMPLES,:]
o = 0.3*NUM_SAMPLES -1 
data[o:o+0.2*NUM_SAMPLES,:] = data_2[0:0.2*NUM_SAMPLES,:]
o = 0.3*NUM_SAMPLES -1 + 0.2*NUM_SAMPLES-1
data[o:o+0.5*NUM_SAMPLES+2,:] = data_3[0:0.5*NUM_SAMPLES+2,:]

labels = np.zeros(NUM_SAMPLES)+1.0

# data = data[:,1:3]
# DIM = data.shape[0]
# labels = np.zeros((DIM,1))+1.0
# #IPython.embed()
for i in range(data.shape[0]):
	if((data[i,1]>1.75 and data[i,1] < 2.25 and data[i,0] >1.75 and data[i,0]<2.25)):
		labels[i] = -1.0
	if((data[i,1]<-1.75 and data[i,1] > -2.25 and data[i,0] >1.75 and data[i,0]<2.25)):
		labels[i] = -1.0
ahqp= AHQP()

ahqp.assembleKernel(data,labels)

weights = ahqp.solveQP()

IPython.embed()
# Learn a frontier for outlier detection with several classifiers
#xx1, yy1 = np.meshgrid(np.linspace(1400,0, 50), np.linspace(1400, 0, 50))
xx1, yy1 = np.meshgrid(np.linspace(-4,4), np.linspace(-4, 4))

plt.figure(1)


test = np.c_[xx1.ravel(), yy1.ravel()]
Z1 = np.zeros((test.shape[0],1))
for i in range(test.shape[0]):
	#point = scaler.transform(test[i,:])
	point = test[i,:]
	Z1[i] = ahqp.predict(point)

# clf.fit(X1)
# Z1 = clf.decision_function(np.c_[xx1.ravel(), yy1.ravel()])
Z1 = Z1.reshape(xx1.shape)

plt.figure(1)  # two clusters

plt.contour(
    xx1, yy1, Z1, levels=[0], linewidths=5, colors='g')



kde = KernelDensity(bandwidth=1, kernel='gaussian')
kde.fit(data)

# evaluate only on the land: -9999 indicates ocean

Z = np.exp(kde.score_samples(np.c_[xx1.ravel(), yy1.ravel()]))
Z = Z.reshape(xx1.shape)

# plot contours of the density
levels = np.linspace(0, Z.max(), 25)
plt.contourf(xx1, yy1, Z, levels=levels, cmap=plt.cm.Blues)


# Plot the results (= shape of the data points cloud)
for i in range(data.shape[0]):
	if((data[i,1]>1.75 and data[i,1] < 2.25 and data[i,0] >1.75 and data[i,0]<2.25)):
		labels[i] = -1.0
	if((data[i,1]<-1.75 and data[i,1] > -2.25 and data[i,0] >1.75 and data[i,0]<2.25)):
		labels[i] = -1.0



for i in range(NUM_SAMPLES):
	#if(labels[i] == -1):
	if(labels[i] == -1):
		plt.scatter(data[i, 0], data[i, 1], color='red',s=10)
	else:
		plt.scatter(data[i,0],data[i,1],color = 'black',s=10)


plt.xlim((xx1.min(), xx1.max()))
plt.ylim((yy1.min(), yy1.max()))
font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 30}

matplotlib.rc('font', **font)

plt.ylabel("Y Position")
plt.xlabel("X Position")

plt.show()


#IPython.embed()
