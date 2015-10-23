"""plot results"""

import IPython
import matplotlib.pyplot as plt
import numpy as np


def getAverage(dic, i1, i2, index):
    ag = np.array([dic[key][i1:i2] for key in sortedL[index:]])
    ag = np.mean(ag, axis=0)
    return ag

results = np.load('resultsTotal.npy')
itrs = np.load('itrTotal.npy')

results = results[1:]
itrs = itrs[1:]

itrsD = {len(itrs[i]): itrs[i] for i in range(len(itrs))}
resultsD = {len(results[i]): results[i] for i in range(len(results))}
sortedL = sorted(itrsD)

avgI = getAverage(itrsD, 0, sortedL[0], 0)
avgR = getAverage(resultsD, 0, sortedL[0], 0)
oldKey = sortedL[0]
i = 1
for key in sortedL[1:]:
    subAgI = getAverage(itrsD, oldKey, key, i)
    subAgR = getAverage(resultsD, oldKey, key, i)
    avgI = np.hstack((avgI, subAgI))
    avgR = np.vstack((avgR, subAgR))
    oldKey = key
    i += 1
    
avgR = avgR.T
avgR[0] = 1 - avgR[0]
avgCost = avgR[0] * 10 + avgR[1]
print avgCost

plt.figure(1)
plt.plot(avgI, avgCost,color='b',linewidth=5.0)
plt.ylabel('Gripper Closed')
plt.xlabel('States Labeled')

#plt.figure(2)
#plt.plot(avgI, avgR.T[1],color='r',linewidth=5.0)

plt.show()
