"""plot results"""

import IPython
import matplotlib.pyplot as plt
import numpy as np


def getAverage(dic, i1, i2, index, lst):
    ag = np.array([dic[key][i1:i2] for key in lst[index:]])
    #print ag
    ag = np.mean(ag, axis=0)
    return ag

def transformList(lst, dic):
    newList = []
    size = lst[-1]
    for e in dic.keys():
        new = np.zeros(shape=dic[size].shape)
        for j in range(len(new)):
            new[j][0] = 1
        #print new
        for i in range(len(dic[e])):
            new[i] = dic[e][i]
        newList.append(new)
    return newList


def getCost(lst):
    a = np.asarray(lst).T
    a[0] = 1 - a[0]
    a = a[0] * 10 + a[1]
    return a


resultsD = np.load('resultsTotalD.npy')
itrsD = np.load('itrTotalD.npy')
resultsS = np.load('resultsTotalSHIV.npy')
itrsS = np.load('itrTotalSHIV.npy')

resultsS = resultsS[1:]
itrsS = itrsS[1:]

itrsDD = {len(itrsD[i]): itrsD[i] for i in range(len(itrsD))}
resultsDD = {len(resultsD[i]): resultsD[i] for i in range(len(resultsD))}
sortedLD = sorted(itrsDD)

itrsSD = {len(itrsS[i]): itrsS[i] for i in range(len(itrsS))}
resultsSD = {len(resultsS[i]): resultsS[i] for i in range(len(resultsS))}
sortedLS = sorted(itrsSD)

avgID = getAverage(itrsDD, 0, sortedLD[0], 0, sortedLD)
#avgRD = getAverage(resultsDD, 0, sortedLD[0], 0, sortedLD)
oldKeyD = sortedLD[0]
i = 1
for key in sortedLD[1:]:
    subAgI = getAverage(itrsDD, oldKeyD, key, i, sortedLD)
    #subAgR = getAverage(resultsDD, oldKeyD, key, i, sortedLD)
    avgID = np.hstack((avgID, subAgI))
    #avgRD = np.vstack((avgRD, subAgR))
    oldKeyD = key
    i += 1

avgIS = getAverage(itrsSD, 0, sortedLS[0], 0, sortedLS)
#avgRS = getAverage(resultsSD, 0, sortedLS[0], 0, sortedLS)
oldKeyS = sortedLS[0]
i = 1
for key in sortedLS[1:]:
    subAgI = getAverage(itrsSD, oldKeyS, key, i, sortedLS)
    #subAgR = getAverage(resultsSD, oldKeyS, key, i, sortedLS)
    avgIS = np.hstack((avgIS, subAgI))
    #avgRS = np.vstack((avgRS, subAgR))
    oldKeyS = key
    i += 1

#lstIS = transformList(sortedLS, itrsSD)
lstRS = transformList(sortedLS, resultsSD)

#lstID = transformList(sortedLD, itrsDD)
lstRD = transformList(sortedLD, resultsDD)

#avgIS = np.mean(np.asarray(lstIS), axis=0)
avgRS = np.mean(np.asarray(lstRS), axis=0)
stdRS = np.std(np.asarray(lstRS), axis=0)
#avgID = np.mean(np.asarray(lstID), axis=0)
avgRD = np.mean(np.asarray(lstRD), axis=0)
stdRD = np.std(np.asarray(lstRD), axis=0)
print stdRS
print stdRD
#print avgIS
#print avgRS
#print avgID
#print avgRD

    
avgRD = np.asarray(avgRD).T
avgRD[0] = 1 - avgRD[0]
avgCostD = avgRD[0] * 10 + avgRD[1]
#print avgCostD

avgRS = np.asarray(avgRS).T
avgRS[0] = 1 - avgRS[0]
avgCostS = avgRS[0] * 10 + avgRS[1]
#print avgCostS

#IPython.embed()

plt.figure(1)
p1, = plt.plot(avgID, avgCostD,color='b',linewidth=4.0,label='DAgger')
p2, = plt.plot(avgIS, avgCostS,color='r',linewidth=4.0,label='SHIV')
plt.legend([p1,p2], ['DAgger','SHIV'])
plt.ylabel('Cost')
plt.xlabel('States Labeled')

#plt.figure(2)
#plt.plot(avgI, avgR.T[1],color='r',linewidth=5.0)

plt.show()
