
import numpy as np
import csv

from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

import pandas as pd
from pyntcloud import PyntCloud

import matplotlib.pyplot as plt

# After completed scan, take combined pose + mass spec data and adjust for delay / noise
# Keep only points with +ve classification
# Use DBSCAN (or other) clustering technique
# Find bounding boxes / centre



# with open('control/paths/spiralZ 2022-05-24 15-13-38 15mmRad30.0EqSide 97-5.csv', newline = '') as csvPath:
#     coordReader = csv.reader(csvPath)
#     for row in coordReader:
#         xPath.append(float(row[0]))
#         yPath.append(float(row[1]))
#         zPath.append(float(row[2]))
#         MSClass.append(float(row[3]))


# cloud = PyntCloud(pd.DataFrame(data=np.array([MSClass]), columns=["x", "y", "z", "classification"]))
# cloud = PyntCloud(pd.DataFrame(data=np.hstack((points, colors)), columns=["x", "y", "z", "classification"]))
# cloud.to_file("output.ply")

class dataClustering:

    def __init__(self):
        self.xData = []
        self.yData = []
        self.zData = []
        self.msData = []
        self.dbInput = np.array([])
        self.dbInputScaled = np.array([])


    def loadData(self, filePath):
        with open(filePath, newline ='') as MSData:
            dataReader = csv.reader(MSData)
            i = 0
            repVal = 0# float('nan')
            # Segment data based on MS classification
            # Discard values if no pose info present?
            for row in dataReader:
                if i == 0:
                    i += 1 # Skip the first row
                else:
                    if (int(row[6]) == 1) and (row[0] != ' '):
                        self.xData.append(float(row[0]) if row[0] != ' ' else repVal)
                        self.yData.append(float(row[1]) if row[1] != ' ' else repVal)
                        self.zData.append(float(row[2]) if row[2] != ' ' else repVal)
                        self.msData.append(float(row[6]) if row[6] != ' ' else repVal)

        self.dbInput = np.column_stack((self.xData, self.yData, self.zData, self.msData))
        self.dbInputScaled = StandardScaler().fit_transform(self.dbInput)
        return self.dbInput


    def shiftSamples(self, arr, num, fill_value):
        # Find delay value that minimises jagged-ness of blobs
            # Shift MS data in the order it was collected
        result = np.empty_like(arr)
        if num > 0:
            result[:num] = fill_value
            result[num:] = arr[:-num]
        elif num < 0:
            result[num:] = fill_value
            result[:num] = arr[-num:]
        else:
            result[:] = arr
        return result




# CLustering might frst be done just finding incices where msClass == 1

    def clusterBlobs(self):
        # db = DBSCAN(eps=0.3, min_samples=10).fit(self.dbInputScaled[:,3].reshape(-1, 1))
        db = DBSCAN(eps=0.3, min_samples=10).fit(self.dbInputScaled[:,0:2])
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True 
        labels = db.labels_

        # Number of clusters in labels, ignoring noise if present.
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        n_noise_ = list(labels).count(-1)


        # Black removed and is used for noise instead.
        unique_labels = set(labels)
        trueMask = labels == 1
        # print(len(self.dbInput[trueMask])) # Use labels to create masks
        colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        for k, col in zip(unique_labels, colors):
            if k == -1:
                # Black used for noise.
                col = [0, 0, 0, 1]

            class_member_mask = labels == k

            xy = self.dbInput[class_member_mask & core_samples_mask]
            # plt.plot(
            ax.scatter(
                xy[:, 0],
                xy[:, 1],
                xy[:,2],
                marker = 'o',
                c=list(col)
                # markerfacecolor=tuple(col),
                # markeredgecolor="k",
                # markersize=14,
            )

            xy = self.dbInput[class_member_mask & ~core_samples_mask]
            # plt.plot(
            ax.scatter(
                xy[:, 0],
                xy[:, 1],
                xy[:,2],
                marker = '.',
                c=list(col)
                # markerfacecolor=tuple(col),
                # markeredgecolor="k",
                # markersize=6,
            )

        plt.title("Estimated number of clusters: %d" % n_clusters_)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()



fileNamePath = 'control/logs/Pose_and_MS_Test_data_-_pose 2022-11-14 16-05-03 (1).csv'

dataClust = dataClustering()

dataClust.loadData(fileNamePath)
dataClust.clusterBlobs()