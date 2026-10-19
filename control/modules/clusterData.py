
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
        self.allData = np.array([])
        self.xDataUnhealthy = []
        self.yDataUnhealthy = []
        self.zDataUnhealthy = []
        self.msDataUnhealthy = []
        self.dbInput = np.array([])
        self.dbInputScaled = np.array([])
        self.centres = np.array([])


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
                elif (row[0] != ' ') and (row[6] != ' '):
                    self.xData.append(float(row[0]))
                    self.yData.append(float(row[1]))
                    self.zData.append(float(row[2]))
                    self.msData.append(int(row[6]))
                # PRESERVE ALL DATA
                # else:      
                #     self.xData.append(float(row[0]) if row[0] != ' ' else 0)
                #     self.yData.append(float(row[1]) if row[1] != ' ' else 0)
                #     self.zData.append(float(row[2]) if row[2] != ' ' else 0)
                #     self.msData.append(int(row[6]) if row[6] != ' ' else 0)

            self.allData = np.column_stack((self.xData, self.yData, self.zData, self.msData))
            fig0 = plt.figure()
            ax0 = fig0.add_subplot(projection='3d')
            ax0.scatter(
                self.allData[:, 0],
                self.allData[:, 1],
                self.allData[:,2],
                marker = 'o',
                color= [0, 0, 0, 1]
                )
            ax0.set_xlabel('X')
            ax0.set_ylabel('Y')
            ax0.set_zlabel('Z')

            for i in range(len(self.xData)):
                pointIsUnhealthy = bool(self.msData[i])
                poseDataPresent = False if self.xData[i] == ' ' else True
                if pointIsUnhealthy and poseDataPresent:
                    self.xDataUnhealthy.append(self.xData[i])
                    self.yDataUnhealthy.append(self.yData[i])
                    self.zDataUnhealthy.append(self.zData[i])
                    self.msDataUnhealthy.append(self.msData[i])


        if len(self.xDataUnhealthy) > 0:
            self.dbInput = np.column_stack((self.xDataUnhealthy, self.yDataUnhealthy, self.zDataUnhealthy, self.msDataUnhealthy))
            self.dbInputScaled = StandardScaler().fit_transform(self.dbInput)
            return self.dbInputScaled
        else:
            return None
        


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

    def clusterBlobs(self, fileNamePath, plotScans):
        dbInputScaled = self.loadData(fileNamePath)
        if dbInputScaled is not None:
            # db = DBSCAN(eps=0.3, min_samples=10).fit(self.dbInputScaled[:,3].reshape(-1, 1))
            db = DBSCAN(eps=0.3, min_samples=10).fit(dbInputScaled[:,0:2])
            core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
            core_samples_mask[db.core_sample_indices_] = True 
            labels = db.labels_

            # Number of clusters in labels, ignoring noise if present.
            n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
            n_noise_ = list(labels).count(-1)


            # Black removed and is used for noise instead.
            unique_labels = set(labels)
            # trueMask = labels == 0
            # print(len(self.dbInput[trueMask])) # Use labels to create masks ZERO INDEXED
            bboxRanges = np.zeros((n_clusters_, 6)) 
            centres = np.zeros((n_clusters_, 3)) 

            for cluster in unique_labels:

                class_member_mask = labels == cluster

                xyz = self.dbInput[class_member_mask & core_samples_mask]

                # Find bounding boxes and their centres
                if cluster != -1: # ignore noise
                    bboxRanges[cluster,0] = min(xyz[:,0])
                    bboxRanges[cluster,1] = max(xyz[:,0])
                    bboxRanges[cluster,2] = min(xyz[:,1])
                    bboxRanges[cluster,3] = max(xyz[:,1])
                    bboxRanges[cluster,4] = min(xyz[:,2])
                    bboxRanges[cluster,5] = max(xyz[:,2])
                    centres[cluster,0] = ((bboxRanges[cluster,1] - bboxRanges[cluster,0])/2) + bboxRanges[cluster,0]
                    centres[cluster,1] = (bboxRanges[cluster,3] - bboxRanges[cluster,2])/2 + bboxRanges[cluster,2]
                    centres[cluster,2] = (bboxRanges[cluster,5] - bboxRanges[cluster,4])/2 + bboxRanges[cluster,4]

            # self.bboxValues = bboxRanges
            self.centres = centres
            print(centres)
            if plotScans:
                self.plotClusters(unique_labels, labels, core_samples_mask, n_clusters_, centres)

            return n_clusters_
        else:
            return None


    def plotClusters(self, unLabels, allLabels, coreMask, numClusters, centres):
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(projection='3d')
        colourList = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unLabels))]
        for cluster, col in zip(unLabels, colourList):

            classMask = allLabels == cluster

            xyz = self.dbInput[classMask & coreMask]

            if cluster == -1:
                # Black colour used for noise.
                col = [0, 0, 0, 1]

            ax1.scatter(
                xyz[:, 0],
                xyz[:, 1],
                xyz[:,2],
                marker = 'o',
                color = col
            )

            # Plot members that are not likely to belong to class
            # xyzNOT = self.dbInput[classMask & ~coreMask]
            # ax1.scatter(
            #     xyzNOT[:, 0],
            #     xyzNOT[:, 1],
            #     xyzNOT[:,2],
            #     marker = '.',
            #     color = col
            # )
        ax1.scatter(
            centres[:,0],
            centres[:,1],
            centres[:,2],
            marker = '+',
            color = [0, 0, 0, 1],
            s = 64
        )

        plt.title("Estimated number of clusters: %d" % numClusters)
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')
        plt.show()




if __name__ == '__main__':
    # fileNamePath = 'control/logs/Pose_and_MS_Test_data_-_pose 2022-11-14 16-05-03 (1).csv'
    fileNamePath = 'control/logs/display pose 2023-03-03 16-36-36.csv'

    dataClust = dataClustering()

    # dataClust.loadData(fileNamePath)
    dataClust.clusterBlobs(fileNamePath, 1)