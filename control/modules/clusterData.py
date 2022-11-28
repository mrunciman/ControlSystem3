
import numpy as np

from sklearn.cluster import DBSCAN


# After completed scan, take combined pose + mass spec data and adjust for delay / noise
# Keep only points with +ve classification
# Use DBSCAN (or other) clustering technique
# Find bounding boxes / centre

class dataClustering:

    def __init__(self):
        self.data
        self.unhealthyBlobs

    
    def cancelNoise(self):
        # Find delay value that minimises jagged-ness of blobs
        self.data = []



    
    def clusterBlobs(self):
        db = DBSCAN(eps=0.3, min_samples=10).fit(self.unhealthyBlobs)
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_

        # Number of clusters in labels, ignoring noise if present.
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        n_noise_ = list(labels).count(-1)

