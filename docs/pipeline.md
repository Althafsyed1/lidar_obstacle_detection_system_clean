# Pipeline Overview

Input: sensor_msgs/PointCloud2

1) ROI filtering (remove irrelevant points)
2) Ground segmentation (separate ground vs obstacles)
3) Clustering (group obstacle points)
4) Bounding box / centroid extraction
5) Tracking (assignment across frames; Hungarian / IOU)
6) Visualization (RViz markers)

Outputs:
- Obstacle markers (boxes/centroids)
- Tracked IDs (where available)
