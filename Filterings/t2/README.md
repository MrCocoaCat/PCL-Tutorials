In this tutorial we will learn how to downsample – that is, reduce the number of points – a point cloud dataset, using a voxelized grid approach.

The VoxelGrid class that we’re about to present creates a 3D voxel grid (think about a voxel grid as a set of tiny 3D boxes in space) over the input point cloud data. Then, in each voxel (i.e., 3D box), all the points present will be approximated (i.e., downsampled) with their centroid. This approach is a bit slower than approximating them with the center of the voxel, but it represents the underlying surface more accurately.
读pcd文件并用VoxelGrid滤波器进行滤波，输出前后对比
