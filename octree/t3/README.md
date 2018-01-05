An octree is a tree-based data structure for organizing sparse 3-D data. In this tutorial we will learn how to use the octree implementation for detecting spatial changes between multiple unorganized point clouds which could vary in size, resolution, density and point ordering. By recursively comparing the tree structures of octrees, spatial changes represented by differences in voxel configuration can be identified. Additionally, we explain how to use the pcl octree “double buffering” technique allows us to efficiently process multiple point clouds over time.
octree 是一种用于稀疏3D数据的树状数据结构，本小节中将介绍如何利用octree实现用于多个无序点云之间的空间变化检测，
通过递归的比较octree的树结构，可以坚定出octree产生的体素之间的区别所代表的空间变化。
此外介绍“双缓冲技术”
