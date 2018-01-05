#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ()); //用于存放读入数据
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ()); //用于存放滤波后数据

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  // Remember to download the file first!
  reader.read ("..//table_scene_lms400.pcd", *cloud);//将pcd文件读入到cloud对象中

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")."<<std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor; //创建VoxelGrid滤波对象
  sor.setInputCloud (cloud); //设置滤波输入cloud
  sor.setLeafSize (0.01f, 0.01f, 0.01f);//设置滤波参数
  sor.filter (*cloud_filtered); //将滤波后数据存入cloud_filtered

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")."<<std::endl;

  pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  return (0);
}
