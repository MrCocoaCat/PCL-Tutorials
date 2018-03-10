#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  //reader.read ("../table_scene_lms400.pcd", *cloud_blob);
  reader.read ("../0.pcd", *cloud_blob);

  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  // 进行进向下采样滤波

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

///******************


  pcl::PointCloud<pcl::PointXYZ>::Ptr zxyz_blob (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr z_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromPCLPointCloud2 (*cloud_blob, *zxyz_blob);


    pcl::PCDWriter wr1;
    wr1.write<pcl::PointXYZ> ("org.pcd", *zxyz_blob, false);

  pcl::PassThrough<pcl::PointXYZ> pass;//滤波器对象
  pass.setInputCloud (zxyz_blob);  //设置输入点云
  pass.setFilterFieldName ("z"); // 设置在z轴上滤波
  pass.setFilterLimits (0.0, 1.0); //范围
  pass.setFilterLimitsNegative (false);
  pass.filter (*cloud_filtered);//执行后的结果


  std::cout<<zxyz_blob->points.size()<<"  "<<cloud_filtered->points.size ()<<std::endl;

  pcl::PCDWriter wri;
  wri.write<pcl::PointXYZ> ("AfterZFil.pcd", *cloud_filtered, false);
///****************z up

  // Convert to the templated PointCloud
  //pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

    //cloud2 转换 xyz
  //pcl::fromPCLPointCloud2 (*cloud_blob, *cloud_filtered);

 // std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk

  pcl::PCDWriter writer;
  //writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg; //创建分割对象
  // Optional
  seg.setOptimizeCoefficients (true);//设置对估计的模型参数进行优化处理
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);//设置分割模型类型
  seg.setMethodType (pcl::SAC_RANSAC);//设置用哪个随机参数估计方法
  seg.setMaxIterations (1000);//设置最大迭代次数
  seg.setDistanceThreshold (0.01);//设置判断是否为模型内点的距离阀值

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract; //创建点云提取对象

  int i = 0,nr_points = (int) cloud_filtered->points.size ();//
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered); //设置输入点云
    extract.setIndices (inliers); //设置分割后的内点为需要提取的点集
    extract.setNegative (true); //设置提取内点而非外点
    extract.filter (*cloud_p); //提出输入为cloud_p
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }




  return (0);
}

