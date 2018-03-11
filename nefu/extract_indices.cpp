#include <iostream>
#include <string>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
int main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
          cloud_p (new pcl::PointCloud<pcl::PointXYZ>),
          cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr x_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  // Fill in the cloud data
  pcl::PCDReader reader;
  //reader.read ("../table_scene_lms400.pcd", *cloud_blob);
  reader.read (argv[1], *cloud_blob);

  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  // 进行进向下采样滤波

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);




///******************按轴进行滤波


  pcl::PointCloud<pcl::PointXYZ>::Ptr org (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr vol (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr z_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromPCLPointCloud2 (*cloud_blob, *org);

  pcl::PCDWriter wr1;
  wr1.write<pcl::PointXYZ> ("org.pcd", *org, false);


  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *vol);
  wr1.write<pcl::PointXYZ> ("vol_org.pcd", *vol, false);



  std::cout<<"org:"<<org->points.size()<<" after vol "<<vol->points.size ()<<std::endl;


  pcl::PassThrough<pcl::PointXYZ> pass;//滤波器对象
  pass.setInputCloud (org);  //设置输入点云
  pass.setFilterFieldName ("z"); // 设置在z轴上滤波
  pass.setFilterLimits (0.0, 1.0); //范围
  pass.setFilterLimitsNegative (false);
  pass.filter (*cloud_filtered);//执行后的结果



  std::cout<<org->points.size()<<"  "<<cloud_filtered->points.size ()<<std::endl;

  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *vol);
  wr1.write<pcl::PointXYZ> ("pass.pcd", *cloud_filtered, false);


    pcl::PassThrough<pcl::PointXYZ> passx;//滤波器对象
    passx.setInputCloud (org);  //设置输入点云
    passx.setFilterFieldName ("x"); // 设置在x轴上滤波
    passx.setFilterLimits (-0.8, 0.8); //范围
    passx.setFilterLimitsNegative (false);

    passx.filter (*x_cloud_filtered);//执行后的结果
    wr1.write<pcl::PointXYZ> ("passx.pcd", *x_cloud_filtered, false);


//  pcl::PCDWriter wri;
//  std::string name;
//  name = name + argv[1];
//  name.insert(0 , "fi_");
//
//  wri.write<pcl::PointXYZ> (name.c_str(), *cloud_filtered, false);
///****************z up


  return (0);
}

