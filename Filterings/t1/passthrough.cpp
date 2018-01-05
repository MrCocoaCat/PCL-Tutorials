#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); //滤波前点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);//滤波后点云

  // Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);
  //随机初始化
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;//滤波器对象
  pass.setInputCloud (cloud);  //设置输入点云
  pass.setFilterFieldName ("z"); // 设置在z轴上滤波
  pass.setFilterLimits (0.0, 1.0); //范围，在此范围内的点被剔除
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);//执行后的结果

  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " " 
                        << cloud_filtered->points[i].y << " " 
                        << cloud_filtered->points[i].z << std::endl;

  return (0);
}
