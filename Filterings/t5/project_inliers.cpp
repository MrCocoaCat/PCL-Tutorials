#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before projection: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;

  // Create a set of planar coefficients with X=Y=0,Z=1
  //定义模型对象，填充对应
  //ax+by+cz+d = 0                   
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ()); 
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0; //a=b=0
  coefficients->values[2] = 1.0; //c=1
  coefficients->values[3] = 0; //d=0

  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZ> proj; //创建投影滤波对象
  proj.setModelType (pcl::SACMODEL_PLANE); //设置好对象对应的投影模型
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients); //设置好对应系数
  proj.filter (*cloud_projected);

  std::cerr << "Cloud after projection: " << std::endl;
  for (size_t i = 0; i < cloud_projected->points.size (); ++i)
    std::cerr << "    " << cloud_projected->points[i].x << " " 
                        << cloud_projected->points[i].y << " " 
                        << cloud_projected->points[i].z << std::endl;

  return (0);
}
