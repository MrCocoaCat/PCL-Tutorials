#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

int
 main (int argc, char** argv)
{
  
  if (argc != 2)
  {
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  srand((int)time(0));
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

  if (strcmp(argv[1], "-r") == 0) //使用RadiusOutlierRemoval 滤波器，周围点不足则被过滤
  {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem; //声明滤波对象
    // build the filter
    outrem.setInputCloud(cloud); //设置输入点云
    outrem.setRadiusSearch(0.8);  //在0.8 半径内寻找临近点
    outrem.setMinNeighborsInRadius (2); //临近点小于2个则删除
    // apply filter
    outrem.filter (*cloud_filtered); //进行滤波，执行后结果放入cloud_filtered中
  }
  else if (strcmp(argv[1], "-c") == 0) //使用ConditionalRemoval 滤波器
  {
    // build the condition
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ()); //创建条件定义变量
    range_cond->addComparison(
            pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
                    new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0))); //添加Z轴大于0.0比较算子
    range_cond->addComparison (
            pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
                    new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8))); //添加Z轴小于0.8比较算子
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem; //声明滤波对象
    condrem.setCondition (range_cond); //按照条件算子进行过滤
    condrem.setInputCloud (cloud); //设置输入点云
    condrem.setKeepOrganized(true); //保持点云结构
    // apply filter
    condrem.filter (*cloud_filtered); //输出
  }
  else
  {
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }

  //输出滤波钱数据
  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
                        << cloud->points[i].y << " "
                        << cloud->points[i].z << std::endl;
  // 输出滤波后数据
  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " "
                        << cloud_filtered->points[i].y << " "
                        << cloud_filtered->points[i].z << std::endl;
  return (0);
}
