#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
  main (int argc, char** argv)
{
  if (argc != 2)
  {
    //-f 链接点云 -p链接字段
    std::cerr << "please specify command line arg '-f' or '-p'" << std::endl;
    exit(0);
  }
  pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;
  pcl::PointCloud<pcl::Normal> n_cloud_b; //储存进行链接时需要的normal点云
  pcl::PointCloud<pcl::PointNormal> p_n_cloud_c; //储存链接XYZ与normal 后的点云

  // Fill in the cloud data,创建点云数据
  cloud_a.width  = 5;
  cloud_a.height = cloud_b.height = n_cloud_b.height = 1;
  cloud_a.points.resize (cloud_a.width * cloud_a.height);
  if (strcmp(argv[1], "-p") == 0)
  {
    //b的宽度为3
    cloud_b.width  = 3;
    cloud_b.points.resize (cloud_b.width * cloud_b.height);
  }
  else
  {
      //-f情况 b的宽度为5
    n_cloud_b.width = 5;
    n_cloud_b.points.resize (n_cloud_b.width * n_cloud_b.height);
  }

  //填充cloud_a
  for (size_t i = 0; i < cloud_a.points.size (); ++i)
  {
    cloud_a.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_a.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_a.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  //填充cloud_b
  if (strcmp(argv[1], "-p") == 0)
  {
      //-p情况下填充
      for (size_t i = 0; i < cloud_b.points.size (); ++i)
      {
          cloud_b.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
          cloud_b.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
          cloud_b.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
      }
  }
  else
  {
      //-f情况下填充
      for (size_t i = 0; i < n_cloud_b.points.size (); ++i)
      {
          n_cloud_b.points[i].normal[0] = 1024 * rand () / (RAND_MAX + 1.0f);
          n_cloud_b.points[i].normal[1] = 1024 * rand () / (RAND_MAX + 1.0f);
          n_cloud_b.points[i].normal[2] = 1024 * rand () / (RAND_MAX + 1.0f);
      }
  }

  //输出点云A 点云B
  std::cerr << "Cloud A: " << std::endl;
  for (size_t i = 0; i < cloud_a.points.size (); ++i)
  {
    std::cerr << "    " << cloud_a.points[i].x << " " << cloud_a.points[i].y << " " << cloud_a.points[i].z << std::endl;
  }

  std::cerr << "Cloud B: " << std::endl;
  if (strcmp(argv[1], "-p") == 0)
  {

    for (size_t i = 0; i < cloud_b.points.size (); ++i)
      std::cerr << "    " << cloud_b.points[i].x << " " << cloud_b.points[i].y << " "
                << cloud_b.points[i].z << std::endl;
  }
  else
  {
    for (size_t i = 0; i < n_cloud_b.points.size(); ++i)
      std::cerr << "    " << n_cloud_b.points[i].normal[0] << " " << n_cloud_b.points[i].normal[1] << " "
                << n_cloud_b.points[i].normal[2] << std::endl;
  }



  // Copy the point cloud data
  if (strcmp(argv[1], "-p") == 0)
  {
    //-p
    cloud_c  = cloud_a;
    cloud_c += cloud_b; //行变化,变为8行
    std::cerr << "链接点云（c=a+b）Cloud C: " << std::endl;
    for (size_t i = 0; i < cloud_c.points.size (); ++i)
      std::cerr << "    " << cloud_c.points[i].x << " " << cloud_c.points[i].y << " "
                << cloud_c.points[i].z << " " << std::endl;
  }
  else
  {
    //-f
    pcl::concatenateFields (cloud_a, n_cloud_b, p_n_cloud_c); //链接字段函数,列变化,变为6列
    std::cerr << "链接字段Cloud C(concatenateFields): " << std::endl;
    for (size_t i = 0; i < p_n_cloud_c.points.size (); ++i)
      std::cerr << "    "
                << p_n_cloud_c.points[i].x << " " << p_n_cloud_c.points[i].y << " " << p_n_cloud_c.points[i].z << " "
                << p_n_cloud_c.points[i].normal[0] << " " << p_n_cloud_c.points[i].normal[1] << " " << p_n_cloud_c.points[i].normal[2] << std::endl;
  }
  return (0);
}

