#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


int main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 // sensor_msgs::PointCloud2 cloud_blob;
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("table_scene_lms400_downsampled.pcd", cloud_blob);
  //pcl::fromROSMsg (cloud_blob, *cloud);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud

  //定义KDtree 指针
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  //构建tree对象
  tree->setInputCloud (cloud);

  // Normal estimation*
  //法线估计对象
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  //存储估计的法线
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  n.setInputCloud (cloud); //法线估计输入
  n.setSearchMethod (tree); //搜索方法
  n.setKSearch (20); //k值
  n.compute (*normals); //储存结果
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
    //储存法线和点云
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
    //定义三角化对象
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    //储存最终三角化的模型
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  //设置链接点最大距离
  gp3.setSearchRadius (0.025);
  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100); //收缩邻域
  gp3.setMaximumSurfaceAngle(M_PI/4); // 偏离角度
  gp3.setMinimumAngle(M_PI/18); // 10 degrees 内角最小角
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees 内角最大角
  gp3.setNormalConsistency(false);
  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);
 // std::cout << triangles;
  // Additional vertex information

  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPolygonMesh(triangles,"my");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
   // 主循环
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  // Finish
  return (0);
}
