#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), 
		cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), 
		cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;

	reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
	// Build a filter to remove spurious NaNs,过滤Z轴点云数据
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, 1.1);
	pass.filter (*cloud_filtered);
	std::cerr << "PointCloud after filtering has: "
		<< cloud_filtered->points.size () << " data points." << std::endl;
	//
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);//储存模型估计参数
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);//inliers指针储存点云分割后的结果
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;//创建分割对象
	// Optional
	seg.setOptimizeCoefficients (true);//设置优化参数
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);//设置分割模型类型
	seg.setMethodType (pcl::SAC_RANSAC);//设置采样一直性估计算法为SAC_RANSAC
	seg.setDistanceThreshold (0.01);//设置距离阀值，与估计平面模型小于该阀值时为内点

	seg.setInputCloud (cloud_filtered);//设置输入点云为滤波后点云
	seg.segment (*inliers, *coefficients);
	std::cerr << "PointCloud after segmentation has: "
		<< inliers->indices.size () << " inliers." << std::endl;

	// Project the model inliers
	pcl::ProjectInliers<pcl::PointXYZ> proj;//点云投影滤波对象
	proj.setModelType (pcl::SACMODEL_PLANE);//设置投影模型为SACMODEL_PLANE
	proj.setInputCloud (cloud_filtered);//设置输入点云为滤波后点云
	proj.setModelCoefficients (coefficients);//设置估计得到的平面
	proj.filter (*cloud_projected);//投影后点云
	std::cerr << "PointCloud after projection has: "
		<< cloud_projected->points.size () << " data points." << std::endl;

	// Create a Concave Hull representation of the projected inliers
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConcaveHull<pcl::PointXYZ> chull;//创建多边形提取对象
	chull.setInputCloud (cloud_projected);//设置输入点云为投影后点云
	chull.setAlpha (0.1);//设置alpha值为0.1
	chull.reconstruct (*cloud_hull);//重建提取创建凹多边形

	std::cerr << "Concave hull has: " << cloud_hull->points.size ()
		<< " data points." << std::endl;

	pcl::PCDWriter writer;
	writer.write ("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);

	return (0);
}
