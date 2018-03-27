#include <iostream> //标准输入/输出
#include <pcl/io/pcd_io.h> //pcd文件输入/输出
#include <pcl/point_types.h> //各种点类型
#include <pcl/registration/icp.h> //ICP(iterative closest point)配准

int main (int argc, char** argv)
{
    //创建点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>); //创建输入点云（指针）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>); //创建输出/目标点云（指针）

    //生成并填充点云cloud_in

    pcl::io::loadPCDFile (argv[1], *cloud_in); //读点云到结构体
    pcl::io::loadPCDFile (argv[2], *cloud_out); //读点云到结构体

    //*********************************
    // ICP配准
    //*********************************
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //创建ICP对象，用于ICP配准
    icp.setInputSource(cloud_in); //设置输入点云
    icp.setInputTarget(cloud_out); //设置目标点云（输入点云进行仿射变换，得到目标点云）
    icp.
    pcl::PointCloud<pcl::PointXYZ> Final; //存储结果
    //进行配准，结果存储在Final中
    icp.align(Final);
    //输出ICP配准的信息（是否收敛，拟合度）
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    //输出最终的变换矩阵（4x4）
    std::cout << icp.getFinalTransformation() << std::endl;






    return (0);
}