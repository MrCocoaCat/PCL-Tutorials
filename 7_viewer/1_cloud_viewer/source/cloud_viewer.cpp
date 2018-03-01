#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
    
int user_data;
void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (0, 0, 0);//���ñ�����ɫ
//     pcl::PointXYZ o;//�洢���Բ��λ��
//     o.x = 1.0;
//     o.y = 0;
//     o.z = 0;
//     viewer.addSphere (o, 0.25, "sphere", 0);//���Բ�򼯺϶���
//     std::cout << "i only run once" << std::endl;
    
}
//     
// void 
// viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
// {
//     static unsigned count = 0;
//     std::stringstream ss;
//     ss << "Once per viewer loop: " << count++;
//     viewer.removeShape ("text", 0);
//     viewer.addText (ss.str(), 200, 300, "text", 0);
//     //FIXME: possible race condition here:
//     user_data++;
// }
    
int 
main (int argc,char** argv)
{
   // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


	  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
std::string file_name = argv[pcd_filename_indices[0]];

    pcl::io::loadPCDFile (file_name, *cloud);//���ص����ļ�
    pcl::visualization::CloudViewer viewer("Cloud Viewer");    
    //showCloud������ͬ���ģ��ڴ˴��ȴ�ֱ����Ⱦ��ʾΪֹ
    viewer.showCloud(cloud);
    //��ע�ắ���ڿ��ӻ�ʱֻ����һ��
    viewer.runOnVisualizationThreadOnce (viewerOneOff);
    //��ע�ắ������Ⱦ���ʱÿ�ζ�����
    //viewer.runOnVisualizationThread (viewerPsycho);
    while (!viewer.wasStopped ())
    {
    //�ڴ˴����������������
    user_data++;
    }
    return 0;
}
