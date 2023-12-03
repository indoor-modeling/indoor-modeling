#define _CRT_SECURE_NO_DEPRECATE
#define PCL_NO_PRECOMPILE
#include "utils.hpp"


//void main() {
//	// ���������Ϣ
//	//---------------------------------------------------------------------------------
//	pcInfo info;
//	info.map_resolution = 0.05;
//
//	// 1. ��ȡply�ļ�,���� pcInfo
//	//---------------------------------------------------------------------------------
//	//std::string ply_path = "../data1/Area_6_full_sub(part).ply";
//	std::string ply_path = "../res/slicePC/Area_6_full_slice_sub.ply";
//	pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
//	read_ply<MyPointType>(ply_path, *cloud);
//	std::cout << "read success" << std::endl;
//
//	setPCInfo<MyPointType>(*cloud, info);
//	//savePCInfo("../data1/pcInfo.txt", info);
//
//	// 2. ��ȡ��ֱ�棬��Ƭ������
//	//---------------------------------------------------------------------------------
//	/*std::vector<int> vertical_labels = { 2,5,11 };
//	pcl::PointCloud<MyPointType>::Ptr label_cloud(new pcl::PointCloud<MyPointType>);
//	filterLabel(*cloud, *label_cloud, vertical_labels);*/
//
//	// pcl::PointCloud<MyPointType>::Ptr slice_cloud(new pcl::PointCloud<MyPointType>);
//	// PassThroughFilter<MyPointType>::filter(cloud, slice_cloud, 1.6, 1.8);
//
//	pcl::PointCloud<MyPointType>::Ptr project_cloud(new pcl::PointCloud<MyPointType>);
//	pcl_projection pj;
//	pj.project<MyPointType>(*cloud, *project_cloud);
//	pcl::PointCloud<MyPointType>::Ptr remove_duplicate_cloud(new pcl::PointCloud<MyPointType>);
//	voxel_grid<MyPointType>(*project_cloud, *remove_duplicate_cloud, 0.05, 0.05, 0.05);
//
//	pcl::visualization::PCLVisualizer vis("ss");
//	vis.addPointCloud<MyPointType>(remove_duplicate_cloud, "ssss");
//	while(!vis.wasStopped())
//		vis.spinOnce();
//
//	save_ply<MyPointType>("../res/slicePC/wall_new.ply", *remove_duplicate_cloud);
//}
//
//
//// ����ǽ���ߵ�ͼ
//void main() {
//	pcInfo info;
//	readPCInfo("../data/pcInfo.txt", info);
//	std::vector<int> labels;
//
//	pcl::PointCloud<MyPointType> cloud;
//	read_ply<MyPointType>("../res/slicePC/wall_new.ply", cloud);
//	cv::Mat wall = genSemanticMap(cloud, info, labels);
//	cv::imshow("wada", wall);
//	cv::imwrite("../res/wall_point.png", wall);
//	//cv::waitKey();
//}



// С�������Ƶ���ƬͶӰ
//void main() {
//	// ���������Ϣ
//	//---------------------------------------------------------------------------------
//	pcInfo info;
//	info.map_resolution = 0.05;
//
//	// 1. ��ȡply�ļ�,���� pcInfo
//	//---------------------------------------------------------------------------------
//	//std::string ply_path = "../data1/Area_6_full_sub(part).ply";
//	std::string ply_path = "../unbalance_small_scene/small_slice.ply";
//	pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
//	read_ply<MyPointType>(ply_path, *cloud);
//	std::cout << "read success" << std::endl;
//
//	setPCInfo<MyPointType>(*cloud, info);
//	//savePCInfo("../data1/pcInfo.txt", info);
//
//	// 2. ��ȡ��ֱ�棬��Ƭ������
//	//---------------------------------------------------------------------------------
//
//	pcl::PointCloud<MyPointType>::Ptr project_cloud(new pcl::PointCloud<MyPointType>);
//	pcl_projection pj;
//	pj.project<MyPointType>(*cloud, *project_cloud);
//	pcl::PointCloud<MyPointType>::Ptr remove_duplicate_cloud(new pcl::PointCloud<MyPointType>);
//	voxel_grid<MyPointType>(*project_cloud, *remove_duplicate_cloud, 0.05, 0.05, 0.05);
//
//	/*pcl::visualization::PCLVisualizer vis("ss");
//	vis.addPointCloud<MyPointType>(remove_duplicate_cloud, "ssss");
//	while (!vis.wasStopped())
//		vis.spinOnce();*/
//
//	save_ply<MyPointType>("../unbalance_small_scene/slice_pro.ply", *remove_duplicate_cloud);
//}


// ����5
//void main() {
//	// ���������Ϣ
//	//---------------------------------------------------------------------------------
//	pcInfo info;
//	info.map_resolution = 0.05;
//
//	// 1. ��ȡply�ļ�,���� pcInfo
//	//---------------------------------------------------------------------------------
//	//std::string ply_path = "../data1/Area_6_full_sub(part).ply";
//	std::string ply_path = "../area5/Area_5_slice.ply";
//	pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
//	read_ply<MyPointType>(ply_path, *cloud);
//	std::cout << "read success" << std::endl;
//
//	setPCInfo<MyPointType>(*cloud, info);
//	savePCInfo("../area5/pcInfo.txt", info);
//
//	// 2. ��ȡ��ֱ�棬��Ƭ������
//	//---------------------------------------------------------------------------------
//
//	pcl::PointCloud<MyPointType>::Ptr project_cloud(new pcl::PointCloud<MyPointType>);
//	pcl_projection pj;
//	pj.project<MyPointType>(*cloud, *project_cloud);
//	pcl::PointCloud<MyPointType>::Ptr remove_duplicate_cloud(new pcl::PointCloud<MyPointType>);
//	voxel_grid<MyPointType>(*project_cloud, *remove_duplicate_cloud, 0.05, 0.05, 0.05);
//
//	/*pcl::visualization::PCLVisualizer vis("ss");
//	vis.addPointCloud<MyPointType>(remove_duplicate_cloud, "ssss");
//	while (!vis.wasStopped())
//		vis.spinOnce();*/
//
//	save_ply<MyPointType>("../area5/slice_pro.ply", *remove_duplicate_cloud);
//}

 // ����5 part2
//void main() {
//	// ���������Ϣ
//	//---------------------------------------------------------------------------------
//	pcInfo info;
//	info.map_resolution = 0.05;
//
//	// 1. ��ȡply�ļ�,���� pcInfo
//	//---------------------------------------------------------------------------------
//	std::string ply_path = "../area5_part2/Area_5_part2_subwall.ply";
//	pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
//	read_ply<MyPointType>(ply_path, *cloud);
//	std::cout << "read success" << std::endl;
//
//	//setPCInfo<MyPointType>(*cloud, info);
//	//savePCInfo("../area5_part2/info/pcInfo.txt", info);
//	readPCInfo("../area5_part2/info/pcInfo.txt", info);
//
//	// 2. ��ȡ��ֱ�棬��Ƭ������
//	//---------------------------------------------------------------------------------
//
//	pcl::PointCloud<MyPointType>::Ptr project_cloud(new pcl::PointCloud<MyPointType>);
//	pcl_projection pj;
//	pj.project<MyPointType>(*cloud, *project_cloud);
//	pcl::PointCloud<MyPointType>::Ptr remove_duplicate_cloud(new pcl::PointCloud<MyPointType>);
//	voxel_grid<MyPointType>(*project_cloud, *remove_duplicate_cloud, 0.05, 0.05, 0.05);
//
//	/*pcl::visualization::PCLVisualizer vis("ss");
//	vis.addPointCloud<MyPointType>(remove_duplicate_cloud, "ssss");
//	while (!vis.wasStopped())
//		vis.spinOnce();*/
//
//	save_ply<MyPointType>("../area5_part2/slice_pro2.ply", *remove_duplicate_cloud);
//}



//// ����5ȫ��
//void main() {
//	// ���������Ϣ
//	//---------------------------------------------------------------------------------
//	pcInfo info;
//	info.map_resolution = 0.05;
//
//	// 1. ��ȡply�ļ�,���� pcInfo
//	//---------------------------------------------------------------------------------
//	std::string ply_path = "../area5_full/Area_5_full_sub0.05_slice.ply";
//	pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
//	read_ply<MyPointType>(ply_path, *cloud);
//	std::cout << "read success" << std::endl;
//
//	setPCInfo<MyPointType>(*cloud, info);
//	savePCInfo("../area5_full/info/pcInfo.txt", info);
//	readPCInfo("../area5_full/info/pcInfo.txt", info);
//
//	// 2. ��ȡ��ֱ�棬��Ƭ������
//	//---------------------------------------------------------------------------------
//
//	pcl::PointCloud<MyPointType>::Ptr project_cloud(new pcl::PointCloud<MyPointType>);
//	pcl_projection pj;
//	pj.project<MyPointType>(*cloud, *project_cloud);
//	pcl::PointCloud<MyPointType>::Ptr remove_duplicate_cloud(new pcl::PointCloud<MyPointType>);
//	voxel_grid<MyPointType>(*project_cloud, *remove_duplicate_cloud, 0.05, 0.05, 0.05);
//
//	/*pcl::visualization::PCLVisualizer vis("ss");
//	vis.addPointCloud<MyPointType>(remove_duplicate_cloud, "ssss");
//	while (!vis.wasStopped())
//		vis.spinOnce();*/
//
//	save_ply<MyPointType>("../area5_full/slice_pro.ply", *remove_duplicate_cloud);
//}


//// ����4
//void main() {
//	// ���������Ϣ
//	//---------------------------------------------------------------------------------
//	pcInfo info;
//	info.map_resolution = 0.05;
//
//	// 1. ��ȡply�ļ�,���� pcInfo
//	//---------------------------------------------------------------------------------
//	std::string ply_path = "../area4/Area_4_slice.ply";
//	pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
//	read_ply<MyPointType>(ply_path, *cloud);
//	std::cout << "read success" << std::endl;
//
//	setPCInfo<MyPointType>(*cloud, info);
//	savePCInfo("../area4/info/pcInfo.txt", info);
//	readPCInfo("../area4/info/pcInfo.txt", info);
//
//	// 2. ��ȡ��ֱ�棬��Ƭ������
//	//---------------------------------------------------------------------------------
//
//	pcl::PointCloud<MyPointType>::Ptr project_cloud(new pcl::PointCloud<MyPointType>);
//	pcl_projection pj;
//	pj.project<MyPointType>(*cloud, *project_cloud);
//	pcl::PointCloud<MyPointType>::Ptr remove_duplicate_cloud(new pcl::PointCloud<MyPointType>);
//	voxel_grid<MyPointType>(*project_cloud, *remove_duplicate_cloud, 0.05, 0.05, 0.05);
//
//	/*pcl::visualization::PCLVisualizer vis("ss");
//	vis.addPointCloud<MyPointType>(remove_duplicate_cloud, "ssss");
//	while (!vis.wasStopped())
//		vis.spinOnce();*/
//
//	save_ply<MyPointType>("../area4/slice_pro.ply", *remove_duplicate_cloud);
//}


// ����3
void main() {
	// ���������Ϣ
	//---------------------------------------------------------------------------------
	pcInfo info;
	info.map_resolution = 0.05;

	// 1. ��ȡply�ļ�,���� pcInfo
	//---------------------------------------------------------------------------------
	std::string ply_path = "../area3/Area_3_slice.ply";
	pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
	read_ply<MyPointType>(ply_path, *cloud);
	std::cout << "read success" << std::endl;

	setPCInfo<MyPointType>(*cloud, info);
	savePCInfo("../area3/info/pcInfo.txt", info);
	readPCInfo("../area3/info/pcInfo.txt", info);

	// 2. ��ȡ��ֱ�棬��Ƭ������
	//---------------------------------------------------------------------------------

	pcl::PointCloud<MyPointType>::Ptr project_cloud(new pcl::PointCloud<MyPointType>);
	pcl_projection pj;
	pj.project<MyPointType>(*cloud, *project_cloud);
	pcl::PointCloud<MyPointType>::Ptr remove_duplicate_cloud(new pcl::PointCloud<MyPointType>);
	voxel_grid<MyPointType>(*project_cloud, *remove_duplicate_cloud, 0.05, 0.05, 0.05);

	/*pcl::visualization::PCLVisualizer vis("ss");
	vis.addPointCloud<MyPointType>(remove_duplicate_cloud, "ssss");
	while (!vis.wasStopped())
		vis.spinOnce();*/

	save_ply<MyPointType>("../area3/slice_pro.ply", *remove_duplicate_cloud);
}