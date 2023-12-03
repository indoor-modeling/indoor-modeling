#define PCL_NO_PRECOMPILE
#include "include.h"
#include "CGAL_utils.h"
#include "Graph.h"
#include "pcl_utils.hpp"
#include "visualization.h"

void main()
{
	// 定义基本信息
	//---------------------------------------------------------------------------------
	pcInfo info;
	info.map_resolution = 0.05;

	// 1. 读取ply文件,设置 pcInfo
	//---------------------------------------------------------------------------------
	std::string ply_path = "./data/Area_6_full_sub.ply";
	pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
	PLY_IO<MyPointType>::read_ply(ply_path, cloud);
	std::cout << "read success" << std::endl;

	setPCInfo<MyPointType>(*cloud, info);
	savePCInfo("./data/pcInfo.txt", info);
	// 2. 提取竖直面，切片，提取直线，简化直线
	//---------------------------------------------------------------------------------
	std::vector<int> vertical_labels = { 2,5,11 };
	pcl::PointCloud<MyPointType>::Ptr label_cloud(new pcl::PointCloud<MyPointType>);
	filterLabel(*cloud, *label_cloud, vertical_labels);

	// pcl::PointCloud<MyPointType>::Ptr slice_cloud(new pcl::PointCloud<MyPointType>);
	// PassThroughFilter<MyPointType>::filter(cloud, slice_cloud, 1.6, 1.8);

	pcl::PointCloud<MyPointType>::Ptr project_cloud(new pcl::PointCloud<MyPointType>);
	pcl_projection<MyPointType> pj;
	pj.project(label_cloud, project_cloud);
	pcl::PointCloud<MyPointType>::Ptr remove_duplicate_cloud(new pcl::PointCloud<MyPointType>);
	pcl_downSample<MyPointType>::voxel_grid(project_cloud, remove_duplicate_cloud, 0.05, 0.05, 0.05);
	// pcl_vis<MyPointType>::showPointCloud(project_cloud);

	RANSACLines<MyPointType> ransac;
	//ransac.genWallLines(remove_duplicate_cloud, 100, 0.05, 0.05); // !!!maxmisspoint<0.3
	ransac.readLines("./result/RANSAC LINES");
	//ransac.readCoff("./result/RANSAC LINES/lines_coff.txt");
	

	std::vector<Segment> segments;
	genSegmens<MyPointType>(ransac.lines, info, segments);
	for (auto line : ransac.lines)
		std::cout << line.size() << std::endl;
	cv::Mat map = cv::imread("./data/wall_map_with_3.png", 0);
	/*show_Segments_on_map(segments, map, info);
	saveSegmens("./result/Reasonable lines with bbox/Segments.txt", segments);*/
	auto s = readSegmens("./result/Reasonable lines with bbox/Segments.txt");
	show_Segments_on_map(s, map, info);
	Mean_Shift ms;
	ms.setInputLines(segments);
	ms.setKernel_bandwidth(0.1);
	ms.fitLines();
	show_Segments_on_map(ms.seg, map, info);

	auto simple_lines = readSegmens("./result/SIMPLE LINES/simple_lines.txt");
	std::vector<Segment> segments2;
	genSegmens(simple_lines, info, segments2);
	show_Segments_on_map(segments2, map, info);
	saveSegmens("./result/Reasonable lines with bbox/Segments2.txt", segments2);
	// pcl_vis<MyPointType>::show_Ransac_lines(project_cloud, ransac.coff);
	//pcl_vis<MyPointType>::show_Ransac_lines(lines);
	//std::string save_ply_path = "./result/SLICE PC/slice.ply";
	//PLY_IO<MyPointType>::save_ply(save_ply_path, project_cloud);

	std::string in;
	std::cin >> in;
	if (in == "1")
	{
		Clouds_vector_IO<MyPointType>::save("./result/RANSAC LINES", ransac.lines);
		saveCoff_vector("./result/RANSAC LINES/lines_coff.txt", ransac.coff);
	}
}