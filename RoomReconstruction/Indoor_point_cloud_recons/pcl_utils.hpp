#pragma once

#include "include.h"
#include "MeanShift.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件
#include <pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>    //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件
#include <pcl/ModelCoefficients.h> // 拟合模型系数 头文件
#include <pcl/segmentation/sac_segmentation.h> // ransac分割头文件
#include <pcl/filters/extract_indices.h> // 滤波器头文件
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/project_inliers.h> // 投影头文件
#include "..\refineLine\utils.hpp"


// 定义自己的pcl数据类型

struct EIGEN_ALIGN16 MyPointType    // enforce SSE padding for correct memory alignment
{
	PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
	float scalar_Label; // scalar_preds
	//float class; // scalar_class
	PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};
POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointType,		// here we assume a XYZ + "test" (as fields)  注册点类型宏
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, scalar_Label, scalar_Label)
	//(float, class, class)
)


template<class PointT>
using Clouds_vector = std::vector<pcl::PointCloud<PointT>, Eigen::aligned_allocator<PointT> >;

using Coff_vector  = std::vector<pcl::ModelCoefficients>;


enum class Roomlabel
{
	ceiling,
	floor,
	outwall,
	window,
	door,
	clutter,
	roomwall,
	hallwall
};
enum class S3DISLabel
{
	ceiling,
	floor,
	wall,
	beam,
	column,
	window,
	door,
	table,
	chair,
	sofa,
	bookcase,
	board,
	clutter

};

// 设置pcInfo
//---------------------------------------------------------------------------------
template<class T>
void setPCInfo(pcl::PointCloud<T>& cloud, pcInfo& info)
{
	T minp, maxp;
	pcl::getMinMax3D<T>(cloud, minp, maxp);
	info.minP.x = minp.x; info.minP.y = minp.y;
	info.maxP.x = maxp.x; info.maxP.y = maxp.y;
	info.map_origin = cv::Point2d(info.minP.x, info.minP.y);
	info.height = (int)(info.maxP.y - info.minP.y) / info.map_resolution;
	info.width = (int)(info.maxP.x - info.minP.x) / info.map_resolution;
}
void savePCInfo(std::string path, pcInfo& info);
void readPCInfo(std::string path, pcInfo& info);
//---------------------------------------------------------------------------------

// 读取 文件夹下所有文件名
//---------------------------------------------------------------------------------
void getFiles(const std::string& path, std::vector<std::string>& files);
void getFiles(const std::string& path, std::vector<std::string>& files)
{
	//文件句柄  
	long long hFile = 0;
	//文件信息，_finddata_t需要io.h头文件  
	struct _finddata_t fileinfo;
	std::string p;
	int i = 0;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之  
			//如果不是,加入列表  
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				//if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					//getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}
//---------------------------------------------------------------------------------

// 读取 ply文件
//---------------------------------------------------------------------------------
template<class T>
class PLY_IO {
	typedef typename pcl::PointCloud<T>::Ptr PointCloudPtr;
public:
	static void read_ply(std::string file_path, PointCloudPtr cloud)
	{
		if (pcl::io::loadPLYFile(file_path, *cloud) < 0)
			PCL_ERROR("点云读取失败！\n");
	}

	static void save_ply(std::string file_path, PointCloudPtr cloud) {
		if(pcl::io::savePLYFile(file_path, *cloud))
			PCL_ERROR("点云保存成功！\n");
	}
};

//---------------------------------------------------------------------------------

// 点云投影
//---------------------------------------------------------------------------------
template<class T>
class pcl_projection
{
	typedef typename pcl::PointCloud<T>::Ptr PointCloudPtr;

public:
	// 构造函数，可向 xoy面 快速投影
	pcl_projection()
	{
		/*
		* 默认向 xoy 平面 投影
		*/
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		coefficients->values.push_back(0.0);
		coefficients->values.push_back(0.0);
		coefficients->values.push_back(1.0);
		coefficients->values.push_back(0.0);
		this->coefficients = coefficients;
	}

	// 构造函数，可向三个平面快速投影
	pcl_projection(std::string plane)
	{
		if (plane == "xz")
		{
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
			coefficients->values.push_back(0.0);
			coefficients->values.push_back(1.0);
			coefficients->values.push_back(0.0);
			coefficients->values.push_back(0.0);
			this->coefficients = coefficients;
		}
		else if (plane == "yz")
		{
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
			coefficients->values.push_back(1.0);
			coefficients->values.push_back(0.0);
			coefficients->values.push_back(0.0);
			coefficients->values.push_back(0.0);
			this->coefficients = coefficients;
		}

	}

	// 构造函数，向任一平面投影
	pcl_projection(pcl::ModelCoefficients::Ptr coefficients)
	{
		this->coefficients = coefficients;
	}

public:
	void project(PointCloudPtr cloud, PointCloudPtr cloud_projected)
	{
		pcl::ProjectInliers<T> proj;
		proj.setInputCloud(cloud);
		proj.setModelCoefficients(coefficients);
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.filter(*cloud_projected);
	}
private:
	pcl::ModelCoefficients::Ptr coefficients;
};
//---------------------------------------------------------------------------------

// 高度切片
//---------------------------------------------------------------------------------
template <class PointT>
class PassThroughFilter
{
	typedef typename pcl::PointCloud<PointT>::Ptr pcPtr;
public:
	static void filter(pcPtr cloud, pcPtr filter_cloud, double thre_low, double thre_high, const bool& flag_in=false)
	{
		pcl::PassThrough<PointT> passthrough;
		passthrough.setInputCloud(cloud);//输入点云
		passthrough.setFilterFieldName("z");//对z轴进行操作
		passthrough.setFilterLimits(thre_low, thre_high);//设置直通滤波器操作范围
		passthrough.setFilterLimitsNegative(flag_in);//true表示保留范围外，false表示保留范围内
		passthrough.filter(*filter_cloud);//执行滤波，过滤结果保存在 cloud_after_PassThrough
		std::cout << "直通滤波后点云数据点数：" << filter_cloud->size() << std::endl;
	}
};
//---------------------------------------------------------------------------------

// 去除噪声----半径滤波
//---------------------------------------------------------------------------------
template<class PointT>
void RadiusOutlierFilter(const pcl::PointCloud<PointT>& cloud, pcl::PointCloud<PointT>& filter_cloud,
						const double& radius, const int& thre_count);
template<class PointT>
void RadiusOutlierFilter(const pcl::PointCloud<PointT>& cloud, pcl::PointCloud<PointT>& filter_cloud, const double& radius, const int& thre_count)
{
	pcl::RadiusOutlierRemoval<PointT> radiusoutlier;  //创建滤波器

	radiusoutlier.setInputCloud(cloud);    //设置输入点云
	radiusoutlier.setRadiusSearch(radius);     //设置radius为100的范围内找临近点
	radiusoutlier.setMinNeighborsInRadius(thre_count); //设置查询点的邻域点集数小于2的删除

	radiusoutlier.filter(filter_cloud);
	std::cout << "半径滤波后点云数据点数：" << filter_cloud.size() << std::endl;
}
//---------------------------------------------------------------------------------

// 下采样----格网滤波
//---------------------------------------------------------------------------------
template<class T>
class pcl_downSample
{
	typedef typename pcl::PointCloud<T>::Ptr PointCloudPtr;
public:
	/***************************************************************
		*  @brief     体素下采样
		*  @param     cloud：输入点云
		*  @param     cloud_filter：输出点云
		*  @param     l/w/h：采样体素长宽高
		*  @note      备注
		*  @Sample usage:    pcl_downSample<pcl::PointXYZ>::voxel_grid(cloud_project, cloud_sample, 0.2, 0.2, 0.2);
		**************************************************************/
	static void voxel_grid(PointCloudPtr cloud, PointCloudPtr cloud_filter, float l, float w, float h)
	{
		std::cout << "开始体素滤波" << std::endl;
		std::cout << "滤波前点云数目：" << cloud->size() << std::endl;
		pcl::VoxelGrid<T> filter;
		filter.setInputCloud(cloud);
		// 设置体素栅格的大小为 1x1x1cm 0.01f
		// filter.setLeafSize(5.0f, 5.0f, 5.0f);
		filter.setLeafSize(l, w, h);
		filter.filter(*cloud_filter);
		std::cout << "滤波后点云数目：" << cloud_filter->size() << std::endl;
	}
};
//---------------------------------------------------------------------------------
 
// 生成语义地图.0-->房间语义，1-->墙面语义
//---------------------------------------------------------------------------------
template<class PointT>
cv::Mat genLabelMap(pcl::PointCloud<PointT>& cloud, pcInfo info, int flag);
template<class PointT>
inline cv::Mat genLabelMap(pcl::PointCloud<PointT>& cloud, pcInfo info, int flag)
{
	cv::Mat map = cv::Mat::zeros(info.height, info.width, CV_8UC1);
	if (flag == 0)
	{
		for (int iter = 0; iter < cloud.size(); iter++)
		{
			int i = int((cloud[iter].x - info.minP.x) / info.map_resolution);
			if (i < 0 || i >= info.width) continue;

			int j = int((cloud[iter].y - info.minP.y) / info.map_resolution);
			if (j < 0 || j >= info.height - 1) continue;
			// if (cloud->points[iter].label != 0 && cloud->points[iter].label != 2 && cloud->points[iter].label != 3 && cloud->points[iter].label != 4)
			//if (cloud[iter].label == 2 || cloud[iter].label == 4 || cloud[iter].label == 5 || cloud[iter].label == 6)
			map.at<uchar>(j, i) = 255;
		}
	}
	else
	{
		for (int iter = 0; iter < cloud.size(); iter++)
		{
			int i = int((cloud[iter].x - info.minP.x) / info.map_resolution);
			if (i < 0 || i >= info.width) continue;

			int j = int((cloud[iter].y - info.minP.y) / info.map_resolution);
			if (j < 0 || j >= info.height - 1) continue;
			// if (cloud->points[iter].label != 0 && cloud->points[iter].label != 2 && cloud->points[iter].label != 3 && cloud->points[iter].label != 4)
			if (cloud[iter].label == 2 || cloud[iter].label == 4 || cloud[iter].label == 5 || cloud[iter].label == 6)
				map.at<uchar>(j, i) = 255;
		}
	}
	return map;
}
//---------------------------------------------------------------------------------

// 提取特定语义
//---------------------------------------------------------------------------------
void filterLabel(pcl::PointCloud<MyPointType>& in_cloud, pcl::PointCloud<MyPointType>& out_cloud, std::vector<int> labels);
//---------------------------------------------------------------------------------

// RANSAC 提取直线
//---------------------------------------------------------------------------------
template<class PointT>
class RANSACLines {
	typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
public:
	Clouds_vector<PointT> lines;
	Coff_vector coff;
public:
	void genWallLines(PointCloudPtr cloud, int maxIter, float maxMissPoint, float distanceThreshold)
	{
		PointCloudPtr outliers(new pcl::PointCloud<PointT>);
		pcl::copyPointCloud<PointT, PointT>(*cloud, *outliers);

		//-----------------------------拟合直线-----------------------------
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());


		//创建分割对象
		pcl::SACSegmentation<PointT> seg;
		seg.setOptimizeCoefficients(true);

		seg.setModelType(pcl::SACMODEL_LINE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(maxIter);
		seg.setDistanceThreshold(distanceThreshold);

		//创建滤波器对象
		pcl::ExtractIndices<PointT> extract;
		int i = 0, nr_points = (int)cloud->size();


		//创建视窗的标准代码
		while (outliers->points.size() > maxMissPoint * nr_points)
		{
			// 从余下的点云中分割最大平面组成部分
			seg.setInputCloud(outliers);
			seg.segment(*inliers, *coefficients);
			coff.push_back(*coefficients);
			if (inliers->indices.size() == 0 && i!=0)
			{
				std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}
			// 分离内层
			PointCloudPtr cloud_tmp(new pcl::PointCloud<PointT>);
			extract.setInputCloud(outliers);
			extract.setIndices(inliers);
			extract.setNegative(false);
			extract.filter(*cloud_tmp);

			lines.push_back(*cloud_tmp);
			// 创建滤波器对象
			extract.setNegative(true);
			extract.filter(*cloud_tmp);
			*outliers = *cloud_tmp;
			std::cout << "提取到第" << i << "条直线" << std::endl;
			i++;
		}
		std::cout << "分割: " << i << "条直线" << std::endl;
	}
	void readCoff(std::string path)
	{
		std::ifstream in;
		in.open(path, ios::in);
		pcl::ModelCoefficients coff_tmp;
		coff_tmp.values.resize(6);
		double x, y, z, a, b, c;
		while (in >> x >> y >> z >> a >> b >> c) {
			coff_tmp.values[0] = x; coff_tmp.values[1] = y; coff_tmp.values[2] = z;
			coff_tmp.values[3] = a; coff_tmp.values[4] = b; coff_tmp.values[5] = c;
			coff.push_back(coff_tmp);
		}
		in.close();
	}
	void readLines(std::string path) {
		std::vector<std::string> files;
		getFiles(path, files);
		auto it = files.begin();
		while (it != files.end())
		{
			PointCloudPtr cloud(new pcl::PointCloud<PointT>);
			if (pcl::io::loadPCDFile(*it, *cloud) < 0)
				std::cerr << *it << "* " << "load error" << std::endl;
			lines.push_back(*cloud);
			it++;
		}
	}
};
// void genWallLines(pcl::PointCloud<PointT>& cloud, Clouds_vector<PointT> lines, Coff_vector coff,  int maxIter, float maxMissPoint, float distanceThreshold);
//---------------------------------------------------------------------------------

// 保存读取 Clouds_vector
//---------------------------------------------------------------------------------
template<class PointT>
class Clouds_vector_IO {
public:
	static void save(std::string path, Clouds_vector<PointT>& lines)
	{
		int i = 0;
		for (auto line : lines)
			pcl::io::savePCDFileASCII(path + "/line_cloud_" + std::to_string(i++) + ".pcd", line);

	}

	static void read(std::string path, Clouds_vector<PointT>& lines)
	{
		typedef typename pcl::PointCloud<PointT>::Ptr pcPtr;
		std::vector<std::string> files;
		getFiles(path, files);
		auto it = files.begin();
		while (it != files.end())
		{
			pcPtr cloud(new pcl::PointCloud<PointT>);
			if (pcl::io::loadPCDFile(*it, *cloud) < 0)
				std::cerr << *it << "* " << "load error" << std::endl;
			lines.push_back(*cloud);
			it++;
		}

	}
};
//---------------------------------------------------------------------------------

// 保存直线因数Coff_vector
//---------------------------------------------------------------------------------
void saveCoff_vector(std::string path, Coff_vector coff);
void saveCoff_vector(std::string path, Coff_vector coff)
{
	std::ofstream out;
	out.open(path);
	for (auto c : coff)
		out << c.values[0] << " " << c.values[1] << " " << c.values[2] << " "
		<< c.values[3] << " " << c.values[4] << " " << c.values[5] << std::endl;
	out.close();

}
//---------------------------------------------------------------------------------

// 读取直线因数Coff_vector
//---------------------------------------------------------------------------------
void readCoff_vector(std::string path, Coff_vector& coff);
void readCoff_vector(std::string path, Coff_vector& coff)
{
	std::ifstream in;
	in.open(path, ios::in);
	pcl::ModelCoefficients coff_tmp;
	coff_tmp.values.resize(6);
	double x, y, z, a, b, c;
	while (in >> x >> y >> z >> a >> b >> c) {
		coff_tmp.values[0] = x; coff_tmp.values[1] = y; coff_tmp.values[2] = z;
		coff_tmp.values[3] = a; coff_tmp.values[4] = b; coff_tmp.values[5] = c;
		coff.push_back(coff_tmp);
	}
	in.close();

}
//---------------------------------------------------------------------------------

// 生成适合的 场景线段, 保存
//---------------------------------------------------------------------------------
template <typename PointT>
void genSegmens(Clouds_vector<PointT> lines, pcInfo info, std::vector<Segment>& res);
void genSegmens(std::vector<Segment>& lines, pcInfo info, std::vector<Segment>& res);
void saveSegmens(std::string path, std::vector<Segment>& res);
std::vector<Segment> readSegmens(std::string path);
//---------------------------------------------------------------------------------
 
// MeanShift 简化直线
//---------------------------------------------------------------------------------
//  Mean_Shift ms;
//	ms.setInputLines<PointT>(rc.load_lines);
//	ms.setKernel_bandwidth(5);
//	ms.fitLines();
//	ms.view<PointT>(cloud);
//  ms.saveLines("./res/sim_lines.txt");
//
class Mean_Shift
{
public:
	Mean_Shift();
	void setInputLines(std::vector<Segment> cloud_lines);
	void setKernel_bandwidth(double width) { kernel_bandwidth = width; };
	void fitLines();
	template<class PointT>
	void view(pcl::PointCloud<PointT> cloud);
	void saveLines(std::string filename);
public:
	

	MeanShift* msp;
	double kernel_bandwidth;
	// 简化结果：
	// model：简化后直线端点 support：原直线
	std::vector<Cluster> shifted_source;
	std::vector<Cluster> shifted_target;
	std::vector <std::vector<double> > source, target;
	std::vector <std::vector<double> > source_shift, target_shift;
	std::vector <std::vector<double> > segment_simple;
	std::vector<Segment> seg;
};

Mean_Shift::Mean_Shift()
{
	msp = new MeanShift();
	kernel_bandwidth = 1;
}

void Mean_Shift::setInputLines(std::vector<Segment> cloud_lines)
{
	for (auto s : cloud_lines)
	{
		source.push_back(std::vector<double>{CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y())});
		target.push_back(std::vector<double>{CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y())});
	}
	
}
void Mean_Shift::fitLines()
{
	shifted_source = msp->cluster(source, kernel_bandwidth);
	shifted_target = msp->cluster(target, kernel_bandwidth);

	// 找出 聚类后 原起点 的 中心点
	for (int point = 0; point < source.size(); point++) {
		for (int cluster = 0; cluster < shifted_source.size(); cluster++) {
			auto source_res = std::find(shifted_source[cluster].original_points.begin(), shifted_source[cluster].original_points.end(), source[point]);
			if (source_res != shifted_source[cluster].original_points.end()) {
				source_shift.push_back(shifted_source[cluster].shifted_points[0]);
				break;
			}				
		}
	}
	// 找出 聚类后 原终点 的 中心点
	for (int point = 0; point < target.size(); point++) {
		for (int cluster = 0; cluster < shifted_target.size(); cluster++) {
			auto target_res = std::find(shifted_target[cluster].original_points.begin(), shifted_target[cluster].original_points.end(), target[point]);
			if (target_res != shifted_target[cluster].original_points.end()) {
				target_shift.push_back(shifted_target[cluster].shifted_points[0]);
				break;
			}
		}
	}


	for (int i = 0; i < source_shift.size(); i++) {
		std::vector<double> s{ source_shift[i][0], source_shift[i][1],target_shift[i][0], target_shift[i][1] };
		segment_simple.push_back(s);
	}
	std::sort(segment_simple.begin(), segment_simple.end());
	std::cout << "线段去重前：" << segment_simple.size()<< std::endl;
	segment_simple.erase(std::unique(segment_simple.begin(), segment_simple.end()), segment_simple.end());
	std::cout << "线段去重后：" << segment_simple.size() << std::endl;

	for (int i = 0; i < segment_simple.size(); i++) {
		seg.push_back(Segment(Point(segment_simple[i][0], segment_simple[i][1]), Point(segment_simple[i][2], segment_simple[i][3])));
	}

	// std::cout << "fit lines num: " << shifted_values.size() << std::endl;
}
template<class PointT>
void Mean_Shift::view(pcl::PointCloud<PointT> cloud)
{
	//----------------------------结果可视化-----------------------------
	//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Fit line Viewer"));

	//viewer->addPointCloud<pcl::PointXYZ>(cloud.makeShared(), "cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	//if (!shifted_values.empty())
	//{
	//	for (int i = 0; i < shifted_values.size(); i++)
	//	{
	//		//随机种子
	//		srand((unsigned)i);
	//		pcl::PointXYZ minP, maxP;
	//		minP.x = shifted_values[i].mode[0]; minP.y = shifted_values[i].mode[1];
	//		maxP.x = shifted_values[i].mode[2]; maxP.y = shifted_values[i].mode[3];
	//		std::cout << rand() / 255.0 << std::endl;
	//		viewer->addLine(minP, maxP, rand() / 255.0, rand() / 255.0, rand() / 255.0, "line_" + std::to_string(i), 0);
	//	}
	//}

	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//}
}
void Mean_Shift::saveLines(std::string filename)
{
	/*std::ofstream outfile;
	outfile.open(filename);
	if (!shifted_values.empty())
	{
		for (int i = 0; i < shifted_values.size(); i++)
			outfile << shifted_values[i].mode[0] << " " << shifted_values[i].mode[1] << " " << shifted_values[i].mode[2] << " " << shifted_values[i].mode[3] << std::endl;
	}
	outfile.close();*/
}
//---------------------------------------------------------------------------------

// 可视化 Arrangement在分割图map上
//---------------------------------------------------------------------------------
template <class PointT>
class pcl_vis {
	typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
public:
	static void show_Ransac_lines(PointCloudPtr cloud, Coff_vector lines) {
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("lines Viewer"));
		
		pcl::visualization::PointCloudColorHandlerRandom<PointT> rgb(cloud);
		viewer->addPointCloud(cloud, rgb,"cloud");
		int i = 0;
		for (auto c : lines) {
			viewer->addLine(c, "Line_" + std::to_string(i));
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Line_" + std::to_string(i));
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, 0, 0, "Line_" + std::to_string(i));
			i++;
		}
		
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
		}
	}

	static void show_Ransac_lines(Clouds_vector<PointT> lines)
	{
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("lines Viewer"));
		int i = 0;
		for (auto line : lines)
		{
			pcl::visualization::PointCloudColorHandlerRandom<PointT> rgb(line.makeShared());
			viewer->addPointCloud(line.makeShared(), rgb, "line"+ std::to_string(i));
			i++;
		}
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
		}
	}

	static void showPointCloud(PointCloudPtr cloud) {
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("cloud Viewer"));
		viewer->addPointCloud<PointT>(cloud, "cloud");
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
		}
	}
};
//---------------------------------------------------------------------------------

void savePCInfo(std::string path, pcInfo& info) {
	std::ofstream out(path);
	out << info.minP.x << " " << info.minP.y << " " << info.maxP.x << " " << info.maxP.y << std::endl;
	out << info.map_origin.x << " " << info.map_origin.y << std::endl;
	out << info.map_resolution << std::endl
		<< info.width << std::endl
		<< info.height << std::endl;
	out.close();
}

void readPCInfo(std::string path, pcInfo& info) {
	std::ifstream in(path);
	pcl::PointXYZ minp, maxp;
	cv::Point2d ori;
	in >> minp.x >> minp.y >> maxp.x >> maxp.y;
	info.minP = minp; info.maxP = maxp;
	in >> ori.x >> ori.y;
	info.map_origin = ori;
	in >> info.map_resolution >> info.width >> info.height;
	in.close();
}

template <typename PointT>
void genSegmens(Clouds_vector<PointT> lines, pcInfo info, std::vector<Segment>& res)
{
	std::vector<Line> CGAL_lines; // 输入的直线
	std::vector<Segment> bbox; // 包围盒
	std::vector<Segment> segments_intersection; // 相交完的直线

	// 转换为CGAL直线
	for (auto line : lines)
	{
		PointT minP, maxP;
		pcl::getMinMax3D<PointT>(line, minP, maxP);
		CGAL_lines.push_back(Line(Point(minP.x, minP.y), Point(maxP.x, maxP.y)));
	}
		

	// 初始化包围盒
	bbox.push_back(Segment(Point(info.minP.x, info.minP.y), Point(info.minP.x, info.maxP.y)));
	bbox.push_back(Segment(Point(info.minP.x, info.minP.y), Point(info.maxP.x, info.minP.y)));
	bbox.push_back(Segment(Point(info.maxP.x, info.maxP.y), Point(info.minP.x, info.maxP.y)));
	bbox.push_back(Segment(Point(info.maxP.x, info.maxP.y), Point(info.maxP.x, info.minP.y)));

	// 计算每个直线与包围盒的交点，作为其线段端点
	for (auto in_line : CGAL_lines)
	{
		std::vector<Point> seg;
		for (auto bbox_line : bbox)
		{
			auto result = CGAL::intersection(bbox_line, in_line);
			if (result) {
				/* not empty */
				if (const Point* p = boost::get<Point>(&*result)) {
					std::cout << "Points intersect:" << std::endl;
					std::cout << *p << std::endl;
					seg.push_back(*p);
				}
				else {
					std::cout << "直线与包围盒共线" << std::endl;
					const Segment* s = boost::get<Segment>(&*result);
					std::cout << *s << std::endl;
				}
			}
			else {
				/* empty intersection */
				std::cout << "None intersection!" << std::endl;
			}
		}
		if (seg.size() == 2)
			segments_intersection.push_back(Segment(seg[0], seg[1]));
	}
	res = segments_intersection;
}

void genSegmens(std::vector<Segment>& lines, pcInfo info, std::vector<Segment>& res)
{
	std::vector<Line> CGAL_lines; // 输入的直线
	std::vector<Segment> bbox; // 包围盒
	std::vector<Segment> segments_intersection; // 相交完的直线

	
	// 转换为CGAL直线
	for (auto line : lines)
		CGAL_lines.push_back(Line(line));


	// 初始化包围盒
	bbox.push_back(Segment(Point(info.minP.x, info.minP.y), Point(info.minP.x, info.maxP.y)));
	bbox.push_back(Segment(Point(info.minP.x, info.minP.y), Point(info.maxP.x, info.minP.y)));
	bbox.push_back(Segment(Point(info.maxP.x, info.maxP.y), Point(info.minP.x, info.maxP.y)));
	bbox.push_back(Segment(Point(info.maxP.x, info.maxP.y), Point(info.maxP.x, info.minP.y)));

	// 计算每个直线与包围盒的交点，作为其线段端点
	for (auto in_line : CGAL_lines)
	{
		std::vector<Point> seg;
		for (auto bbox_line : bbox)
		{
			auto result = CGAL::intersection(bbox_line, in_line);
			if (result) {
				/* not empty */
				if (const Point* p = boost::get<Point>(&*result)) {
					std::cout << "Points intersect:" << std::endl;
					std::cout << *p << std::endl;
					seg.push_back(*p);
				}
				else {
					std::cout << "直线与包围盒共线" << std::endl;
					const Segment* s = boost::get<Segment>(&*result);
					std::cout << *s << std::endl;
				}
			}
			else {
				/* empty intersection */
				std::cout << "None intersection!" << std::endl;
			}
		}
		if (seg.size() == 2)
			segments_intersection.push_back(Segment(seg[0], seg[1]));
	}
	res = segments_intersection;
}

void saveSegmens(std::string path, std::vector<Segment>& res) {
	std::ofstream out(path);
	for (auto s : res)
		out << s.source() << " " << s.target() << std::endl;
	out.close();
}

std::vector<Segment> readSegmens(std::string path) {
	std::ifstream in(path);
	std::vector<Segment> segments;
	Point source, target;
	while (in >> source >> target)
		segments.push_back(Segment(source, target));
	in.close();
	return segments;
}



void filterLabel(pcl::PointCloud<MyPointType>& in_cloud, pcl::PointCloud<MyPointType>& out_cloud, std::vector<int> labels) {
	for (const auto p : in_cloud.points) {
		if (std::find(labels.begin(), labels.end(), (int)p.scalar_Label) != labels.end())
			out_cloud.push_back(p);
	}
}