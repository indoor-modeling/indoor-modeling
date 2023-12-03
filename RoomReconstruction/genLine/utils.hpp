#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h> // 拟合模型系数 头文件
#include <pcl/segmentation/sac_segmentation.h> // ransac分割头文件
#include <pcl/filters/extract_indices.h> // 滤波器头文件




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

// 点云分割预备信息
struct pcInfo
{
	pcl::PointXYZ minP, maxP;
	cv::Point2d map_origin;
	float map_resolution;
	int width, height;
};

template<class T>
using Clouds_vector = std::vector<pcl::PointCloud<T>, Eigen::aligned_allocator<T> >;

using Coff_vector = std::vector<pcl::ModelCoefficients>;

// 读取 文件夹下所有文件名
//---------------------------------------------------------------------------------
void getFiles(const std::string& path, std::vector<std::string>& files);
//---------------------------------------------------------------------------------

// 读取，保存Clouds_vector文件
//---------------------------------------------------------------------------------
template<class T>
void save_Cloud_vector(std::string path, Clouds_vector<T>& lines);
template<class T>
void read_Cloud_vector(std::string path, Clouds_vector<T>& lines);
//---------------------------------------------------------------------------------

// 读取，保存PLY文件
//---------------------------------------------------------------------------------
template<class T>
void read_ply(std::string file_path, pcl::PointCloud<T>& cloud);
template<class T>
void save_ply(std::string file_path, pcl::PointCloud<T>& cloud);
//---------------------------------------------------------------------------------


// RANSAC 提取直线
//---------------------------------------------------------------------------------
template<class T>
void genWallLines(pcl::PointCloud<T>& cloud, Clouds_vector<T>& lines,int maxIter, float maxMissPoint, float distanceThreshold);
//---------------------------------------------------------------------------------

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

template<class T>
inline void save_Cloud_vector(std::string path, Clouds_vector<T>& lines)
{
	
	int i = 0;
	for (auto line : lines)
		pcl::io::savePCDFileASCII(path + "/line_cloud_" + std::to_string(i++) + ".pcd", line);
	
}

template<class T>
inline void read_Cloud_vector(std::string path, Clouds_vector<T>& lines)
{
	typedef typename pcl::PointCloud<T>::Ptr pcPtr;
	std::vector<std::string> files;
	getFiles(path, files);
	auto it = files.begin();
	while (it != files.end())
	{
		pcPtr cloud(new pcl::PointCloud<T>);
		if (pcl::io::loadPCDFile(*it, *cloud) < 0)
			std::cerr << *it << "* " << "load error" << std::endl;
		lines.push_back(*cloud);
		it++;
	}
}

template<class T>
void read_ply(std::string file_path, pcl::PointCloud<T>& cloud) {
	if (pcl::io::loadPLYFile(file_path, cloud) < 0)
		PCL_ERROR("点云读取失败！\n");
}
template<class T>
void save_ply(std::string file_path, pcl::PointCloud<T>& cloud) {
	if (pcl::io::savePLYFile(file_path, cloud))
		PCL_ERROR("点云保存成功！\n");
}

template<class T>
void genWallLines(pcl::PointCloud<T>& cloud,Clouds_vector<T>& lines, int maxIter, float maxMissPoint, float distanceThreshold)
{
	pcl::PointCloud<T> outliers;
	pcl::copyPointCloud<T, T>(cloud, outliers);

	//-----------------------------拟合直线-----------------------------
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());


	//创建分割对象
	pcl::SACSegmentation<T> seg;
	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_LINE);
	seg.setMethodType(pcl::SAC_RANSAC);
	
	seg.setMaxIterations(maxIter);
	seg.setDistanceThreshold(distanceThreshold);

	//创建滤波器对象
	pcl::ExtractIndices<T> extract;
	int i = 0, nr_points = (int)cloud.size();


	//创建视窗的标准代码
	while (outliers.size() > maxMissPoint * nr_points)
	{
		// 从余下的点云中分割最大平面组成部分
		seg.setInputCloud(outliers.makeShared());
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0 && i != 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}
		// 分离内层
		pcl::PointCloud<T> cloud_tmp;
		extract.setInputCloud(outliers.makeShared());
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(cloud_tmp);

		lines.push_back(cloud_tmp);
		// 创建滤波器对象
		extract.setNegative(true);
		extract.filter(cloud_tmp);
		outliers = cloud_tmp;
		std::cout << "提取到第" << i << "条直线" << std::endl;
		i++;
	}
	std::cout << "分割: " << i << "条直线" << std::endl;
}

void drawline(cv::Mat map, cv::Point2d start, cv::Point2d end) {
	int thick = 1;
	int lineType = 8;
	cv::RNG rng(time(0));
	cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
	cv::line(map, start, end, color, thick, lineType);
}


void drawLines(pcl::PointCloud<MyPointType>& cloud, Clouds_vector<MyPointType>& lines) {
	MyPointType minp, maxp;
	pcl::getMinMax3D<MyPointType>(cloud, minp, maxp);
	int map_resolution = 0.05;
	auto map_origin = cv::Point2d(minp.x, minp.y);
	auto height = (int)((maxp.y - minp.y) / map_resolution) + 1;
	auto width = (int)((maxp.x - minp.x) / map_resolution) + 1;

	cv::Mat map = cv::Mat::zeros(width, height, CV_8UC3);
	for (auto line : lines) {
		MyPointType min, max;
		pcl::getMinMax3D<MyPointType>(line, min, max);
		cv::Point2d s = cv::Point2d(min.x, min.y);
		cv::Point2d e = cv::Point2d(max.x, max.y);
		drawline(map, s, e);
	}
	cv::imshow("sda", map);
	cv::waitKey(0);

}