#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

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

// 定义自己的pcl数据类型

struct EIGEN_ALIGN16 MyPointType    // enforce SSE padding for correct memory alignment
{
	PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
	float scalar_class; // scalar_preds //scalar_Label
	//int label;
	//float class; // scalar_class
	PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};
POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointType,		// here we assume a XYZ + "test" (as fields)  注册点类型宏
	(float, x, x)
	(float, y, y)
	(float, z, z)
	//(int, label, label)
	(float, scalar_class, scalar_class)
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

template<class PointT>
using Clouds_vector = std::vector<pcl::PointCloud<PointT>, Eigen::aligned_allocator<PointT> >;

using Coff_vector = std::vector<pcl::ModelCoefficients>;

// 语义标签
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

// 设置，保存，读取pcInfo
//---------------------------------------------------------------------------------
template<class T>
void setPCInfo(pcl::PointCloud<T>& cloud, pcInfo& info);
void savePCInfo(std::string path, pcInfo& info);
void readPCInfo(std::string path, pcInfo& info);
//---------------------------------------------------------------------------------

// 读取 文件夹下所有文件名
//---------------------------------------------------------------------------------
void getFiles(const std::string & path, std::vector<std::string>&files);
//---------------------------------------------------------------------------------

// 读取，保存PLY文件
//---------------------------------------------------------------------------------
template<class T>
void read_ply(std::string file_path, pcl::PointCloud<T>& cloud);
template<class T>
void save_ply(std::string file_path, pcl::PointCloud<T>& cloud);
//---------------------------------------------------------------------------------

// 点云投影
//---------------------------------------------------------------------------------
class pcl_projection
{
public:
	// 构造函数，可向 xoy面 快速投影
	pcl_projection();
	// 构造函数，可向三个平面快速投影
	pcl_projection(std::string plane);
	// 构造函数，向任一平面投影
	pcl_projection(pcl::ModelCoefficients::Ptr coefficients);
	
	template<class T>
	void project(pcl::PointCloud<T>& cloud, pcl::PointCloud<T>& cloud_projected);
private:
	pcl::ModelCoefficients::Ptr coefficients;
	
};
//---------------------------------------------------------------------------------

// 高度切片
//---------------------------------------------------------------------------------
template <class T>
void PassThroughFilter(pcl::PointCloud<T>& cloud, pcl::PointCloud<T>& filter_cloud, double thre_low, double thre_high, const bool& flag_in = false);
//---------------------------------------------------------------------------------

// 去除噪声----半径滤波
//---------------------------------------------------------------------------------
template<class T>
void RadiusOutlierFilter(const pcl::PointCloud<T>& cloud, pcl::PointCloud<T>& filter_cloud, const double& radius, const int& thre_count);
//---------------------------------------------------------------------------------

// 下采样----格网滤波
//---------------------------------------------------------------------------------
template<class T>
void voxel_grid(pcl::PointCloud<T>& cloud, pcl::PointCloud<T>& cloud_filter, float l, float w, float h);
//---------------------------------------------------------------------------------

// 提取特定语义
//---------------------------------------------------------------------------------
void filterLabel(pcl::PointCloud<MyPointType>& in_cloud, pcl::PointCloud<MyPointType>& out_cloud, std::vector<int> labels);
//---------------------------------------------------------------------------------

// 生成语义地图
/*
* @param inverseLabel 默认False，提取labels；反之则去除labels，投影剩余的语义
*/
//---------------------------------------------------------------------------------
cv::Mat genSemanticMap(pcl::PointCloud<MyPointType>& in_cloud, pcInfo& info, std::vector<int> labels, bool inverseLabel=false);
//---------------------------------------------------------------------------------



template<class T>
void setPCInfo(pcl::PointCloud<T>& cloud, pcInfo& info)
{
	T minp, maxp;
	pcl::getMinMax3D<T>(cloud, minp, maxp);
	info.minP.x = minp.x; info.minP.y = minp.y;
	info.maxP.x = maxp.x; info.maxP.y = maxp.y;
	info.map_origin = cv::Point2d(info.minP.x, info.minP.y);
	info.height = (int)((info.maxP.y - info.minP.y) / info.map_resolution)+1;
	info.width = (int)((info.maxP.x - info.minP.x) / info.map_resolution)+1;
}

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
inline void PassThroughFilter(pcl::PointCloud<T>& cloud, pcl::PointCloud<T>& filter_cloud, double thre_low, double thre_high, const bool& flag_in)
{
	pcl::PassThrough<T> passthrough;
	passthrough.setInputCloud(cloud);//输入点云
	passthrough.setFilterFieldName("z");//对z轴进行操作
	passthrough.setFilterLimits(thre_low, thre_high);//设置直通滤波器操作范围
	passthrough.setFilterLimitsNegative(flag_in);//true表示保留范围外，false表示保留范围内
	passthrough.filter(filter_cloud);//执行滤波，过滤结果保存在 cloud_after_PassThrough
	std::cout << "直通滤波后点云数据点数：" << filter_cloud->size() << std::endl;
}

pcl_projection::pcl_projection() {
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

pcl_projection::pcl_projection(std::string plane) {
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
pcl_projection::pcl_projection(pcl::ModelCoefficients::Ptr coefficients)
{
		this->coefficients = coefficients;
}
template <class T>
void pcl_projection::project(pcl::PointCloud<T>& cloud, pcl::PointCloud<T>& cloud_projected) {
	
	pcl::ProjectInliers<T> proj;
	proj.setInputCloud(cloud.makeShared());
	proj.setModelCoefficients(coefficients);
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.filter(cloud_projected);
	
}

template<class T>
void RadiusOutlierFilter(const pcl::PointCloud<T>& cloud, pcl::PointCloud<T>& filter_cloud, const double& radius, const int& thre_count) {
	pcl::RadiusOutlierRemoval<T> radiusoutlier;  //创建滤波器

	radiusoutlier.setInputCloud(cloud);    //设置输入点云
	radiusoutlier.setRadiusSearch(radius);     //设置radius为100的范围内找临近点
	radiusoutlier.setMinNeighborsInRadius(thre_count); //设置查询点的邻域点集数小于2的删除

	radiusoutlier.filter(filter_cloud);
	std::cout << "半径滤波后点云数据点数：" << filter_cloud.size() << std::endl;
}

template<class T>
inline void voxel_grid(pcl::PointCloud<T>& cloud, pcl::PointCloud<T>& cloud_filter, float l, float w, float h)
{
	std::cout << "开始体素滤波" << std::endl;
	std::cout << "滤波前点云数目：" << cloud.size() << std::endl;
	pcl::VoxelGrid<T> filter;
	filter.setInputCloud(cloud.makeShared());
	// 设置体素栅格的大小为 1x1x1cm 0.01f
	// filter.setLeafSize(5.0f, 5.0f, 5.0f);
	filter.setLeafSize(l, w, h);
	filter.filter(cloud_filter);
	std::cout << "滤波后点云数目：" << cloud_filter.size() << std::endl;
}

void filterLabel(pcl::PointCloud<MyPointType>& in_cloud, pcl::PointCloud<MyPointType>& out_cloud, std::vector<int> labels) {
	for (const auto p : in_cloud.points) {
		if (std::find(labels.begin(), labels.end(), (int)p.scalar_class) != labels.end())
			out_cloud.push_back(p);
	}
}

inline cv::Mat genSemanticMap(pcl::PointCloud<MyPointType>& in_cloud, pcInfo& info, std::vector<int> labels, bool inverseLabel)
{
	cv::Mat map = cv::Mat::zeros(info.height, info.width, CV_8UC1);
	std::cout <<"语义地图尺寸（宽、高）：" << map.size() << std::endl;
	
	for (int iter = 0; iter < in_cloud.points.size(); iter++) {
		int i = int((in_cloud.points[iter].x - info.map_origin.x) / info.map_resolution);
		if (i < 0 || i >= info.width) continue;

		int j = int((in_cloud.points[iter].y - info.map_origin.y) / info.map_resolution);
		if (j < 0 || j >= info.height - 1) continue;

		if (inverseLabel == false) {
			if (!labels.empty()) {
				if (std::find(labels.begin(), labels.end(), in_cloud.points[iter].scalar_class) != labels.end())
					map.at<uchar>(j, i) = 255;
			}
			else
					map.at<uchar>(j, i) = 255;
		}
		else if (inverseLabel == true) {
			if (!labels.empty()) {
				if (std::find(labels.begin(), labels.end(), in_cloud.points[iter].scalar_class) == labels.end())
					map.at<uchar>(j, i) = 255;
			}
			else
					map.at<uchar>(j, i) = 255;
		}		
	}
	return map;
}
