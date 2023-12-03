#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h> // 拟合模型系数 头文件
#include <pcl/filters/extract_indices.h> // 滤波器头文件


typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::FT                                        Number_type;
typedef Kernel::Point_2										Point;
typedef Kernel::Segment_2									Segment;
typedef Kernel::Line_2										Line;
typedef Kernel::Point_3										Point3;
typedef Kernel::Line_3										Line3;


// 点云分割预备信息
struct pcInfo
{
	pcl::PointXYZ minP, maxP;
	cv::Point2d map_origin;
	float map_resolution;
	int width, height;
};

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


template<class T>
using Clouds_vector = std::vector<pcl::PointCloud<T>, Eigen::aligned_allocator<T> >;

using Coff_vector = std::vector<pcl::ModelCoefficients>;

// 读取 文件夹下所有文件名
//---------------------------------------------------------------------------------
void getFiles(const std::string& path, std::vector<std::string>& files);
//---------------------------------------------------------------------------------

// 读取pcInfo文件
//---------------------------------------------------------------------------------
void readPCInfo(std::string path, pcInfo& info);
//---------------------------------------------------------------------------------

// 读取，保存Clouds_vector文件
//---------------------------------------------------------------------------------
template<class T>
void save_Cloud_vector(std::string path, Clouds_vector<T>& lines);
template<class T>
void read_Cloud_vector(std::string path, Clouds_vector<T>& lines);
//---------------------------------------------------------------------------------

// 生成适合的 场景线段, 保存
//---------------------------------------------------------------------------------
template <class T>
void genSegmens(Clouds_vector<T> lines, pcInfo info, std::vector<Segment>& res);
void genSegmens(std::vector<Segment>& lines, pcInfo info, std::vector<Segment>& res);
void genSegments(std::vector<std::vector<double>> lines, pcInfo info, std::vector<Segment>& res);
void saveSegmens(std::string path, std::vector<Segment>& res);
std::vector<Segment> readSegmens(std::string path);
//---------------------------------------------------------------------------------

// 归一化线段方向
// 以线段一端点A为原点，另一点B在A右方，则方向为A-->B,
// 若另一点B与A构成直线接近垂直，则比较y，y大的在上方
//---------------------------------------------------------------------------------
std::vector<Segment> regularSegementDirection(std::vector<Segment>& segments, float angle=5);
//---------------------------------------------------------------------------------
void genSegments(std::vector<std::vector<double>> lines, pcInfo info, std::vector<Segment>& res) {
	std::vector<Line> CGAL_lines; // 输入的直线
	std::vector<Segment> bbox; // 包围盒
	std::vector<Segment> segments_intersection; // 相交完的直线

	// 转换为CGAL直线
	for (auto line : lines)
	{
		CGAL_lines.push_back(Line(Point(line[0], line[1]), Point(line[5], line[4])));
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
		if (seg.size() == 2) {
			segments_intersection.push_back(Segment(seg[0], seg[1]));
			
		}
			
	}
	res = segments_intersection;
}
template <class T>
void genSegmens(Clouds_vector<T> lines, pcInfo info, std::vector<Segment>& res)
{
	std::vector<Line> CGAL_lines; // 输入的直线
	std::vector<Segment> bbox; // 包围盒
	std::vector<Segment> segments_intersection; // 相交完的直线

	// 转换为CGAL直线
	for (auto line : lines)
	{
		T minP, maxP;
		pcl::getMinMax3D<T>(line, minP, maxP);
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

	cv::Mat map = cv::imread("../area5/wall_map.png", 0);
	cv::Mat segments_on_map(map.size(), CV_8UC3);
	if (map.type() == CV_8UC1)
	{
		// 构造等大小的彩色图
		for (int y = 0; y < map.rows; y++)
			for (int x = 0; x < map.cols; x++)
			{
				auto gray = map.at<uchar>(y, x);
				cv::Vec3b& pixel = segments_on_map.at<cv::Vec3b>(y, x);
				if (gray == 0) {
					pixel[0] = cv::abs(255);
					pixel[1] = cv::abs(255);
					pixel[2] = cv::abs(255);
				}
				else
				{
					pixel[0] = cv::abs(106);
					pixel[1] = cv::abs(55);
					pixel[2] = cv::abs(66);
				}

			}
	}



	std::vector<Line> CGAL_lines; // 输入的直线
	std::vector<Segment> bbox; // 包围盒
	std::vector<Segment> segments_intersection; // 相交完的直线


	// 转换为CGAL直线,并且加入包围盒的直线
	for (auto line : lines)
		CGAL_lines.push_back(Line(line));

	// 初始化包围盒
	bbox.push_back(Segment(Point(info.minP.x, info.minP.y), Point(info.minP.x, info.maxP.y)));
	bbox.push_back(Segment(Point(info.minP.x, info.minP.y), Point(info.maxP.x, info.minP.y)));
	bbox.push_back(Segment(Point(info.maxP.x, info.maxP.y), Point(info.minP.x, info.maxP.y)));
	bbox.push_back(Segment(Point(info.maxP.x, info.maxP.y), Point(info.maxP.x, info.minP.y)));

	CGAL_lines.push_back(Line(bbox[0]));
	CGAL_lines.push_back(Line(bbox[1]));
	CGAL_lines.push_back(Line(bbox[2]));
	CGAL_lines.push_back(Line(bbox[3]));

	// 计算每个直线与包围盒的交点，作为其线段端点
	int ii = 0;
	for (auto in_line : CGAL_lines)
	{
		std::vector<Point> seg;
		for (auto bbox_line : bbox)
		{
			auto result = CGAL::intersection(bbox_line, in_line);
			if (result) {
				/* not empty */
				if (const Point* p = boost::get<Point>(&*result)) {
					//std::cout << "Points intersect:" << std::endl;
					//std::cout << *p << std::endl;
					seg.push_back(*p);
				}
				else {
					//std::cout << "直线与包围盒共线" << std::endl;
					const Segment* s = boost::get<Segment>(&*result);
					//std::cout << *s << std::endl;
				}
			}
			else {
				/* empty intersection */
				//std::cout << "None intersection!" << std::endl;
			}
		}
		if (seg.size() == 2) {
			std::cout << ii++ << std::endl;
			segments_intersection.push_back(Segment(seg[0], seg[1]));
			
			
				
				auto source = cv::Point2d(((CGAL::to_double(seg[0].x()) - info.map_origin.x) / info.map_resolution),
					((CGAL::to_double(seg[0].y()) - info.map_origin.y) / info.map_resolution));
				auto target = cv::Point2d(((CGAL::to_double(seg[1].x()) - info.map_origin.x) / info.map_resolution),
					(((CGAL::to_double(seg[1].y())) - info.map_origin.y) / info.map_resolution));
				cv::line(segments_on_map, source, target, cv::Scalar(92, 193, 252));
			
			/*cv::imshow("lie", segments_on_map);
			cv::waitKey();*/
		}
			
	}
	res = segments_intersection;
	
}

void saveSegmens(std::string path, std::vector<Segment>& res) {
	std::ofstream out(path);
	for (auto s : res)
		out << s.source() << " " << s.target() << std::endl;
	out.close();
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
		auto pos = (*it).find("ply");
		pcPtr cloud(new pcl::PointCloud<T>);
		if (pos == std::string::npos) {
			if (pcl::io::loadPCDFile(*it, *cloud) < 0)
				std::cerr << *it << "* " << "load error" << std::endl;
		}
		else {
			if (pcl::io::loadPLYFile(*it, *cloud) < 0)
				std::cerr << *it << "* " << "load error" << std::endl;
		}
			
		lines.push_back(*cloud);
		it++;
	}
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


inline std::vector<Segment> regularSegementDirection(std::vector<Segment>& segments, float angle)
{
	std::vector<Segment> regular_segments;
	// 创建竖直线
	Line3 vertical_line(Point3(0, 0, 0), Point3(0, 1, 0));

	for (auto s : segments) {
		// 将二维线段转换为三维直线
		Point3 source(s.source().x(), s.source().y(), 0);
		Point3 target(s.target().x(), s.target().y(), 0);
		Line3 l(source, target);
		// 计算与竖直线的夹角
		auto line_angle = CGAL::approximate_angle(vertical_line.to_vector(), l.to_vector());
		// 夹角超过阈值则比较x
		if (line_angle > angle) {
			if (s.source().x() < s.target().x())
				regular_segments.push_back(s);
			else
				regular_segments.push_back(Segment(s.target(), s.source()));
		}
		// 不超过则比较y
		else {
			if (s.source().y() < s.target().y())
				regular_segments.push_back(s);
			else
				regular_segments.push_back(Segment(s.target(), s.source()));
		}
	}
	return regular_segments;
}

