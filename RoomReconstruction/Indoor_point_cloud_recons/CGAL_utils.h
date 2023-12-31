#pragma once
#include "include.h"



// 检查Arrangement面是否为简单图形
//---------------------------------------------------------------------------------
bool face_is_simple(Face_handle f);
//---------------------------------------------------------------------------------

// 查询一个面的所有相邻面
//---------------------------------------------------------------------------------
std::vector<Face_handle> find_adjacent_face(Arrangement arr, Face_handle f);
//---------------------------------------------------------------------------------

// 查询两个面的相邻边
//---------------------------------------------------------------------------------
Halfedge_handle find_two_arr_face_adjacent_edge(Face_handle v1, Face_handle v2);
//---------------------------------------------------------------------------------

// 点投影到栅格地图
//---------------------------------------------------------------------------------
std::vector<Point> point2map(std::vector<Point> random_points, pcInfo info);
//---------------------------------------------------------------------------------

// 随机点语义获取
//---------------------------------------------------------------------------------
std::vector<int> point_label(std::vector<Point> map_point, cv::Mat map);
//---------------------------------------------------------------------------------

// 直线投影到栅格地图
//---------------------------------------------------------------------------------
std::vector<Point> line2map(Halfedge_handle he, float sample_num, pcInfo info, int thickness = 1);
//---------------------------------------------------------------------------------

// 直线点语义获取
//---------------------------------------------------------------------------------
std::vector<int> line_label(std::vector<Point> line_sample_point, cv::Mat map);
//---------------------------------------------------------------------------------

// 统计特定语义的点数目
//---------------------------------------------------------------------------------
int cal_label_num(std::vector<int> point_label, int label);
//---------------------------------------------------------------------------------


// 生成Cell单元
//---------------------------------------------------------------------------------
class cellDecom
{
public:
	// 输入 划分 线段
	void setSegments(std::vector<Segment> lines);
	// 开始划分
	void decompsoition();
	// 保存 划分线段
	void saveSegments(std::string path);
	// 读取 划分线段
	void readSegments(std::string path);
	// 保存 Cell划分
	void saveCell(std::string path);
	// 读取 Cell划分
	void readCell(std::string path);
private:
	Arrangement arr;
	std::vector<Segment> _lines;
};
//---------------------------------------------------------------------------------



