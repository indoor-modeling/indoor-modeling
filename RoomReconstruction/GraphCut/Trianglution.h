#pragma once
#include "include.h"

// 三角剖分
struct TriIndex
{
	unsigned value[3];
	TriIndex() {
		for (unsigned i = 0; i < 3; ++i)
			value[i] = -1;
	}
	TriIndex(unsigned i1, unsigned i2, unsigned i3) {
		value[0] = i1;
		value[1] = i2;
		value[2] = i3;
	}

	unsigned& operator[](unsigned i) {
		return value[i];
	}
};
typedef std::vector<TriIndex> TriIndices;

unsigned findVertexIndex(Point pt, std::vector<Point> points);

TriIndices  GetTriIndices(std::vector<Point> points);


/*
* 功能：将多边形三角剖分，及随机撒点
* 输入：Face_handle f
* 输出：剖分后的三角形、随机撒的点
* 使用示例: auto t = Trianglution(f)
*			auto random_points = t.get_random_points();
*/
class Trianglution
{
private:
	
	//随机撒点
	void gen_random_points();
	//三角剖分
	void triangle();
public:
	//计算多边形的面积
	float area();
	//获取随机点
	std::vector<Point> get_random_points();
	//可视化 剖分结果
	void showTriangle();
	//可视化 撒点结果
	void showRandomPoints();
	Trianglution(Face_handle f, int num_points_per_area = 50);

private:
	//输入的多边形
	std::vector<Point> cell_polygon;
	//分解后的多个三角形
	std::vector<Kernel::Triangle_2> triangles;
	//随机点
	std::vector<Point> random_points;
	//单位面积的点数目
	int points_per_area;
};
