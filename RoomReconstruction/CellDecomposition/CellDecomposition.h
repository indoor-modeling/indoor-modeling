#pragma once
#include "include.h"

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
