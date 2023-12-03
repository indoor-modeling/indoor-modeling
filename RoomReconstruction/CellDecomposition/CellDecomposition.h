#pragma once
#include "include.h"

// ����Cell��Ԫ
//---------------------------------------------------------------------------------
class cellDecom
{
public:
	// ���� ���� �߶�
	void setSegments(std::vector<Segment> lines);
	// ��ʼ����
	void decompsoition();
	// ���� �����߶�
	void saveSegments(std::string path);
	// ��ȡ �����߶�
	void readSegments(std::string path);
	// ���� Cell����
	void saveCell(std::string path);
	// ��ȡ Cell����
	void readCell(std::string path);
private:
	Arrangement arr;
	std::vector<Segment> _lines;
};
//---------------------------------------------------------------------------------
