#pragma once
#include "include.h"

// �����ʷ�
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
* ���ܣ�������������ʷ֣����������
* ���룺Face_handle f
* ������ʷֺ�������Ρ�������ĵ�
* ʹ��ʾ��: auto t = Trianglution(f)
*			auto random_points = t.get_random_points();
*/
class Trianglution
{
private:
	
	//�������
	void gen_random_points();
	//�����ʷ�
	void triangle();
public:
	//�������ε����
	float area();
	//��ȡ�����
	std::vector<Point> get_random_points();
	//���ӻ� �ʷֽ��
	void showTriangle();
	//���ӻ� ������
	void showRandomPoints();
	Trianglution(Face_handle f, int num_points_per_area = 50);

private:
	//����Ķ����
	std::vector<Point> cell_polygon;
	//�ֽ��Ķ��������
	std::vector<Kernel::Triangle_2> triangles;
	//�����
	std::vector<Point> random_points;
	//��λ����ĵ���Ŀ
	int points_per_area;
};
