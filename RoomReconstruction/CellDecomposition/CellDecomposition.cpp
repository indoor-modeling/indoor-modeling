#include "CellDecomposition.h"

void cellDecom::setSegments(std::vector<Segment> lines)
{
	_lines = lines;
}

void cellDecom::decompsoition()
{
	//CGAL::insert(arr, _lines);
	for (auto line : _lines)
	{
		CGAL::insert_curve(arr, line);
	}
}

void cellDecom::saveSegments(std::string path)
{
	std::ofstream out;
	out.open(path);
	for (auto line : _lines)
		out << line << std::endl;
	out.close();
}

void cellDecom::readSegments(std::string path)
{
	std::ifstream in;
	in.open(path);
	Segment s;
	std::string oneLine;
	while (in >> s)
		_lines.push_back(s);
	in.close();
}

void cellDecom::saveCell(std::string path)
{
	std::ofstream out_file(path);
	out_file << arr;
	out_file.close();
}

void cellDecom::readCell(std::string path)
{
	std::ifstream in(path);
	in >> arr;
	in.close();
}