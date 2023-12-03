#include "CellDecomposition.h"

//void main() {
//	cellDecom cell;
//	cell.readSegments("../res1/refineLine/simple_lines_3.txt");
//	cell.decompsoition();
//	cell.saveCell("../res1/Cell/cell.dat");
//}

// 按平面提取的直线
//void main() {
//	cellDecom cell;
//	cell.readSegments("../simple_lines_3.txt");
//	cell.decompsoition();
//	cell.saveCell("../cell.dat");
//}
// 
 //新的按平面提取的直线
//void main() {
//	cellDecom cell;
//	cell.readSegments("../Plane_line2_simply/simple_lines_3.txt");
//	cell.decompsoition();
//	cell.saveCell("../Plane_line2_simply/cell.dat");
//}

//// 小场景实验
//void main() {
//	cellDecom cell;
//	cell.readSegments("../unbalance_small_scene/simple_lines.txt");
//	cell.decompsoition();
//	cell.saveCell("../unbalance_small_scene/cell.dat");
//}

 //Area5新的按平面提取的直线
//void main() {
//	cellDecom cell;
//	cell.readSegments("../area5/simple_lines.txt");
//	cell.decompsoition();
//	cell.saveCell("../area5/cell.dat");
//}

////Area5全景
//void main() {
//	cellDecom cell;
//	// 根据切片生成
//	/*cell.readSegments("../area5_full/slice_graph_cut/refine_lines.txt");
//	cell.decompsoition();
//	cell.saveCell("../area5_full/slice_graph_cut/cell.dat");*/
//	// 根据平面生成
//	cell.readSegments("../area5_full/regular_lines.txt");
//	cell.decompsoition();
//	cell.saveCell("../area5_full/cell.dat");
//}

////Area4全景
//void main() {
//	cellDecom cell;
//	// 根据切片生成
//	/*cell.readSegments("../area5_full/slice_graph_cut/refine_lines.txt");
//	cell.decompsoition();
//	cell.saveCell("../area5_full/slice_graph_cut/cell.dat");*/
//	// 根据平面生成
//	cell.readSegments("../area4/regular_lines.txt");
//	cell.decompsoition();
//	cell.saveCell("../area4/cell.dat");
//}

//Area3全景
void main() {
	cellDecom cell;
	// 根据切片生成
	/*cell.readSegments("../area5_full/slice_graph_cut/refine_lines.txt");
	cell.decompsoition();
	cell.saveCell("../area5_full/slice_graph_cut/cell.dat");*/
	// 根据平面生成
	cell.readSegments("../area3/regular_lines1.txt");
	cell.decompsoition();
	cell.saveCell("../area3/cell.dat");
}