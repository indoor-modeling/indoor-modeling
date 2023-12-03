//#define _CRT_SECURE_NO_DEPRECATE
//#define PCL_NO_PRECOMPILE
//#include "utils.hpp"

//void main() {
//	// 定义基本信息
//	//---------------------------------------------------------------------------------
//	pcInfo info;
//	info.map_resolution = 0.15;
//
//	// 1. 读取ply文件,设置 pcInfo
//	//---------------------------------------------------------------------------------
//	std::string ply_path = "../data/Area_6_full_sub_0.04.ply";
//	pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
//	read_ply<MyPointType>(ply_path, *cloud);
//	std::cout << "read success" << std::endl;
//
//	setPCInfo<MyPointType>(*cloud, info);
//	savePCInfo("../data1/wall_info.txt", info);
//	/*std::vector<int> wall_label{ (int)S3DISLabel::wall,
//								(int)S3DISLabel::window,
//								(int)S3DISLabel::door,
//								(int)S3DISLabel::board };*/
//	std::vector<int> wall_label{ (int)S3DISLabel::wall,
//								(int)S3DISLabel::window,
//								(int)S3DISLabel::door,
//								(int)S3DISLabel::board,
//								(int)S3DISLabel::column,
//	(int)S3DISLabel::beam};
//	cv::Mat wall_map = genSemanticMap(*cloud, info, wall_label);
//	cv::imshow("wall", wall_map);	
//	cv::waitKey();
//	cv::imwrite("../wall_map1.png", wall_map);
//}



//生成room_map地图
//void main() {
//	// 定义基本信息
//	//---------------------------------------------------------------------------------
//	pcInfo info;
//	readPCInfo("../data/pcInfo.txt", info);
//
//	// 1. 读取ply文件,设置 pcInfo
//	//---------------------------------------------------------------------------------
//	std::string ply_path = "../data/Area_6_full.ply";
//	pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
//	read_ply<MyPointType>(ply_path, *cloud);
//	std::cout << "read success" << std::endl;
//
//	std::vector<int> wall_label{ (int)S3DISLabel::wall,
//										(int)S3DISLabel::window,
//										(int)S3DISLabel::door,
//										(int)S3DISLabel::board,
//		(int)S3DISLabel::beam,
//		(int)S3DISLabel::column
//											 };
//	//(int)S3DISLabel::beam
//	cv::Mat room_map = genSemanticMap(*cloud, info, wall_label, false);
//	cv::imshow("wall", room_map);	
//	cv::waitKey();
//	//cv::imwrite("../data/room_map_chen2.png", room_map);
//	cv::imwrite("../data/wall_map_multi2.png", room_map);
//}


//小场景面积不平衡实验
//void main() {
//	 定义基本信息
//	---------------------------------------------------------------------------------
//	pcInfo info;
//	info.map_resolution = 0.05;
//
//	 1. 读取ply文件,设置 pcInfo
//	---------------------------------------------------------------------------------
//	std::string ply_path = "../unbalance_small_scene/small_area_class.ply";
//	pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
//	read_ply<MyPointType>(ply_path, *cloud);
//	std::cout << "read success" << std::endl;
//
//	setPCInfo<MyPointType>(*cloud, info);
//	savePCInfo("../unbalance_small_scene/wall_info.txt", info);
//	/*std::vector<int> wall_label{ (int)S3DISLabel::wall,
//								(int)S3DISLabel::window,
//								(int)S3DISLabel::door,
//								(int)S3DISLabel::board };*/
//	std::vector<int> wall_label{ (int)S3DISLabel::wall,
//								(int)S3DISLabel::window,
//								(int)S3DISLabel::door,
//								(int)S3DISLabel::board,
//								(int)S3DISLabel::column,
//	(int)S3DISLabel::beam };
//	/*std::vector<int> wall_label{ (int)S3DISLabel::wall,(int)S3DISLabel::board,(int)S3DISLabel::column
//	};*/
//	cv::Mat wall_map = genSemanticMap(*cloud, info, wall_label,false);
//	cv::imshow("wall", wall_map);
//	cv::waitKey();
//	cv::imwrite("../unbalance_small_scene/room_map_slice.png", wall_map);
//}

////区域5
//void main() {
//	// 定义基本信息
//	//---------------------------------------------------------------------------------
//	pcInfo info;
//	readPCInfo("../area5/pcInfo.txt", info);
//
//	// 1. 读取ply文件,设置 pcInfo
//	//---------------------------------------------------------------------------------
//	std::string ply_path = "../area5/Area_5_part.ply";
//	pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
//	read_ply<MyPointType>(ply_path, *cloud);
//	std::cout << "read success" << std::endl;
//
//		/*std::vector<int> wall_label{ (int)S3DISLabel::wall,
//									(int)S3DISLabel::window,
//									(int)S3DISLabel::door,
//									(int)S3DISLabel::board,
//									(int)S3DISLabel::column,
//		(int)S3DISLabel::beam};*/
//	std::vector<int> wall_label{2,3,4};
//	//(int)S3DISLabel::beam
//	cv::Mat room_map = genSemanticMap(*cloud, info, wall_label, false);
//	cv::imshow("wall", room_map);	
//	cv::waitKey();
//	//cv::imwrite("../data/room_map_chen2.png", room_map);
//	cv::imwrite("../area5/wall_map.png", room_map);
//}


// 区域5 part2
//void main() {
//	// 定义基本信息
//	//---------------------------------------------------------------------------------
//	pcInfo info;
//	info.map_resolution = 0.05;
//
//	// 1. 读取ply文件,设置 pcInfo
//	//---------------------------------------------------------------------------------
//	std::string ply_path = "../area5_part2/Area_5_part2.ply";
//	pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
//	read_ply<MyPointType>(ply_path, *cloud);
//	std::cout << "read success" << std::endl;
//
//	//setPCInfo<MyPointType>(*cloud, info);
//	readPCInfo("../area5_part2/info/pcInfo.txt", info);
//	//savePCInfo("../area5_part2/info/pcInfo.txt", info);
//	/*std::vector<int> wall_label{ (int)S3DISLabel::wall,
//								(int)S3DISLabel::window,
//								(int)S3DISLabel::door,
//								(int)S3DISLabel::board };*/
//	std::vector<int> wall_label{ 2,3,4};
//	std::vector<int> room_label{  0,1,2,3,4,5};
//	/*std::vector<int> wall_label{ (int)S3DISLabel::wall,(int)S3DISLabel::board,(int)S3DISLabel::column
//	};*/
//	cv::Mat wall_map = genSemanticMap(*cloud, info, room_label, false);
//	cv::imshow("wall", wall_map);
//	cv::waitKey();
//	cv::imwrite("../area5_part2/room_map.png", wall_map);
//}


////区域5全景
//void main() {
//	// 定义基本信息
//	//---------------------------------------------------------------------------------
//	pcInfo info;
//	info.map_resolution = 0.05;
//
//	// 1. 读取ply文件,设置 pcInfo
//	//---------------------------------------------------------------------------------
//	std::string ply_path = "../area5_full/Area_5_full.ply";
//	pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
//	read_ply<MyPointType>(ply_path, *cloud);
//	std::cout << "read success" << std::endl;
//
//	//setPCInfo<MyPointType>(*cloud, info);
//	readPCInfo("../area5_full/info/pcInfo.txt", info);
//	//savePCInfo("../area5_part2/info/pcInfo.txt", info);
//	/*std::vector<int> wall_label{ (int)S3DISLabel::wall,
//								(int)S3DISLabel::window,
//								(int)S3DISLabel::door,
//								(int)S3DISLabel::board };*/
//	std::vector<int> wall_label{ 2,3,4};
//	std::vector<int> room_label{ 0,1,2,3,4,5};
//	/*std::vector<int> wall_label{ (int)S3DISLabel::wall,(int)S3DISLabel::board,(int)S3DISLabel::column
//	};*/
//	cv::Mat wall_map = genSemanticMap(*cloud, info, wall_label, false);
//	cv::imshow("wall", wall_map);
//	cv::waitKey();
//	cv::imwrite("../area5_full/wall_map.png", wall_map);
//}

////区域4全景
//void main() {
//	// 定义基本信息
//	//---------------------------------------------------------------------------------
//	pcInfo info;
//	info.map_resolution = 0.05;
//
//	// 1. 读取ply文件,设置 pcInfo
//	//---------------------------------------------------------------------------------
//	std::string ply_path = "../area4/Area_4.ply";
//	pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
//	read_ply<MyPointType>(ply_path, *cloud);
//	std::cout << "read success" << std::endl;
//
//	//setPCInfo<MyPointType>(*cloud, info);
//	readPCInfo("../area4/info/pcInfo.txt", info);
//	//savePCInfo("../area4/info/pcInfo.txt", info);
//	/*std::vector<int> wall_label{ (int)S3DISLabel::wall,
//								(int)S3DISLabel::window,
//								(int)S3DISLabel::door,
//								(int)S3DISLabel::board };*/
//	std::vector<int> wall_label{ 2,4,5,6};
//	std::vector<int> room_label{ 0,1,2,3,4,5,6,7,8,9,10,11,12};
//	/*std::vector<int> wall_label{ (int)S3DISLabel::wall,(int)S3DISLabel::board,(int)S3DISLabel::column
//	};*/
//	cv::Mat wall_map = genSemanticMap(*cloud, info, wall_label, false);
//	//cv::Mat room_map = genSemanticMap(*cloud, info, room_label, false);
//	cv::imshow("wall", wall_map);
//	cv::waitKey();
//	cv::imwrite("../area4/wall_map.png", wall_map);
//	//cv::imwrite("../area4/room_map.png", room_map);
//}

////区域3全景
//void main() {
//	// 定义基本信息
//	//---------------------------------------------------------------------------------
//	pcInfo info;
//	info.map_resolution = 0.05;
//
//	// 1. 读取ply文件,设置 pcInfo
//	//---------------------------------------------------------------------------------
//	std::string ply_path = "../area3/Area_3.ply";
//	pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
//	read_ply<MyPointType>(ply_path, *cloud);
//	std::cout << "read success" << std::endl;
//
//	setPCInfo<MyPointType>(*cloud, info);
//	//readPCInfo("../area3/info/pcInfo.txt", info);
//	savePCInfo("../area3/info/pcInfo.txt", info);
//	
//	/*std::vector<int> wall_label{ 2,4,5,6 };
//	std::vector<int> room_label{ 0,1,2,3,4,5,6,7,8,9,10,11,12 };*/
//
//	std::vector<int> wall_label{ 2,3 };
//	std::vector<int> room_label{ 0,1,2,3,4,5 };
//	
//	cv::Mat wall_map = genSemanticMap(*cloud, info, wall_label, false);
//	cv::Mat room_map = genSemanticMap(*cloud, info, room_label, false);
//	cv::imshow("wall", wall_map);
//	cv::waitKey();
//	cv::imwrite("../area3/wall_map.png", wall_map);
//	cv::imwrite("../area3/room_map.png", room_map);
//}