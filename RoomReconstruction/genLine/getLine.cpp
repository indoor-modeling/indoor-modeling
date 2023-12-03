#define PCL_NO_PRECOMPILE
#include "utils.hpp"


void savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string outpath)
{
    std::cerr << "save path is :" << outpath << endl;
    //将string保存路径转为char*
    char* path = new char[outpath.size() + 1];
    strcpy(path, outpath.c_str());
    std::cerr << "Path is : " << path << " ." << std::endl;

    //写出点云图
    pcl::PLYWriter writer;
    writer.write(path, *cloud, true);
    std::cerr << "PointCloud has : " << cloud->width * cloud->height << " data points." << std::endl;
}

//void main() {
//	pcl::PointCloud<MyPointType> cloud;
//	//read_ply<MyPointType>("../res1/slicePC/slicePC.ply", cloud);
//	read_ply<MyPointType>("../res/slicePC/wall_new.ply", cloud);
//
//	Clouds_vector<MyPointType> lines;
//	genWallLines<MyPointType>(cloud, lines, 100, 0.05, 0.05); // !!!maxmisspoint<0.3
//   
//	//save_Cloud_vector<MyPointType>("../res/linePC_new", lines);
//    for (int i = 0; i < lines.size(); i++) {
//        pcl::PointCloud<pcl::PointXYZRGB> out;
//        auto rgb = pcl::getRandomColor();
//        for (auto p : lines[i]) {   
//            
//            pcl::PointXYZRGB  p_rgb(p.x, p.y, 0, rgb.r, rgb.g, rgb.b);
//            out.push_back(p_rgb);
//        }
//        save_ply<pcl::PointXYZRGB>("../res/linePC_new/line_"+ std::to_string(i) + "_rgb.ply", out);
//    }
//    
//}


// 大场景
//void main() {
//    pcl::PointCloud<MyPointType> cloud;
//    //read_ply<MyPointType>("../res1/slicePC/slicePC.ply", cloud);
//    read_ply<MyPointType>("../res/slicePC/wall_new.ply", cloud);
//
//    Clouds_vector<MyPointType> lines;
//    genWallLines<MyPointType>(cloud, lines, 100, 0.05, 0.05); // !!!maxmisspoint<0.3
//
//    //save_Cloud_vector<MyPointType>("../res/linePC_new", lines);
//    for (int i = 0; i < lines.size(); i++) {
//        pcl::PointCloud<pcl::PointXYZRGB> out;
//        auto rgb = pcl::getRandomColor();
//        for (auto p : lines[i]) {
//
//            pcl::PointXYZRGB  p_rgb(p.x, p.y, 0, rgb.r, rgb.g, rgb.b);
//            out.push_back(p_rgb);
//        }
//        save_ply<pcl::PointXYZRGB>("../res/linePC_new/line_" + std::to_string(i) + "_rgb.ply", out);
//    }
//
//}

// 小场景实验
//void main() {
//    pcl::PointCloud<MyPointType> cloud;
//    //read_ply<MyPointType>("../res1/slicePC/slicePC.ply", cloud);
//    read_ply<MyPointType>("../unbalance_small_scene/slice_pro.ply", cloud);
//
//    Clouds_vector<MyPointType> lines;
//    genWallLines<MyPointType>(cloud, lines, 100, 0.05, 0.05); // !!!maxmisspoint<0.3
//
//    //save_Cloud_vector<MyPointType>("../res/linePC_new", lines);
//    for (int i = 0; i < lines.size(); i++) {
//        pcl::PointCloud<pcl::PointXYZRGB> out;
//        auto rgb = pcl::getRandomColor();
//        for (auto p : lines[i]) {
//
//            pcl::PointXYZRGB  p_rgb(p.x, p.y, 0, rgb.r, rgb.g, rgb.b);
//            out.push_back(p_rgb);
//        }
//        save_ply<pcl::PointXYZRGB>("../unbalance_small_scene/line_pc/line_" + std::to_string(i) + "_rgb.ply", out);
//    }
//
//}


//// 区域5
//void main() {
//    pcl::PointCloud<MyPointType> cloud;
//    //read_ply<MyPointType>("../res1/slicePC/slicePC.ply", cloud);
//    read_ply<MyPointType>("../area5/slice_pro.ply", cloud);
//
//    Clouds_vector<MyPointType> lines;
//    genWallLines<MyPointType>(cloud, lines, 100, 0.05, 0.05); // !!!maxmisspoint<0.3
//
//    //save_Cloud_vector<MyPointType>("../res/linePC_new", lines);
//    for (int i = 0; i < lines.size(); i++) {
//        pcl::PointCloud<pcl::PointXYZRGB> out;
//        auto rgb = pcl::getRandomColor();
//        for (auto p : lines[i]) {
//
//            pcl::PointXYZRGB  p_rgb(p.x, p.y, 0, rgb.r, rgb.g, rgb.b);
//            out.push_back(p_rgb);
//        }
//        save_ply<pcl::PointXYZRGB>("../area5/line_pc/line_" + std::to_string(i) + "_rgb.ply", out);
//    }
//
//}

//// 区域5 part2
//void main() {
//    pcl::PointCloud<MyPointType> cloud;
//    read_ply<MyPointType>("../area5_part2/slice_pro.ply", cloud);
//
//    Clouds_vector<MyPointType> lines;
//    genWallLines<MyPointType>(cloud, lines, 100, 0.01, 0.05); // !!!maxmisspoint<0.3
//
//    //save_Cloud_vector<MyPointType>("../res/linePC_new", lines);
//    for (int i = 0; i < lines.size(); i++) {
//        pcl::PointCloud<pcl::PointXYZRGB> out;
//        auto rgb = pcl::getRandomColor();
//        for (auto p : lines[i]) {
//
//            pcl::PointXYZRGB  p_rgb(p.x, p.y, 0, rgb.r, rgb.g, rgb.b);
//            out.push_back(p_rgb);
//        }
//        save_ply<pcl::PointXYZRGB>("../area5_part2/line_pc2/line_" + std::to_string(i) + "_rgb.ply", out);
//    }
//}

// 区域5全景
void main() {
    pcl::PointCloud<MyPointType> cloud;
    read_ply<MyPointType>("../area5_full/slice_pro.ply", cloud);

    Clouds_vector<MyPointType> lines;
    genWallLines<MyPointType>(cloud, lines, 100, 0.01, 0.05); // !!!maxmisspoint<0.3

    //save_Cloud_vector<MyPointType>("../res/linePC_new", lines);
    for (int i = 0; i < lines.size(); i++) {
        pcl::PointCloud<pcl::PointXYZRGB> out;
        auto rgb = pcl::getRandomColor();
        for (auto p : lines[i]) {

            pcl::PointXYZRGB  p_rgb(p.x, p.y, 0, rgb.r, rgb.g, rgb.b);
            out.push_back(p_rgb);
        }
        save_ply<pcl::PointXYZRGB>("../area5_full/line_pc2/line_" + std::to_string(i) + "_rgb.ply", out);
    }
}