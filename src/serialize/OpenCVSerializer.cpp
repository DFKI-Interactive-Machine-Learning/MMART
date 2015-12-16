#include "serialize/OpenCVSerializer.hpp"
#include <vector>

void save_depth_pnm(cv::Mat depth, const bfs::path path)
{
    FILE *pFile=0;
    pFile = fopen((path.string()).c_str(),"wb");

    if (pFile!=0)
    {
        fprintf(pFile, "P5\n");
		fprintf(pFile, "%d %d\n%i\n", depth.cols, depth.rows, 65535);
        fwrite(depth.data,2, depth.cols*depth.rows,pFile);
        fflush(pFile);
        fclose(pFile);
    }
}

void opencv_serialize_depth(cv::Mat depth, const bfs::path path) 
{
	double min;
	double max;
	cv::minMaxIdx(depth, &min, &max);
	cv::Mat adjMap;
	// expand your range to 0..255. Similar to histEq();
	depth.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min); 

	cv::Mat adjMap2;
	cv::convertScaleAbs(depth, adjMap, 255 / max);
	cv::imwrite(path.string(), adjMap);
}