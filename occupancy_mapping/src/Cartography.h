#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <map>
#include <math.h>

#define nullptr	0


// Stores the block matrix row/column for easier access
struct BlockMatrixData;

class Cartography
{
protected:
	image_transport::ImageTransport m_ImageTransport;
	image_transport::Publisher m_ImagePublisher;

	// Map = Cells. Cells = Fixed size matrices. Matrix elements = data about a world space square of dimension m_dCellSize.
	std::map<int, BlockMatrixData*> m_CellMap;
	// Concatenates data from the map of cells.
	 cv::Mat *m_pFinalMatrix;	// cvCreateMat(m,n,CV_64FC1);

	const double m_dCellSize;
	// Size of the matrix representing a square cell of dimension m_dCellSize
	const unsigned int m_uiCellSize;
	// i, j : row/column of the block matrix to access cells
	// Stores the maximum i
	int m_MaxCellRow;
	int m_MinCellRow;
	// Stores the maximum j
	int m_MaxCellColumn;
	int m_MinCellColumn;
	
	// Used to avoid having to re-create at each Publish m_pFinalMatrix
	int m_OldMaxCellRow;
	int m_OldMinCellRow;
	int m_OldMaxCellColumn;
	int m_OldMinCellColumn;

public:
	Cartography(ros::NodeHandle &n, double dCellSize, unsigned int uiCellSize);

	~Cartography();

	void PublishImage();

	void Update(double x, double y, double data);
	cv::Mat* getMat();
};
