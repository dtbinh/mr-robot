// http://www.cs.iit.edu/~agam/cs512/lect-notes/opencv-intro/opencv-intro.html#SECTION00061000000000000000
// http://ftp.isr.ist.utl.pt/pub/roswiki/cv_bridge(2f)Tutorials(2f)UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages.html
// http://jeux-et-mathematiques.davalan.org/divers/bij/
// Bijections de N^n dans N
// http://wiki.ros.org/catkin/CMakeLists.txt

#include "Cartography.h"
#include "Cell.h"
#include <float.h>
#include <math.h>

// We cap the maximum/minimum value for log odd
#define MAX_LOG_ODD	log(FLT_MAX/2)
#define MIN_LOG_ODD	-log(FLT_MAX/2)

using namespace std;

struct BlockMatrixData
{
	//! Coordinates (m,n) of the block matrix in m_pFinalMatrix
	int m;
	int n;
	cv::Mat *pMatrix;

	BlockMatrixData() : pMatrix(nullptr)	{}

	~BlockMatrixData()
	{
		if(pMatrix)
			delete pMatrix;
	}
};

Cartography::Cartography(ros::NodeHandle &n, double dCellSize, unsigned int uiCellSize):
	m_ImageTransport(n), m_dCellSize(dCellSize), m_uiCellSize(uiCellSize),
	m_MinCellRow(INT_MAX), m_MaxCellRow(INT_MIN),
	m_MinCellColumn(INT_MAX), m_MaxCellColumn(INT_MIN),
	m_pFinalMatrix(nullptr),
	m_OldMaxCellRow(0), m_OldMinCellRow(0),
	m_OldMaxCellColumn(0), m_OldMinCellColumn(0)
{
	m_ImagePublisher = m_ImageTransport.advertise("image",1);
}

Cartography::~Cartography()
{
	for(auto it = m_CellMap.begin(); it != m_CellMap.end(); it++)
		delete(it->second);
}

void Cartography::PublishImage()
{
	if(m_MinCellRow == INT_MAX || m_MaxCellRow == INT_MIN || m_MinCellColumn == INT_MAX || m_MaxCellColumn == INT_MIN)
		return;

	int numRows = (m_MaxCellRow-m_MinCellRow+1)*m_uiCellSize;
	int numColumns = (m_MaxCellColumn-m_MinCellColumn+1)*m_uiCellSize;

	if(((m_MinCellColumn != m_OldMinCellColumn) || (m_MaxCellRow != m_OldMaxCellRow) || (m_MinCellRow != m_OldMinCellRow) || (m_MaxCellColumn != m_OldMaxCellColumn)))
	{
		m_OldMaxCellRow = m_MaxCellRow;
		m_OldMinCellColumn = m_MinCellColumn;
		m_OldMinCellRow = m_MinCellRow;
		m_OldMaxCellColumn = m_MaxCellColumn;

		if(m_pFinalMatrix)
			delete m_pFinalMatrix;
		m_pFinalMatrix =  new cv::Mat(numRows,numColumns,CV_32F);
		for(int i = 0; i < numRows; i++)
		{
			for(int j = 0; j < numColumns; j++)
				m_pFinalMatrix->at<float>(i, j) = 0.0f;
		}
	}
	
	// ROS_INFO("%d %d", numRows, numColumns);

	// Copy the data from m_CellMap
	// Note that the image has the Y axis inverted so we invert the rows in the final matrix
	// to restore the correct orientation
	for(auto it = m_CellMap.begin(); it != m_CellMap.end(); it++)
	{
		for(int i = 0; i < m_uiCellSize; i++)
		{
			for(int j = 0; j < m_uiCellSize; j++)
			{
				int m = int(it->second->m-m_MinCellRow)*m_uiCellSize+i;
				int n = int(it->second->n-m_MinCellColumn)*m_uiCellSize+j;
				m_pFinalMatrix->at<float>(m, n) = it->second->pMatrix->at<float>(i, j);
			}
		}
	}

	auto imageMatrix = cv::Mat(numRows,numColumns,CV_32S);
	// Prepare the colors before publishing the cvMat as an image
	for(int i = 0; i < numRows; i++)
	{
		for(int j = 0; j < numColumns; j++)
		{
			// Compute the probability associated with the log odd
			double p = 1.0f-1.0f/(1.0f+exp(m_pFinalMatrix->at<float>(i, j)));
			// p close to 1 means that the certainty that the cell is traversable is very high.
			// We return a color close to 255 (white) for such values.
			//ROS_INFO("%f",m_pFinalMatrix->at<float>(i, j));
			int color=int(p*255.0);
			//ROS_INFO("%f %d", p, color);
			int finalColor = color;
			finalColor |= (color << 8);
			finalColor |= (color << 16);
			imageMatrix.at<int>(i, j) =  finalColor;
		}
	}


	cv_bridge::CvImage out_msg;
	out_msg.encoding = "rgba8";
	out_msg.image    = imageMatrix;

	// O.O This is really weird we did an infinite loop for Project 1, and it... worked ?

	/*
	ros::Rate loop_rate(5);
	while (ros::ok())
	{
		m_ImagePublisher.publish(out_msg.toImageMsg());

		loop_rate.sleep();
	}*/

	m_ImagePublisher.publish(out_msg.toImageMsg());
}

static void CapRange(float &value)
{
	if(value >= MAX_LOG_ODD)
		value = MAX_LOG_ODD;
	if(value <= MIN_LOG_ODD)
		value = MIN_LOG_ODD;
}

void Cartography::Update(double x, double y, double data)
{
	int i, j;
	i = floor(double(x)/m_dCellSize);
	j = floor(double(y)/m_dCellSize);

	m_MaxCellRow = std::max(m_MaxCellRow, i);
	m_MinCellRow = std::min(m_MinCellRow, i);
	m_MaxCellColumn = std::max(m_MaxCellColumn, j);
	m_MinCellColumn = std::min(m_MinCellColumn, j);

	// Compute the bijection between Z^2 and N
	int f_i, f_j; 
	if(i < 0)
		f_i = -2*i-1;
	else
		f_i = 2*i;
	if(j < 0)
		f_j = -2*j-1;
	else
		f_j = 2*j;
	int idx = ((f_i+f_j)*(f_i+f_j)+f_i+3*f_j)/2;

	// Fills in the new data
	auto it = m_CellMap.find(idx);
	BlockMatrixData *pData;
	if(it == m_CellMap.end())
	{
		pData = new BlockMatrixData;
		pData->m = i;
		pData->n = j;
		pData->pMatrix = new cv::Mat(m_uiCellSize,m_uiCellSize,CV_32F);

		for(int i = 0; i < m_uiCellSize; i++)
		{
			for(int j = 0; j < m_uiCellSize; j++)
			{
				pData->pMatrix->at<float>(i,j)=0.0f;
			}
		}
		m_CellMap[idx] = pData;
	}
	else
		pData = it->second;

	// Take the decimal part of x and y
	// For negative value it will take 1-decimal part
	double _01x = x-double(i)*m_dCellSize; 
	double _01y = y-double(j)*m_dCellSize;
	// Convert to cvMat row and column
	int m = int(_01x*m_uiCellSize);
	int n = int(_01y*m_uiCellSize);
	//ROS_INFO("%d %d", m, n);
	//ROS_INFO("%f", pData->pMatrix->at<float>(m, n));
	float fLogOdd = pData->pMatrix->at<float>(m, n);
	fLogOdd = float(data) + fLogOdd;
	//CapRange(fLogOdd);
	pData->pMatrix->at<float>(m, n) = fLogOdd;
	//ROS_INFO("%f", pData->pMatrix->at<float>(m, n));

}

cv::Mat* Cartography::getMat(){
	return this->m_pFinalMatrix;
}
