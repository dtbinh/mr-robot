// http://www.cs.iit.edu/~agam/cs512/lect-notes/opencv-intro/opencv-intro.html#SECTION00061000000000000000
// http://ftp.isr.ist.utl.pt/pub/roswiki/cv_bridge(2f)Tutorials(2f)UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages.html
// http://jeux-et-mathematiques.davalan.org/divers/bij/
// Bijections de N^n dans N
// http://wiki.ros.org/catkin/CMakeLists.txt

#include "DME.h"
#include "Cell.h"
#include <limits.h>
#include <math.h>
#include <fstream>

// We cap the maximum number of measures to update recursively their mean
#define MAX_NUM_MEASURES	INT_MAX/2
// Parameter of the covariance function (squared exp. kernel)
#define TAU	20.0
#define SIGMA_2	0.1

using namespace std;

struct BlockMatrixData
{
	//! Coordinates (m,n) of the block matrix in m_pFinalMatrix
	int m;
	int n;
	// Stores the mean and the variance of the height, which follows a normal distribution
	cv::Mat *pHeightMatrix;
	cv::Mat *pVarianceMatrix;
	// Stores the number of measurements for a given cell
	cv::Mat *pNumMeasurementsMatrix;

	BlockMatrixData() : pHeightMatrix(nullptr), pVarianceMatrix(nullptr), pNumMeasurementsMatrix(nullptr)	{}

	~BlockMatrixData()
	{
		if(pHeightMatrix)
			delete pHeightMatrix;
		if(pVarianceMatrix)
			delete pVarianceMatrix;
		if(pNumMeasurementsMatrix)
			delete pNumMeasurementsMatrix;
	}
};

DME::DME(double dCellSize, unsigned int uiCellSize):
	m_dCellSize(dCellSize), m_uiCellSize(uiCellSize),
	m_MinCellRow(INT_MAX), m_MaxCellRow(INT_MIN),
	m_MinCellColumn(INT_MAX), m_MaxCellColumn(INT_MIN),
	m_pFinalMatrix(nullptr),
	m_pFinalVarianceMatrix(nullptr),
	m_OldMaxCellRow(0), m_OldMinCellRow(0),
	m_OldMaxCellColumn(0), m_OldMinCellColumn(0)
{
}

DME::~DME()
{
	for(auto it = m_CellMap.begin(); it != m_CellMap.end(); it++)
		delete(it->second);
	if(m_pFinalMatrix)
		delete m_pFinalMatrix;
	if(m_pFinalVarianceMatrix)
		delete m_pFinalVarianceMatrix;
}

void DME::PublishToFile()
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
		if(m_pFinalVarianceMatrix)
			delete m_pFinalVarianceMatrix;
		m_pFinalMatrix =  new cv::Mat(numRows,numColumns,CV_32F);
		m_pFinalVarianceMatrix =  new cv::Mat(numRows,numColumns,CV_32F);
		for(int i = 0; i < numRows; i++)
		{
			for(int j = 0; j < numColumns; j++)
			{
				m_pFinalMatrix->at<float>(i, j) = 0.0f;
				m_pFinalVarianceMatrix->at<float>(i, j) = 0.0f;
			}
		}
	}
	
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
				m_pFinalMatrix->at<float>(m, n) = it->second->pHeightMatrix->at<float>(i, j);
				m_pFinalVarianceMatrix->at<float>(m, n) = it->second->pVarianceMatrix->at<float>(i, j);
			}
		}
	}
	std::ofstream out("/tmp/DME.txt", std::ios_base::ate);
	for(int i = 0; i < numRows; i++)
	{
		for(int j = 0; j < numColumns; j++)
			out << double(i)*m_dCellSize << " " << double(j)*m_dCellSize << " " << m_pFinalMatrix->at<float>(i, j) << " " << m_pFinalVarianceMatrix->at<float>(i, j) << std::endl;
	}
	out.close();
}

// Squared exponential kernel function
static float Covariance(float u, float v, float sigmaU, float sigmaV)
{
	return sigmaU*sigmaV*exp(-(u-v)*(u-v)/(2*TAU*TAU));
}

static float Mean(float newValue, float previousMean, int numMeasurements)
{
	if(numMeasurements>=MAX_NUM_MEASURES)
		return previousMean+newValue/numMeasurements;
	return (float(numMeasurements)/(numMeasurements+1))*previousMean+newValue/(numMeasurements+1);
}

void DME::Update(double x, double y, double data)
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
		pData->pHeightMatrix = new cv::Mat(m_uiCellSize,m_uiCellSize,CV_32F);
		pData->pVarianceMatrix = new cv::Mat(m_uiCellSize,m_uiCellSize,CV_32F);
		pData->pNumMeasurementsMatrix = new cv::Mat(m_uiCellSize,m_uiCellSize,CV_32S);

		for(int i = 0; i < m_uiCellSize; i++)
		{
			for(int j = 0; j < m_uiCellSize; j++)
			{
				pData->pHeightMatrix->at<float>(i,j)=FLT_MIN;
				pData->pVarianceMatrix->at<float>(i,j)=SIGMA_2;
				pData->pNumMeasurementsMatrix->at<int>(i,j)=0;
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
	// Previous height stored before measuring data
	float &fHeight = pData->pHeightMatrix->at<float>(m, n);
	float &fVariance = pData->pVarianceMatrix->at<float>(m,n);
	float fCorrelationCoefficient = Covariance(data,fHeight, SIGMA_2, fVariance);
	int &numMeasurements = pData->pNumMeasurementsMatrix->at<int>(m,n);

	// Now perform the real update of the DME
	if(fHeight==FLT_MIN)
		fHeight = data;
	else
		fHeight = Mean(data,fHeight,numMeasurements)+(SIGMA_2/fVariance)*fCorrelationCoefficient*(data-fHeight);
	//fVariance = SIGMA_2*(1-fCorrelationCoefficient*fCorrelationCoefficient);
	fVariance = 1/(fVariance*fVariance+numMeasurements/SIGMA_2);
	numMeasurements = std::min(numMeasurements+1,MAX_NUM_MEASURES);
}

cv::Mat* DME::getMat(){
	return this->m_pFinalMatrix;
}

cv::Mat* DME::getVarMat(){
	return this->m_pFinalVarianceMatrix;
}
