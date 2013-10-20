// http://www.cs.iit.edu/~agam/cs512/lect-notes/opencv-intro/opencv-intro.html#SECTION00061000000000000000
// http://ftp.isr.ist.utl.pt/pub/roswiki/cv_bridge(2f)Tutorials(2f)UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages.html
// http://jeux-et-mathematiques.davalan.org/divers/bij/
// Bijections de N^n dans N
// http://wiki.ros.org/catkin/CMakeLists.txt

#include "Cartography.h"
#include "Cell.h"

// We cap the maximum/minimum value for log odd
#define MAX_LOG_ODD	1000000.0f
#define MIN_LOG_ODD	-1000000.0f

using namespace std;

struct BlockMatrixData
{
	//! Coordinates (m,n) of the block matrix in m_pFinalMatrix
	int m;
	int n;
	cv::Mat *pMatrix;

	BlockMatrixData()
	{
		pMatrix = nullptr;
	}

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
				m_pFinalMatrix->at<float>(m, n) = m_pFinalMatrix->at<float>(m, n) + it->second->pMatrix->at<float>(i, j);
			}
		}
	}

	auto imageMatrix = cv::Mat(numRows,numColumns,CV_8UC1);
	// Prepare the colors before publishing the cvMat as an image
	for(int i = 0; i < numRows; i++)
	{
		for(int j = 0; j < numColumns; j++)
		{
			// Compute the probability associated with the log odd
			double p = 1.0-1.0/(1.0+exp(m_pFinalMatrix->at<float>(i, j)));
			// p close to 1 means that the certainty that the cell is traversable is very high.
			// We return a color close to 255 (white) for such values.
			imageMatrix.at<int>(i, j) = int(p*255.0);
		}
	}


	cv_bridge::CvImage out_msg;
	out_msg.encoding = "mono8";
	out_msg.image    = imageMatrix;

	// O.O This is really weird we did an infinite loop ?
	/*
	ros::Rate loop_rate(5);
	while (ros::ok())
	{
		m_ImagePublisher.publish(out_msg.toImageMsg());
		ros::spinOnce();
		loop_rate.sleep();
	}
	*/
	m_ImagePublisher.publish(out_msg.toImageMsg());
}

void Cartography::Update(double x, double y, double data)
{
	int i, j;
	i = x/m_dCellSize;
	j = y/m_dCellSize;
	if(x < 0 && (x-double(i)*m_dCellSize < 0))
		i--;
	if(y < 0 && (y-double(j)*m_dCellSize < 0))
		j--;

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

	// Number of units covered for an element in the array
	double dResolution = m_dCellSize/m_uiCellSize;
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
				pData->pMatrix->at<float>(i,j)=0.0f;
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
	pData->pMatrix->at<float>(m, n) = float(data) + pData->pMatrix->at<float>(m, n);
}

int main(int argc, char** argv)
{
	/* initialize random seed: */
	srand (time(NULL));
	ros::init(argc, argv, "cartography");
	ros::NodeHandle n;
	Cartography foo(n, 1, 10);
	for(int i = 0; i < 20000; i++)
	{
		double x = (rand() % 100) + (float)rand()/(float)RAND_MAX;
		double y = (rand() % 100) + (float)rand()/(float)RAND_MAX;
		if(rand() % 3 == 0)
			foo.Update(double(x-50)/10, double(y-50)/10, NonTraversable);
		else if(rand() % 3 == 1)
			foo.Update(double(x-50)/10, double(y-50)/10, Traversable);
		else if(rand() % 3 == 2)
			foo.Update(double(x-50)/10, double(y-50)/10, Unknown);
	}
	foo.PublishImage();
	ros::spin();
	return 0;
}
