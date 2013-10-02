// http://www.cs.iit.edu/~agam/cs512/lect-notes/opencv-intro/opencv-intro.html#SECTION00061000000000000000
// http://ftp.isr.ist.utl.pt/pub/roswiki/cv_bridge(2f)Tutorials(2f)UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages.html
// http://jeux-et-mathematiques.davalan.org/divers/bij/
// Bijections de N^n dans N
// http://wiki.ros.org/catkin/CMakeLists.txt

#include "Cartography.h"
#include "Cell.h"

using namespace std;

struct BlockMatrixData
{
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
	m_MinCellRow(INT_MAX), m_MaxCellRow(INT_MIN), m_MinCellColumn(INT_MAX), m_MaxCellColumn(INT_MIN),
	m_pFinalMatrix(nullptr), m_OldMaxCellRow(0),
	m_OldMinCellRow(0),
	m_OldMaxCellColumn(0),
	m_OldMinCellColumn(0)
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

	if(m_pFinalMatrix && ((m_MinCellColumn != m_OldMinCellColumn) || (m_MaxCellRow != m_OldMaxCellRow) || (m_MinCellRow != m_OldMinCellRow) || (m_MaxCellColumn != m_OldMaxCellColumn)))
	{
		m_OldMaxCellRow = m_MaxCellRow;
		m_OldMinCellColumn = m_MinCellColumn;
		m_OldMinCellRow = m_MinCellRow;
		m_OldMaxCellColumn = m_MaxCellColumn;

		delete m_pFinalMatrix;
		m_pFinalMatrix =  new cv::Mat(numRows,numColumns,CV_32S);
		for(int i = 0; i < numRows; i++)
		{
			for(int j = 0; j < numColumns; j++)
				m_pFinalMatrix->at<int>(i, j) = Unknown;
		}
	}
	
	ROS_INFO("FinalMatrix size %d %d", (m_MaxCellRow-m_MinCellRow+1)*m_uiCellSize, (m_MaxCellColumn-m_MinCellColumn+1)*m_uiCellSize);
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
				m = numRows-1-m;
				int n = int(it->second->n-m_MinCellColumn)*m_uiCellSize+j;
				m_pFinalMatrix->at<int>(m, n) = int(it->second->pMatrix->at<int>(i, j)) & int(m_pFinalMatrix->at<int>(m, n));
			}
		}
	}

	// Prepare the colors before publishing the cvMat as an image
	for(int i = 0; i < numRows; i++)
	{
		for(int j = 0; j < numColumns; j++)
		{
			switch(m_pFinalMatrix->at<int>(i, j))
			{
			case NonTraversable:
				m_pFinalMatrix->at<int>(i, j) = 255;	// Red
				break;
			case Traversable:
				m_pFinalMatrix->at<int>(i, j) = 255 << 16;	// Blue
				break;
			case Unknown:
				m_pFinalMatrix->at<int>(i, j) = 0xFFFFFFFF;	// White
				break;
			}
		}
	}

	/*
	for(int i = 0; i < numRows; i++)
	{
	for(int j = 0; j < numColumns; j++)
	ROS_INFO("%d", m_pFinalMatrix->at<int>(i, j));
	}
	*/

	cv_bridge::CvImage out_msg;
	out_msg.encoding = "rgba8";
	out_msg.image    = *m_pFinalMatrix;

	ros::Rate loop_rate(5);
	while (ros::ok())
	{
		ROS_INFO("Publishing image");
		m_ImagePublisher.publish(out_msg.toImageMsg());
		ros::spinOnce();
		loop_rate.sleep();
	}
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

	// Compute the bijection between Z� and N
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
		pData->pMatrix = new cv::Mat(m_uiCellSize,m_uiCellSize,CV_32S);

		for(int i = 0; i < m_uiCellSize; i++)
		{
			for(int j = 0; j < m_uiCellSize; j++)
				//cvmSet(pData->pMatrix, i, j, Unknown);
				pData->pMatrix->at<int>(i,j)=Unknown;
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
	pData->pMatrix->at<int>(int(_01x*m_uiCellSize), int(_01y*m_uiCellSize)) = (int)data & int(pData->pMatrix->at<int>(m, n));
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
