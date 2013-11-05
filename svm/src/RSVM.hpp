/*
 * RSVM.hpp
 *
 *  Created on: Nov 1, 2013
 *      Author: sheng
 */
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>

#ifndef RSVM_HPP_
#define RSVM_HPP_

class RSVM
{
private:
	CvSVM svm;
	CvSVMParams params;

	cv::Mat trainData;
	cv::Mat labels;
public:
	RSVM();
	RSVM(int svm_type, int kernel_type, double degree, double gamma,
			double coef0, double Cvalue, double nu, double p,
			CvMat* class_weights, CvTermCriteria term_crit);
	virtual ~RSVM();

	void setTrainData(const cv::Mat& td);
	void setLabels(const cv::Mat& lb);
	void train(const cv::Mat& td, const cv::Mat& lb);
	void predict(const cv::Mat& sampleMat, cv::Mat& results);
};



#endif /* RSVM_HPP_ */
