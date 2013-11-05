/*
 * RSVM.cpp
 *
 *  Created on: Nov 1, 2013
 *      Author: sheng
 */

#include "RSVM.hpp"

RSVM::RSVM():params(){
}

RSVM::RSVM(
		int svm_type, int kernel_type, double degree, double gamma,
		double coef0, double Cvalue, double nu, double p,
		CvMat* class_weights, CvTermCriteria term_crit):
		params(svm_type,kernel_type,degree, gamma, coef0, Cvalue, nu, p, class_weights, term_crit){
}

RSVM::~RSVM(){

}

void RSVM::setLabels(const cv::Mat& lb){
	this->labels = lb;
}

void RSVM::setTrainData(const cv::Mat& td){
	this->trainData = td;
}

void RSVM::train(const cv::Mat& td, const cv::Mat & lb){
	svm.train(trainData,labels,cv::Mat(),cv::Mat(),params);
}

void RSVM::predict(const cv::Mat& sampleMat, cv::Mat& results){

}
