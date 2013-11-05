/*
 * SVM.cpp
 *
 *  Created on: Nov 5, 2013
 *      Author: sheng
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

using namespace cv;

int main()
{
    // 用于保存可视化数据的矩阵
    int width = 512, height = 512;
    Mat image = Mat::zeros(height, width, CV_8UC3);

    // 创建一些训练样本
    float labels[4] = {1.0, 1.0, -1.0, -1.0};
    Mat labelsMat(3, 1, CV_32FC1, labels);
    labelsMat.push_back(1.0);

    float trainingData[4][2] = { {501, 10}, {255, 10}, {501, 255}, {10, 501} };
    Mat trainingDataMat(3, 2, CV_32FC1, trainingData);

    // 设置SVM参数
    CvSVMParams params;
    params.svm_type    = CvSVM::C_SVC;
    params.kernel_type = CvSVM::RBF; // exp(-gamma*|u-v|^2)
    params.C = 1;
    params.gamma = 0.5;
    params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);


//    CvSVMParams params;
//     params.svm_type    = CvSVM::C_SVC;
//     params.kernel_type = CvSVM::LINEAR;
//     params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
    // 对SVM进行训练
    CvSVM SVM;
    SVM.train(trainingDataMat, labelsMat, Mat(), Mat(), params);

    Vec3b green(0,255,0), blue (255,0,0);
    // 将SVM断定的分划区域绘制出来
    for (int i = 0; i < image.rows; ++i)
        for (int j = 0; j < image.cols; ++j)
        {
            Mat sampleMat = (Mat_<float>(1,2) << i,j);

            float response = SVM.predict(sampleMat);
            Mat result;
            CvMat m = sampleMat;
            SVM.predict(&m,&result);
            std::cout<<sampleMat<<" result" << response<<std::endl;
            if (response == 1)
                image.at<Vec3b>(j, i)  = green;
            else if (response == -1)
                image.at<Vec3b>(j, i)  = blue;
        }

    // 绘制训练数据点
    int thickness = -1;
    int lineType = 8;
    circle( image, Point(501,  10), 5, Scalar(  0,   0,   0), thickness, lineType);
    circle( image, Point(255,  10), 5, Scalar(255, 255, 255), thickness, lineType);
    circle( image, Point(501, 255), 5, Scalar(255, 255, 255), thickness, lineType);
    circle( image, Point( 10, 501), 5, Scalar(255, 255, 255), thickness, lineType);

    // 绘制支持向量
    thickness = 2;
    lineType  = 8;
    int c     = SVM.get_support_vector_count();

    for (int i = 0; i < c; ++i)
    {
        const float* v = SVM.get_support_vector(i);
        circle( image,  Point( (int) v[0], (int) v[1]),   6,  Scalar(128, 128, 128), thickness, lineType);
    }

    imwrite("result.png", image);
    imshow("简单SVM分类", image);
    waitKey(0);

}
