//
// Created by lai on 2021/10/9.
//

#ifndef IMAGEICP_IMAGEICP_H
#define IMAGEICP_IMAGEICP_H

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
#include <iostream>
#include "points.h"
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class imageICP {
public:
    imageICP();
    imageICP(const cv::Mat& oImage, const cv::Mat& tImage)
    : originImage(oImage), targetImage(tImage)
    {
    }
    ~imageICP(){}
    //获得4x4RT矩阵
    Eigen::Matrix<float, 4, 4> getRt()  {return this->Rt;}
    //获得图像RT
    Eigen::Matrix3f getiRt(){
        this->iRt << Rt(0,0), Rt(0,1),Rt(0,3),
                     Rt(1,0), Rt(1,1),Rt(1,3),
                     Rt(2,0), Rt(2,1),Rt(3,3);
        return this->iRt;
    }
    //计算icp
    void registrationICP();
    //
    void setImage(const cv::Mat& oImage, const cv::Mat& tImage)
    {
        this->originImage = oImage;
        this->targetImage = tImage;
    }
    //像素转化为点云
    static PointCloudT pixel2Points(const cv::Mat& image);
    void imageDataChange();
    cv::Mat ICP4Image(const Eigen::Matrix<float, 4, 4>& Rt,const cv::Mat& originImage);
    cv::Mat showImage(const int& cols, const int& rows, const PointCloudT& pt);
    cv::Mat resultInImage(const cv::Mat& colorImage, const cv::Mat& crackData, const cv::Mat &originCrackData);
    Eigen::Matrix3f svd4Rt();
    std::vector<Eigen::Vector2f> mat2Point2D(const cv::Mat& mat);
    void allMat2Point2d();
    static void letterBox(cv::Mat& inputMat, const int rows, const int cols);


private:
    Eigen::Matrix<float, 4, 4> Rt;
    cv::Mat originImage, targetImage;
    PointCloudT originPoints, targetPoints;
    Eigen::Matrix<float, 2, 2> iR;
    Eigen::Matrix<float, 3, 3> iRt;
    Eigen::Vector2cf it;
    points origin2DPoints;
    points target2DPoints;


};


#endif //IMAGEICP_IMAGEICP_H
