//
// Created by lai on 2021/10/9.
//

#include "imageICP.h"

/***
 *
 * @param image must be gray image
 * @return pcl::PointCloud<PointT>
 */
PointCloudT imageICP::pixel2Points(const cv::Mat &image) {
    int cols, rows;
    PointCloudT tmpPoints;
    cols = image.cols;
    rows = image.rows;
    int count = 0;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (image.at<uchar>(i,j) <= 10) {
                continue;
            } else {
                count++;
                //x = i = rows ; y = j = cols
                PointT pt;
                pt.x = i;
                pt.y = j;
                pt.z = 0;
                tmpPoints.points.push_back(pt);
            }

        }
    }
    std::cout << "the point size is " << count << std::endl;
    return tmpPoints;
}

/**
 *
 * @param mat 输入的mat图像，灰度图
 * @return 2d点
 */
std::vector<Eigen::Vector2f> imageICP::mat2Point2D(const cv::Mat& mat){
    std::vector<Eigen::Vector2f> points;
    int cols, rows;
    cols = mat.cols;
    rows = mat.rows;
    int count = 0;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            Eigen::Vector2f p;
            if (mat.at<uchar>(i, j) == 0) {
                continue;
            } else {
                count++;
                p.x() = i;
                p.y() = j;
                points.push_back(p);
            }

        }
    }
    return points;
}

void imageICP::registrationICP() {
    PointCloudT::Ptr oPPtr(new PointCloudT);
    PointCloudT::Ptr tgPPtr(new PointCloudT);
    oPPtr = this->originPoints.makeShared();
    tgPPtr = this->targetPoints.makeShared();


    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(oPPtr);
    icp.setInputTarget(tgPPtr);


    PointCloudT Final;
    icp.align(Final);

    std::cout << "has converged:" << icp.hasConverged() << std::endl;
    this->Rt = icp.getFinalTransformation();


}

void imageICP::imageDataChange() {
    /***
     * 图像转化为点云
     */
    this->originPoints = this->pixel2Points(this->originImage);
    this->targetPoints = this->pixel2Points(this->targetImage);
}
/***
 *
 * @param Rt 旋转矩阵
 * @param originImage 要处理的原始图像
 * @return 返回icp匹配后的Mat
 */
cv::Mat imageICP::ICP4Image(const Eigen::Matrix<float, 4, 4>& Rt,const cv::Mat& originImage){
    PointCloudT::Ptr tmpCloud(new PointCloudT);
    pcl::transformPointCloud(pixel2Points(originImage), *tmpCloud, Rt);
    int rows = originImage.rows;
    int cols = originImage.cols;
    cv::Mat resultImage(rows , cols, CV_8UC1, cv::Scalar(0));
    for (int i = 0; i < tmpCloud->size(); ++i) {
        // x = rows ,y = cols
        resultImage.at<uchar>(tmpCloud->points[i].x ,tmpCloud->points[i].y) = 255 ;
    }
    return resultImage;
}
/***
 *
 * @param cols
 * @param rows
 * @param pt
 * @return
 */
cv::Mat imageICP::showImage(const int& cols, const int& rows, const PointCloudT& pt){
    cv::Mat resultImage(rows , cols, CV_8UC1, cv::Scalar(0));
    for (int i = 0; i < pt.size(); ++i) {
        resultImage.at<uchar>(pt.points[i].x ,pt.points[i].y) = 255 ;
    }
    return resultImage;
}
Eigen::Matrix3f imageICP::svd4Rt(){

}
/**
 * 将目标图像和源图像转化为points
 */
void imageICP::allMat2Point2d() {
    points p1(mat2Point2D(this->originImage));
    points p2(mat2Point2D(this->targetImage));
    this->origin2DPoints = p1;
    this->target2DPoints = p2;
}
/**
 *
 * @param colorImage 要映射裂缝到彩色图像(BGR)的mat
 * @param crackData 裂缝图像
 * @param originCrackData 彩色图像对应的裂缝
 */
cv::Mat imageICP::resultInImage(const cv::Mat &colorImage, const cv::Mat &crackData, const cv::Mat &originCrackData) {
    cv::Mat CrackInImage = colorImage;
    //设置图像尺寸一致
    if (colorImage.cols != crackData.cols && colorImage.rows != crackData.rows ){
        std::cout <<  " 裂缝图像大小" << crackData.size() << std::endl;
        letterBox(CrackInImage,crackData.rows, crackData.cols);
//        cv::resize(colorImage, CrackInImage, cv::Size(crackData.cols, crackData.rows));
        std::cout <<  " 裂缝图像大小" << crackData.size() << std::endl;
        std::cout <<  " resize后的图像大小" << CrackInImage.size() << std::endl;
        for (int i = 0; i < crackData.rows; ++i) {
            for (int j = 0; j < crackData.cols; ++j) {
                if (crackData.at<uchar>(i,j) != 0){
                    //绿色显示匹配结果
                    CrackInImage.at<cv::Vec3b>(i,j)[0] = 255 ;
                }
                if (originCrackData.at<uchar>(i,j)  >= 10 ){
                    //红色是原始图像的裂缝识别结果
                    CrackInImage.at<cv::Vec3b>(i,j)[2] = 255 ;
                }
            }
        }
    }else{
        for (int i = 0; i < crackData.rows; ++i) {
            for (int j = 0; j < crackData.cols; ++j) {
                if (crackData.at<uchar>(i,j) != 0){
                    CrackInImage.at<cv::Vec3b>(i,j)[0] = 255 ;
                }
                if (originCrackData.at<uchar>(i,j) >= 10){
                    CrackInImage.at<cv::Vec3b>(i,j)[2] = 255 ;
                }
            }
        }
    }
    return CrackInImage;

}
/**
 * 等比例缩放照片，边缘用黑色填充
 * @param rows 要输出的mat的行
 * @param cols 要输出的列
 */
void imageICP::letterBox(cv::Mat& inputMat, const int rows, const int cols ) {
    float scale;
    int targetRows, targetCols;
    float originRows = static_cast<float>(inputMat.rows) ;
    float originCols = static_cast<float>(inputMat.cols) ;
    scale = cols/originCols;
    //取相除小的
    ((rows / originRows) < (cols / originCols)) ?  scale = (rows / originRows): scale = (cols / originCols);
    std::cout << "scale: " << scale << std::endl;
    targetRows = static_cast<int>(scale * originRows);
    targetCols = static_cast<int>(scale * originCols);
    std::cout
    << "targetRows,targetCols "
    << targetRows
    << ", "
    << targetCols
    << "originRows, originCols"
    << inputMat.rows
    << ", "
    << inputMat.cols
    << std::endl;

    cv::Mat tmp;
    cv::resize(inputMat, tmp, cv::Size(targetCols, targetRows), cv::INTER_LINEAR);
    int left = (targetCols - cols) / 2;
    int right = targetCols - cols - left;
    int top = (rows- targetRows) / 2;
    int bottom = rows - targetRows- top;
    std::cout
    << top
    << "  "
    << bottom
    << "  "
    << left
    << "  "
    << right
    << std::endl;
    //输入mat，输出mat
    cv::copyMakeBorder(tmp,  inputMat, top, bottom, left, right, cv::BORDER_CONSTANT);

}

