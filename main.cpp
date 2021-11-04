#include <iostream>
#include "include/imageICP.h"
#include <opencv2/opencv.hpp>

//输出字体颜色设置
#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"    /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"   /* Yellow */
#define BLUE "\033[34m"   /* Blue */
#define MAGENTA "\033[35m"   /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"   /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"  /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"   /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"   /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"  /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */


void showHelp();
int main(int argc,char* argv[]) {
    showHelp();
    std::string inputImagePath, targetImagePath, colorImagePath;
    cv::Mat inputImage, targetImag, colorImage;
    std::string crackDataPathHead("../data/crack/");
    std::string colorImagePathHead("../data/image/");
    if(argc < 3){
        std::cerr << RED << "please input exactly image path !" << RESET << std::endl;
        abort();
    } else {
        inputImagePath = crackDataPathHead + argv[1];
        targetImagePath = crackDataPathHead + argv[2];
        colorImagePath = colorImagePathHead + argv[2];

    }
    try {
         inputImage = cv::imread(inputImagePath, cv::IMREAD_GRAYSCALE);
         targetImag = cv::imread(targetImagePath, cv::IMREAD_GRAYSCALE);
         //RGB彩色图
         colorImage = cv::imread(colorImagePath, cv::IMREAD_COLOR);
        if (inputImage.empty()){
            throw inputImagePath;
        }
        if (targetImag.empty()){
            throw targetImagePath;
        }
        if (colorImage.empty()){
            throw colorImagePath;
        }
    } catch (char *e) {
        std::cout << RED << "[error]please check your image Path "<< e <<"!!!" << RESET << std::endl;
        abort();
    }

    //对象实例化
    imageICP icp(inputImage, targetImag);

    //开始匹配
    icp.imageDataChange();
    icp.registrationICP();
    std::cout << icp.getiRt() << std::endl;

    //获得匹配结果
    int rows = inputImage.rows;
    int cols = inputImage.cols;
    cv::Mat resultImage(rows , cols, CV_8UC1, cv::Scalar(0));

    resultImage =  icp.ICP4Image(icp.getRt(), inputImage);

    cv::Mat crackColorImage;
    crackColorImage = icp.resultInImage(colorImage, resultImage, targetImag);

    cv::imshow("colorImage",crackColorImage);
    cv::waitKey(0);
    return EXIT_SUCCESS;
}
/***
 * help text
 */
void showHelp(){
    std::cout
    <<"======================================================================"
    <<std::endl
    <<"first param is the input image name, second is the target image name "
    <<std::endl
    << "this program will return 4*4 Rt matrix "
    <<std::endl
    <<"======================================================================="
    << std::endl;
}
