//
// Created by lai on 2021/10/29.
//

#include "points.h"
#include <cmath>
#include <iostream>
/**
 *
 * @param p1 输入点1
 * @param p2 输入点2
 * @return
 */
float points::getL2Distance(const point2d& p1,const point2d& p2){
    float distance = 0;
    distance = sqrt((p1.x()-p2.x())*(p1.x()-p2.x())+
                      (p1.y()-p2.y())*(p1.y()-p2.y()));
    return distance;

}


point2d points::getCentroid(){
    float xCenter = 0;
    float yCenter = 0;
    for (int i = 0; i < this->Points.size(); ++i) {
        xCenter += this->Points[i].x();
        yCenter += this->Points[i].y();
    }
    xCenter = xCenter / this->Points.size();
    yCenter = yCenter / this->Points.size();
    point2d Centroid(xCenter,yCenter);
    this->Centroid = Centroid;
    return Centroid;

}
/**
 *
 * @param R 旋转矩阵R
 * @param t 平移向量T
 */
void points::transform2D(const Eigen::Matrix2f& R,const Eigen::Vector2f& t){
    std::vector<point2d> transform2DAfter;
    for (int i = 0; i < this->Points.size(); ++i) {
        point2d transformPoint;
        transformPoint = R * this->Points[i] + t ;
        transform2DAfter.push_back(transformPoint);
    }
    this->Points = transform2DAfter;

}
/**
 *
 * @param R 旋转矩阵
 */
void points::rotation2D(const Eigen::Matrix2f& R){
    std::vector<point2d> rotation2DAfter;
    for (int i = 0; i < this->Points.size(); ++i) {
        point2d rotationPoint;
        rotationPoint = R * this->Points[i] ;
        rotation2DAfter.push_back(rotationPoint);
    }
    this->Points = rotation2DAfter;

}
/**
 *
 * @param t 平移向量
 */
void points::translation2D(const Eigen::Vector2f& t){
    std::vector<point2d> translation2DAfter;
    for (int i = 0; i < this->Points.size(); ++i) {
        point2d translationPoint;
        translationPoint = this->Points[i] + t ;
        translation2DAfter.push_back(translationPoint);
    }
    this->Points = translation2DAfter;
}
Eigen::Matrix2f points::operator*(const points& p){
    Eigen::Matrix2f result;
    if (p.size() != this->size())
    {
        std::cerr  << "two point set must be equal !!" << std::endl ;
        abort();
    } else {
        for (int i = 0; i < p.size(); ++i) {
            result += Eigen::Vector2f(this->Points[i].x(), this->Points[i].y()) *
                    Eigen::Vector2f(p.Points[i].x(), p.Points[i].y()).transpose();
        }
    }
    return result;
}

void points::removeCenter() {
    std::vector<point2d> q(this->Points.size());
    for (int i = 0; i < this->Points.size(); ++i) {
        this->Points[i] = this->Points[i] - this->Centroid;
    }
}

