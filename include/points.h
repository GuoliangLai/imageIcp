//
// Created by lai on 2021/10/29.
//

#ifndef IMAGEICP_POINTS_H
#define IMAGEICP_POINTS_H

#include <Eigen/Dense>
#include <vector>

typedef Eigen::Vector2f point2d;
class points {
public:
    //默认构造函数
    points() = default;
    points(const std::vector<point2d>& points):
    Points(points),
    OriginPoints(points){
    }
    ~points() {}
    //返回点
    point2d getPoint(const int index) const {return this->Points[index];}
    void pointBackup(){ this->OriginPoints = this->Points;}
    std::vector<point2d> getPoints() const {return this->Points;}
    int size() const {return this->Points.size();}
    //得到两点间的距离
    float getL2Distance(const point2d& p1,const point2d& p2);
    //添加
    void push_back(const point2d& p){ this->Points.push_back(p);
        //每次更新备份一次
    pointBackup();}
    point2d getCentroid();
    //RT变换，旋转变换，平移
    void transform2D(const Eigen::Matrix2f& R,const Eigen::Vector2f& t);
    void rotation2D(const Eigen::Matrix2f& R);
    void translation2D(const Eigen::Vector2f& t);
    void removeCenter();
    Eigen::Matrix2f operator*(const points& p);




private:
    //点精度为float类型
    point2d point;
    //质心
    point2d Centroid;
    std::vector<point2d> Points;
    std::vector<point2d> OriginPoints;

};


#endif //IMAGEICP_POINTS_H
