#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <iostream>


using namespace std;

int main(){
Eigen::Isometry3d desired_pose;
desired_pose(0,3)=15;
desired_pose(1,3)=10;
desired_pose(2,3)=14;
Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();
P(5,5) = 15;
cout<< P.matrix()<<endl;
cout<< desired_pose.linear()<<endl;
}
