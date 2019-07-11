#include "custom_arm.hpp"

namespace robot{
    Eigen::Vector3d CustomArm::forward_model(Eigen::VectorXd angles){
        // Computes end effector position using forward kinematics

        Eigen::VectorXd _l_arm = Eigen::VectorXd(9);
        _l_arm(0) = 0;        
        _l_arm(1) = l1;
        _l_arm(2) = l2;
        _l_arm(3) = l3;
        _l_arm(4) = l4;
        _l_arm(5) = l5;
        _l_arm(6) = l6;
        _l_arm(7) = l7;
        _l_arm(8) = l8;

        // Cully2018 suggests the change in the sign of the angles, but why?. 
        angles = - angles;

        // Identity matrix
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity(4,4);

        // Compute the transformation matrix multiplications from base to the last link
        // Only rotation over z is considered
        for(size_t i=0; i < angles.size(); i++){
            Eigen::Matrix4d submat;
            submat << cos(angles(i)), -sin(angles(i)), 0, _l_arm(i),
                      sin(angles(i)), cos(angles(i)),  0,    0,
                      0,              0,               1,    0,
                      0,              0,               0,    1;
            mat *= submat;
        }

        // Computes a translation over the last link 
        Eigen::Matrix4d submat;
        submat << 1, 0, 0, _l_arm(angles.size()),
                  0, 1, 0,     0,
                  0, 0, 1,     0,
                  0, 0, 0,     1;
        
        mat *= submat;

        Eigen::VectorXd position = mat * Eigen::Vector4d(0,0,0,1);

        return position.head(3); 

    }
}