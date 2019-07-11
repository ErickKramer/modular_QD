#ifndef ROBOT_CUSTOM_ARM_HPP // TODO
#define ROBOT_CUSTOM_ARM_HPP

#include <Eigen/Core>

// Definition of the arm
namespace robot{
    class CustomArm{
        public:
            static Eigen::Vector3d forward_model(Eigen::VectorXd angles);
            static double max_length(){
                return l1 + l2 + l3 + l4 + l5 + l6 + l7 + l8;
                //return l1 + l2 + l3;
            }

        protected:
            // TODO: Based the file to a real arm
            static constexpr double l1 = 0.0875;
            static constexpr double l2 = 0.078;
            static constexpr double l3 = 0.072;
            static constexpr double l4 = 0.072;
            static constexpr double l5 = 0.071;
            static constexpr double l6 = 0.068;
            static constexpr double l7 = 0.068;
            static constexpr double l8 = 0.065;
    };
}

#endif