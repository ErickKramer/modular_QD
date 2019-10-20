//|--------------------------------------------------------------------
//| License 
//|--------------------------------------------------------------------
//| This file is a part of the sferes2 framework.
//| Copyright 2009, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jean-Baptiste Mouret, mouret@isir.fr
//|
//| This software is a computer program whose purpose is to facilitate
//| experiments in evolutionary computation and evolutionary robotics.
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software.  You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.
//|--------------------------------------------------------------------
//| Experiment:
//|--------------------------------------------------------------------
//| author: Erick Kramer
//| email: erickkramer@gmail.com
//| Description:
//|     * Generate a repertoire of joints angle for a robotic arm 
//|     * Behavioral descriptor: 3D position | 6D Pose 
//|     * Performance measures:
//|         - Variance of the joint angles
//|         - Total torque
//|         - Amount of movement
//|         - Movement duration
//|--------------------------------------------------------------------

#include <iostream>
#include <cmath>
#include <stdio.h>
#include <unistd.h>
#include <boost/filesystem.hpp>
#define GetCurrentDir getcwd

#include <boost/foreach.hpp>
#include <boost/multi_array.hpp>
#include <boost/array.hpp>
#include <boost/fusion/algorithm/iteration/for_each.hpp>
#include <boost/fusion/include/for_each.hpp>

#include <Eigen/Core>

#include <sferes/eval/parallel.hpp> // Multicore evaluator
#include <sferes/gen/evo_float.hpp> // Genotype of real (float) numbers
#include <sferes/phen/parameters.hpp> // Phenotype parameters
#include <sferes/modif/dummy.hpp> // Dummy modifier
#include <sferes/run.hpp> // Contain the execution methods for sferes
// #include <sferes/stat/best_fit.hpp> // Used to write the bestfit.dat file

#include "quality_diversity.hpp" // Declaration of the QualityDiversity class
#include "fit_qd.hpp" // Contains declaration and set of novelty, curiosity, local quality, descriptor and value

#include "selector.hpp" // Include the header files for the different selector
#include "container.hpp" // Includes the header files for the containers
#include "stat.hpp" // Include the header files for the different statistical methods

#include <robot_dart/arm/arm_simulation.hpp>
#include <robot_dart/robot.hpp>

// #include <robot_dart/robot_dart_simu.hpp>
using namespace sferes;
using namespace sferes::gen::evo_float;

// Create a global namespace
namespace global{
    // std::shared_ptr<arm_dart::SchunkArm> simu;
    std::shared_ptr<robot_dart::Robot> global_robot;
    std::vector<std::string> descriptors;
    Eigen::VectorXd pos_limits;
    std::string pid_file_path;
    std::string end_effector_name;
}

// TODO: Tune QD Parameters
struct Params{
    // Definition of the QD parameters
    struct nov{
        // Parameters required for the containers

        SFERES_CONST size_t deep = 3; // Sub-grid depth parameter (Grid)
        SFERES_CONST size_t k = 15; // Nearest neighbors (Archive)
        SFERES_CONST double l = 0.01; // Distance threshold (Archive)
        SFERES_CONST double eps = 0.1; // Epsilon for exclusive e-dominance (Archive)
    };

    struct ea{

        // Parameters for the evolutionary algorithm
        SFERES_CONST size_t behav_dim = 3; // Dimensions of the behavioral descriptor (Name expected by other files)
        SFERES_ARRAY(size_t, behav_shape, 100, 100); // Dimensions of the grid
        SFERES_CONST size_t genotype_dimensions = 7; // Schunk Arm has 7 DoF
    };

    struct pop{
        SFERES_CONST size_t size = 50; // Size of the population (batch)
        // SFERES_CONST size_t nb_gen = 10001; // Number of generations to run
        SFERES_CONST size_t nb_gen = 2001; // Number of generations to run
        SFERES_CONST size_t dump_period = 50; // Rate to write the archive
    };

    struct parameters{
        // Limits of the genotype parameters
        SFERES_CONST float min = -1;
        SFERES_CONST float max = 1;
    };

    struct evo_float{
        SFERES_CONST float cross_rate = 0.0f;
        SFERES_CONST float mutation_rate = 0.125f;
        SFERES_CONST float eta_m = 10.0f; // Parameter for the polynomial mutation
        SFERES_CONST float eta_c = 10.0f; // Parameter for the polynomial mutation
        SFERES_CONST mutation_t mutation_type = polynomial; // Type of mutation
        SFERES_CONST cross_over_t cross_over_type = sbx; // Type of cross-over
    };
};

FIT_QD(FitPose){
    // Fitness that measures the performance of the behaviors in terms of minimizing the
    // variance between the angular position of the joints.
    // Captures the idea that all the joints of the arm should contribute equally to the movement

    public:
        template<typename Indiv>
        void eval(Indiv& ind){

            // Set individual as target configuration
            std::vector<double> angles_joints(ind.size());
            Eigen::VectorXd angles(ind.size());

            for(size_t i = 0; i < ind.size(); ++i){
                // Constraint the joints angles to the corresponding position limits
                angles_joints[i] = ind.data(i) * global::pos_limits[i];
                angles[i] = ind.data(i) * global::pos_limits[i];
            }
            // Add gripper condition
            angles_joints.push_back(0.); 

            // Launch simulation 
            auto robot = global::global_robot->clone();
            arm_dart::SchunkArmSimu simu(angles_joints, robot, 0.001, global::end_effector_name);
            simu.init_controller(global::pid_file_path);
            simu.set_descriptors(global::descriptors);
            simu.run_simu(10.);

            // Collect recorded data
            double total_movement = simu.get_total_joints_motion();
            // std::cout << "Total Movement " << total_movement << std::endl;
            double total_torque = simu.get_total_torque();
            // std::cout << "Total torque " << total_torque << std::endl;
            double total_steps = simu.get_total_steps();
            // std::cout << "Total steps " << total_steps << std::endl;
            Eigen::VectorXd end_effector_pose = simu.get_final_pose();

            // Computes joints variance
            double joints_variance = sqrt((angles.array() - angles.mean()).square().mean());
            // std::cout << "Joints variance " << joints_variance << std::endl;

            // Computes fitness
            this -> _value = - total_movement;

            // Computes behavioral descriptor
            Eigen::Vector3d end_effector_position = end_effector_pose.head(3);

            // Get arm_length
            // Normalize position to a bounding box similar to Cully2018_2
            float length = simu.get_total_length() * 1.1;

            std::vector<float> data = {(float) (end_effector_position[0] + length) / (2 * length),
                                       (float) (end_effector_position[1] + length) / (2 * length),
                                       (float) (end_effector_position[2] + length) / (2 * length)};

            // Set behavioral descriptor to a 3D position
            this -> set_desc(data);

        }
};

int main(int argc, char **argv){
    // Ensures the generation of random numbers by having a different seed
    srand(time(NULL));
    // Used to controll the task scheduler inside the Thread Building Block
    tbb::task_scheduler_init init(20);

    // -----------------------------------------------------------
    // Robot Definition
    // -----------------------------------------------------------
    // Loading URDF
    char cCurrentPath[FILENAME_MAX];
    if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath))){
        return errno;
    }
    boost::filesystem::path cur_path = cCurrentPath;
    std::vector<std::pair<std::string, std::string>> packages = {{"lwa4d",
        cur_path.parent_path().string() + "/robot_dart/res/models/meshes/lwa4d"}};
    std::string urdf_path = cur_path.parent_path().string()+
        "/robot_dart/res/models/schunk_with_pg70.urdf";
    std::string name = "schunk arm";
    global::end_effector_name = "end_virtual_link";

    // Set PID params file path
    global::pid_file_path = cur_path.parent_path().string() + "/robot_dart/res/pid_params.txt";

    // Create Robot
    global::global_robot = std::make_shared<robot_dart::Robot>(urdf_path, packages,name); 

    // Set simulation descriptors
    global::descriptors = {"joint_states", "pose_states", "velocity_states"};

    // Create a dummy simulation just to display info and get pos_limits 
    arm_dart::SchunkArmSimu dummy_simu(global::global_robot, global::end_effector_name);
    dummy_simu.display_robot_info();
    global::pos_limits = dummy_simu.get_positions_upper_limits();
    // -----------------------------------------------------------
    // QD Definition
    // -----------------------------------------------------------

    // Name of the results_file
    std::string results_name = "schunk_l1_experiment";

    // Define fitness function
    typedef FitPose<Params> fit_t; // Fitness function for the algorithm

    // Define evaluation type
    typedef eval::Parallel<Params> eval_t;
    // typedef eval::Eval<Params> eval_t;

    // Define genotype
    typedef gen::EvoFloat<Params::ea::genotype_dimensions, Params> gen_t;

    // Define phenotype
    typedef phen::Parameters<gen_t, fit_t, Params> phen_t;

    // Define modifier
    typedef modif::Dummy<> modifier_t;

    // Containers definition
    #if defined(GRID)
        typedef container::Grid<phen_t, Params> container_t;
        results_name.append("grid_");
    #else
        typedef container::Archive<phen_t, Params> container_t;
        results_name.append("archive_");
    #endif

    // Definition of the statistical functions to be used
    // Contains scripts to write the progress and archive files.
    typedef boost::fusion::vector<stat::Container<phen_t, Params>, stat::Progress<phen_t, Params> > stat_t;

    // Selectors definition
    #if defined(RANDOM)
        typedef selector::Random<phen_t> selector_t;
        results_name.append("random_");
    #elif defined(FITNESS)
        typedef selector::ScoreProportionate<phen_t, selector::getFitness> selector_t;
        results_name.append("fitness_");
    #elif defined(NOVELTY)
        typedef selector::ScoreProportionate<phen_t, selector::getNovelty> selector_t;
        results_name.append("novelty_");
    #elif defined(CURIOSITY)
        typedef selector::ScoreProportionate<phen_t, selector::getCuriosity> selector_t;
        results_name.append("curiosity_");
    #else
        typedef selector::NoSelection<phen_t> selector_t;
        results_name.append("noselection_");
    #endif

    typedef ea::QualityDiversity<phen_t, eval_t, stat_t, modifier_t, selector_t, container_t, Params> ea_t;

    ea_t ea;

    std::cout << "===================================" << std::endl;
    std::cout << " Starting experiment " << std::endl;
    std::cout << "===================================" << std::endl;

    ea.run(results_name);

    std::cout << "===================================" << std::endl;
    std::cout << " Finishing experiment " << std::endl;
    std::cout << "===================================" << std::endl;
}
