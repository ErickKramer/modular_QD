//author: Erick Kramer
//email: erickkramer@gmail.com

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

// TODO: Include schunk_arm 
#include <robot_dart/arm/arm_simulation.hpp>
// #include <robot_dart/robot_dart_simu.hpp>
using namespace sferes;
using namespace sferes::gen::evo_float;

// Create a global namespace
namespace global{
    std::shared_ptr<arm_dart::SchunkArm> simu;
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
        SFERES_CONST size_t behav_dim = 2; // Dimensions of the behavioral descriptor (Name expected by other files)
        SFERES_ARRAY(size_t, behav_shape, 100, 100); // Dimensions of the grid 
        SFERES_CONST size_t genotype_dimensions = 8;
    };

    struct pop{
        SFERES_CONST size_t size = 200; // Size of the population (batch)
        SFERES_CONST size_t nb_gen = 10001; // Number of generations to run
        SFERES_CONST size_t dump_period = 1000; // Rate to write the archive
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

FIT_QD(AngularVariance){
    // Fitness that measures the performance of the behaviors in terms of minimizing the 
    // variance between the angular position of the joints. 
    // Captures the idea that all the joints of the arm should contribute equally to the movement

    public:
        template<typename Indiv>
        void eval(Indiv& ind){

            // Set individual as target configuration
            std::vector<double> angles_joints(ind.size(), 0.0);
            Eigen::VectorXd pos_limits = global::simu->get_positions_upper_limits();
            for(size_t i = 0; i < ind.size(); ++i){
                // Constraint the joints angles to the corresponding position limits
                angles_joints[i] = ind.data(i) * pos_limits[i]; 
            }
            global::simu->set_goal_configuration(angles_joints);

            // Run simulation 
            global::simu->run_simu(10.);

            // Collect recorded data 
            double total_movement = global::simu->get_total_joints_motion();
            double total_torque = global::simu->get_total_torque();
            double total_steps = global::simu->get_total_steps();
            Eigen::VectorXd end_effector_pose = global::simu->get_final_pose();

            // Computes joints variance
            // double joints_variance = sqrt((angles_joints.array() - angles_joints.mean()).square().mean());

            // Computes fitness
            this -> _value = - total_movement;

            // Computes behavioral descriptor
            Eigen::Vector3d end_position = end_effector_pose.head(3);

            // TODO: Normalize position

    //         this -> _value = - sqrt((angles_joints.array() - angles_joints.mean()).square().mean());

    //         Eigen::Vector3d position = robot::CustomArm::forward_model(angles_joints); // Get the position of the end effector

    //         float total_length = robot::CustomArm::max_length() * 1.1; // Normalized the values to a bounding box with values between 0 and 1

    //         std::vector<float> data = {(float) (position[0] + total_length) / (2 * total_length),
    //                                    (float) (position[1] + total_length) / (2 * total_length) };
            
    //         this -> set_desc(data); // Pass the behavioral descriptor corresponding to a position in a 2D space.

        }
};

int main(int argc, char **argv){
    srand(time(NULL)); // Ensures the generation of random numbers by having a different seed 
    tbb::task_scheduler_init init(20); // Used to controll the task scheduler inside the Thread Building Block

    // Get Working directory
    char cCurrentPath[FILENAME_MAX];
    if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath))){
        return errno;
    }
    // cCurrentPath[sizeof(cCurrentPath)-1] = '\0';
    // std::cout<<"Current working directory is " << cCurrentPath << std::endl;
    boost::filesystem::path cur_path = cCurrentPath;

    // Print info 
    // std::cout << "parent path  " << cur_path.parent_path() << std::endl;


    // -----------------------------------------------------------
    // Simulation Definition
    // -----------------------------------------------------------
    // Loading URDF 
    std::vector<std::pair<std::string, std::string>> packages = {{"lwa4d",
        cur_path.parent_path().string() + "/robot_dart/res/models/meshes/lwa4d"}};
    std::string urdf_path = cur_path.parent_path().string()+
        "/robot_dart/res/models/schunk_with_pg70.urdf";
    std::string name = "schunk arm";

    // Load simulation
    global::simu = std::make_shared<arm_dart::SchunkArm>(urdf_path, packages, name);
    // arm_dart::SchunkArm simu(urdf_path, packages,name); 

    // Initialize simulation
    double time_step = 0.001;
    global::simu->init_simu(time_step);

    // Initialize PID controller 
    std::string pid_file_path = cur_path.parent_path().string()+
        "/robot_dart/res/pid_params.txt";
    global::simu->init_controller(pid_file_path);

    // Set Acceleration limits
    global::simu->set_acceleration_limits(0.01);

    // Specify desired descriptors
    std::vector<std::string> descriptors = {"joint_states", "pose_states", "velocity_states"};
    global::simu->set_descriptors(descriptors);

    // Display robot_info
    global::simu->display_robot_info();

    // Run simulation
    // double simulation_time = 10.;
    // global::simu->run_simu(simulation_time/4.);

    // Reset descriptors 
    // simu.reset_descriptors(descriptors);

    // Set a new configuration
    // std::vector<double> conf(simu.get_control_dofs(), 0.0);
    // conf[1] = M_PI_2;
    // simu.set_goal_configuration(conf);
    // simu.run_simu(simulation_time);
    // simu.reset_descriptors(descriptors);

    // -----------------------------------------------------------
    // QD Definition
    // -----------------------------------------------------------

    // Name of the results_file
    std::string results_name = "schunk_l1_experiment";

    // Define fitness function
    typedef AngularVariance<Params> fit_t; // Fitness function for the algorithm 

    // Define evaluation type
    typedef eval::Parallel<Params> eval_t;

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
