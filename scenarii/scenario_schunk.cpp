//author: Erick Kramer
//email: erickkramer@gmail.com

#include <iostream>
#include <cmath>

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

using namespace sferes;
using namespace sferes::gen::evo_float;

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

    // public:
    //     template<typename Indiv>
    //     void eval(Indiv& ind){
    //         Eigen::VectorXd angles_joints(ind.size()); // Declaration of the angles_joints vector
    //         for(size_t i = 0; i < ind.size(); ++i){
    //             angles_joints[i] = ind.data(i) * M_PI/2; // Constraint the values between [-pi/2, pi/2]
    //         }

    //         this -> _value = - sqrt((angles_joints.array() - angles_joints.mean()).square().mean());

    //         Eigen::Vector3d position = robot::CustomArm::forward_model(angles_joints); // Get the position of the end effector

    //         float total_length = robot::CustomArm::max_length() * 1.1; // Normalized the values to a bounding box with values between 0 and 1

    //         std::vector<float> data = {(float) (position[0] + total_length) / (2 * total_length),
    //                                    (float) (position[1] + total_length) / (2 * total_length) };
            
    //         this -> set_desc(data); // Pass the behavioral descriptor corresponding to a position in a 2D space.

    //     }

};

int main(int argc, char **argv){
    srand(time(NULL)); // Ensures the generation of random numbers by having a different seed 
    tbb::task_scheduler_init init(20); // Used to controll the task scheduler inside the Thread Building Block

    std::string results_name = "schunk_L1_experiment";

    typedef AngularVariance<Params> fit_t; // Fitness function for the algorithm 
    typedef eval::Parallel<Params> eval_t;
    typedef gen::EvoFloat<Params::ea::genotype_dimensions, Params> gen_t;
    typedef phen::Parameters<gen_t, fit_t, Params> phen_t;
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
