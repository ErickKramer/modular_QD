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

#include <iostream>
#include <cmath>

#include <boost/foreach.hpp>
#include <boost/multi_array.hpp>
#include <boost/array.hpp>
#include <boost/fusion/algorithm/iteration/for_each.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <Eigen/Core>

#include <sferes/eval/parallel.hpp>
#include <sferes/gen/evo_float.hpp>
#include <sferes/phen/parameters.hpp>
#include <sferes/modif/dummy.hpp>
#include <sferes/run.hpp>
#include <sferes/stat/best_fit.hpp>

#include "quality_diversity.hpp"
#include "fit_qd.hpp"

#include "selector.hpp"
#include "container.hpp"
#include "stat.hpp"

#include "arm_hori.hpp"

/*#define NO_MPI
#ifdef GRAPHIC
#define NO_PARALLEL
#include "renderer/osg_visitor.hh"
#endif

#ifndef NO_PARALLEL
#include <sferes/eval/parallel.hpp>
#ifndef NO_MPI
#include <sferes/eval/mpi.hpp>
#endif
#else
#include <sferes/eval/eval.hpp>
#endif*/



using namespace sferes::gen::evo_float;


struct Params{
    struct nov{
        SFERES_CONST size_t deep=3; //7x7
        SFERES_CONST size_t k=15; // Nearest neighbors
        SFERES_CONST double l=0.01; // Distance threshold for the archive container
        SFERES_CONST double eps=0.1; // epsilon for exclusive e-dominance
    };

    struct ea{
        /*SFERES_CONST size_t res_x = 256;
      SFERES_CONST size_t res_y = 256;*/

        SFERES_CONST size_t behav_dim = 2; // x,y position
        SFERES_ARRAY(size_t, behav_shape, 100, 100);

    };

    struct pareto{
        SFERES_CONST bool genoDiv =true;
    };

    struct pop{
        // number of initial random points
        //SFERES_CONST size_t init_size = 100;
        // size of a batch
        SFERES_CONST size_t size = 200; // Size of the population (batch)
        SFERES_CONST size_t nb_gen = 50001; // Number of generations
        SFERES_CONST size_t dump_period = 1000; // Refreshment rate
    };

    struct parameters{
        SFERES_CONST float min = -1; // Limits of the phenotype parameters
        SFERES_CONST float max = 1;
    };

    struct evo_float{
        SFERES_CONST float cross_rate = 0.0f; // Cross-over rate
        SFERES_CONST float mutation_rate = 0.125f;
        SFERES_CONST float eta_m = 10.0f; // Parameter for the polynomial mutation
        SFERES_CONST float eta_c = 10.0f; // Parameter for the polynomial mutation
        SFERES_CONST mutation_t mutation_type = polynomial; // Type of polynomial
        SFERES_CONST cross_over_t cross_over_type = sbx; // Type of cross-over
    };

};



FIT_QD(ArmFit){
    // Minimizing the variance of the angular position of the joints. 
    // Captures the idea that all the joints of the arm should contribute equally to 
    // the movement
    public:
        template<typename Indiv>
        void eval(Indiv& ind){
            Eigen::VectorXd angle(ind.size());
            for (size_t i = 0; i < ind.size(); ++i)
                angle[i] = ind.data(i)*M_PI/2; // Constraint the joint angles between [-pi/2, pi/2]
            
            this->_value = - sqrt((angle.array()-angle.mean()).square().mean());

            Eigen::Vector3d pos=robot::Arm::forward_model(angle); // Get the position of the end effector
            float L=robot::Arm::max_length()*1.1; // Total length of the robot times a boundary factor
            // Normalized final position of the gripper according to a square bounding box with values
            // between 0 and 1
            std::vector<float> data = {(float) (pos[0]+L)/(2*L), (float) (pos[1]+L)/(2*L)};
            //this->set_desc(ind.gen().data(0), ind.gen().data(1));
            this->set_desc(data);
        }
};

int main(){
    srand(time(NULL));
    tbb::task_scheduler_init init(20);  

    using namespace sferes;

    typedef ArmFit<Params> fit_t;
    typedef gen::EvoFloat<8, Params> gen_t;
    typedef phen::Parameters<gen_t, fit_t, Params> phen_t;
    typedef eval::Parallel<Params> eval_t;
    /*#ifndef NO_PARALLEL
    typedef eval::Parallel<Params> eval_t;
    #else
    typedef eval::Eval<Params> eval_t;
    #endif*/

    //    typedef boost::fusion::vector<stat::Map<phen_t, Params>, stat::BestFit<phen_t, Params>,stat::Selection<phen_t,Params> > stat_t;
    //

    typedef modif::Dummy<> modifier_t;
    
    // Checking which container was selected for the experiment during compilation
    #if defined(GRID)
        typedef container::Grid<phen_t, Params> container_t;
        //typedef boost::fusion::vector<stat::Map<phen_t, Params>,stat::Progress<phen_t, Params> > stat_t;

    #else // ARCHIVE
        typedef container::Archive<phen_t, Params> container_t;
        //typedef boost::fusion::vector<stat::Archive<phen_t, Params>,stat::Progress<phen_t, Params> > stat_t;
        //typedef boost::fusion::vector<stat::Archive<phen_t, Params>, stat::Selection<phen_t,Params> > stat_t;
    #endif

    typedef boost::fusion::vector<stat::Container<phen_t, Params>,stat::Progress<phen_t, Params> > stat_t;

    // Checking the type of selector chosen for the experiment during compilation
    #if defined(RANDOM)
        typedef selector::Random<phen_t> select_t;
    #elif defined(FITNESS)
        typedef selector::ScoreProportionate<phen_t,selector::getFitness> select_t;
    #elif defined(NOVELTY)
        typedef selector::ScoreProportionate<phen_t,selector::getNovelty> select_t;
    #elif defined(CURIOSITY)
        typedef selector::ScoreProportionate<phen_t,selector::getCuriosity> select_t;
    #elif defined(POPFITNESS)
        typedef selector::PopulationBased<phen_t, selector::ScoreProportionate<phen_t, selector::getFitness> > select_t;
    #elif defined(POPNOVELTY)
        typedef selector::PopulationBased<phen_t, selector::ScoreProportionate<phen_t, selector::getNovelty> > select_t;
    #elif defined(POPCURIOSITY)
        typedef selector::PopulationBased<phen_t, selector::ScoreProportionate<phen_t, selector::getCuriosity> > select_t;
    #elif defined(TOURFITNESS)
        typedef selector::TournamentBased<phen_t,selector::getFitness> select_t;
    #elif defined(TOURNOVELTY)
        typedef selector::TournamentBased<phen_t,selector::getNovelty> select_t;
    #elif defined(TOURCURIOSITY)
        typedef selector::TournamentBased<phen_t,selector::getCuriosity> select_t;
    #elif defined(PARETO) //NSLC
        typedef selector::ParetoBased<phen_t,boost::fusion::vector<selector::getNovelty,selector::getLocalQuality>, Params > select_t;
    #else // NOSELECTION
        typedef selector::NoSelection<phen_t> select_t;

    #endif

    typedef ea::QualityDiversity<phen_t, eval_t, stat_t, modifier_t, select_t, container_t, Params> ea_t;

    ea_t ea;
    ea.run();
}
