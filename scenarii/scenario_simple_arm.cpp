//author: Erick Kramer
//email: erickkramer@gmail.com

#include <iostream>
#include <cmath>

#include <boost/foreach.hpp> // TODO
#include <boost/multi_array.hpp> // TODO
#include <boost/array.hpp> // TODO
#include <boost/fusion/algorithm/iteration/for_each.hpp> // TODO
#include <boost/fusion/include/for_each.hpp> // TODO

#include <Eigen/Core> // TODO

#include <sferes/eval/parallel.hpp> // Multicore evaluator
#include <sferes/gen/evo_float.hpp> // Genotype of real (float) numbers
#include <sferes/phen/parameters.hpp> // Phenotype parameters
#include <sferes/run.hpp> // Contain the execution methods for sferes
// #include <sferes/stat/best_fit.hpp> // Used to write the bestfit.dat file

#include "quality_diversity.hpp" // Declaration of the QualityDiversity class
#include "fit_qd.hpp" // Contains declaration and set of novelty, curiosity, local quality, descriptor and value

#include "selector.hpp" // Include the header files for the different selector
#include "container.hpp" // Includes the header files for the containers
#include "stat.hpp" // Include the header files for the different statistical methods


int main(){

    
}
