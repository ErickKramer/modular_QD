#ifndef QD_SELECTOR_RANDOM_HPP
#define QD_SELECTOR_RANDOM_HPP

/*
How it works:
    - Individuals are selected using a uniform probability from the individuals stored in the collection.
    - Used in MAP-Elites.
 */

namespace sferes{    // Group named sferes
    namespace selector{     // Group named selector
        template <typename Phen>    // Template for the phenotype
        struct Random{  // Struct following the template for the Phenotype
            typedef boost::shared_ptr<Phen> indiv_t;    // Pointer of type Phen
    
            //void init(const std::vector<indiv_t>& pop){
                //  _pop= &pop;
            //}

            /*indiv_t operator()()const{
                int x1 = misc::rand< int > (0, _pop->size());
                return _pop->operator[](x1);
            }*/
            template<typename EA>   // Template for the Evolutionary Algorithm
            void operator()(std::vector<indiv_t>& pop,const EA& ea)const{   // Overload function call
                for (auto& indiv : pop){    // Iterate over individual in the population
                    int x1 = misc::rand< int > (0, ea.pop().size());    
                    // Produces an index based on a uniform distribution (See sferes/sferes/misc/rand.hpp)
                    indiv=ea.pop()[x1];
                }
            }
        //const std::vector<indiv_t>*  _pop;
        };
    }
}
#endif
