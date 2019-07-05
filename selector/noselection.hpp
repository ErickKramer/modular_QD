#ifndef QD_SELECTOR_NOSELECTION_HPP
#define QD_SELECTOR_NOSELECTION_HPP

/*
How it works:
    - Random individuals are generated to produce the next batch.
    - Resembles random sampling of the search space
 */

namespace sferes{   // Group named sferes
    namespace selector{     // Group named selector
        template <typename Phen>
        struct NoSelection{     // Struct following the template for the Phenotype.
            typedef boost::shared_ptr<Phen> indiv_t; // Pointer of type Phen
            
            template<typename EA>   // Template for the evolutionary algorithm  
            void operator()(std::vector<indiv_t>& pop,const EA& ea)const{   // Overload function call
                for (auto& indiv : pop){ // Iterate over every individual in the population
                    indiv = boost::shared_ptr<Phen>(new Phen());    // Create a new shared_pointer individual
                    indiv->random();    //Individual is generated randomly
                }
            }
        }; // struct NoSelection
    } // namespace selector
} // namespace sferes
#endif
