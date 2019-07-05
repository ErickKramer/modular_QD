#ifndef QD_SELECTOR_NOSELECTION_HPP
#define QD_SELECTOR_NOSELECTION_HPP

/*
How it works:
    - Random individuals are generated to produce the next batch.
    - Resembles random sampling of the search space
 */

namespace sferes{
    // Group named sferes
    namespace selector{
        // Group named selector
        template <typename Phen>
        struct NoSelection{
            // Struct following the template for the Phenotype.
            // Every instantiation of the word "Phen" will be replaced when the structure is declared by the 
            // type passed to the template
            typedef boost::shared_ptr<Phen> indiv_t;
            
            template<typename EA>
            // Template for the evolutionary algorithm  
            void operator()(std::vector<indiv_t>& pop,const EA& ea)const{
                // Overload function call
                for (auto& indiv : pop){
                    indiv = boost::shared_ptr<Phen>(new Phen());
                    indiv->random();
                }
            }
        };
    }
}
#endif
