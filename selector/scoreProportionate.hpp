#ifndef _SELECTOR_SCOREPROPORTIONATE_HPP
#define _SELECTOR_SCOREPROPORTIONATE_HPP

#include "tbb/parallel_reduce.h"
#include "tbb/parallel_sort.h"

/*
How it works:
    - Selects an individual from the population biased to a particular score. 
    - Types of scores: Fitenss, Novelty, Curiosity
 */


namespace sferes{ // Group named sferes
    namespace selector{ // Group named selector 
        template <typename Phen, typename ValueSelector=getFitness> // Template  for the phenotype and type of ValueSelector
        class ScoreProportionate{
            public:
                typedef boost::shared_ptr<Phen> indiv_t; // Pointer of type Phen      
                typedef typename std::vector<indiv_t >::iterator it_t; // Iterator of a vector containing individuals
                typedef typename std::vector<indiv_t >::const_iterator itc_t; // Constant iterator of a vector containing individuals
      
                template<typename EA>
                void operator()(std::vector<indiv_t>& pop,const EA& ea) const {
                    this->operator()(pop,ea.pop()); // Virtual call to overload function call
                }

                void operator()(std::vector<indiv_t>& pop,const std::vector<indiv_t>& ea_pop) const {
                    std::pair<double, double> res = getMinAndSum(ea_pop); // Get the minimum value of a population and the sum
                    double min=res.first;
                    double sum=res.second-min*ea_pop.size();

                    for (auto& indiv : pop){ // Iterate over the individuals in the population
                        double r=0;
                        if(sum)
                            r= misc::rand((double) sum); // Random number between 0 and sum
                        itc_t it=ea_pop.cbegin();
                        double p = (ValueSelector::getValue(*it)-min);

                        while (p<r && it!=ea_pop.end()){
                            it++;
                            p+=(ValueSelector::getValue(*it)-min);
                        }
                        indiv= *it;
                    }
                }
            private:
                std::pair<double,double> getMinAndSum(const std::vector<indiv_t>& pop)const{
                    typedef tbb::blocked_range<itc_t> range_type;
                    
                    return tbb::parallel_reduce(
                            range_type( pop.begin(), pop.end() ), 
                            std::pair<double,double>(0,0),
                            [](const range_type& r, std::pair<double,double> value)->std::pair<double,double> {
                                // Lambda expression to find the minimum value in the population and the sum
                                for( auto it=r.begin(); it!=r.end(); ++it ){ // Iterate over the entire population
                                    value.first=std::min(value.first,ValueSelector::getValue(*it)); 
                                    value.second+=ValueSelector::getValue(*it);
                                }
                                return value;
                            },
                            []( std::pair<double,double> x, std::pair<double,double> y )->std::pair<double,double> {
                                // Lambda expression, Not sure what it does
                                return {std::min(x.first,y.first), x.second+y.second};
                            }
                    );
                }
        }; // class ScoreProportionate
    } // namespace selector
} // namespace sferes

#endif
