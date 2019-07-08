#ifndef STAT_SELECTION_HPP_
#define STAT_SELECTION_HPP_

#include <numeric>
#include <boost/multi_array.hpp>
#include <sferes/stat/stat.hpp>

//#define MAP_WRITE_PARENTS

namespace sferes{
    namespace stat{
        SFERES_STAT(Selection, Stat){
            public:
                typedef boost::shared_ptr<Phen> phen_t;
                typedef boost::multi_array<phen_t, Params::ea::behav_dim> array_t;
                template<typename E>
                void refresh(const E& ea){
                    if(!ofs.is_open()){
                        std::string prefix("selection");	
                        std::string fname =  ea.res_dir() + "/"
                                                          + prefix
                                                          + std::string(".dat");
                        ofs.open(fname.c_str());
                    }

                    assert(ea.offspring().size()==ea.parents().size()); // Assert both offspring and parent have the same size
                    assert(ea.offspring().size()==ea.added().size());
      
                    for(size_t i=0;i<ea.offspring().size();i++){ // Iterate over the size of the offspring
                        ofs<<ea.gen()<<"  ";
                        for(size_t dim = 0; dim < ea.parents()[i]->fit().desc().size(); ++dim) // Iterate over the dimensions of the fitnes of the parent
                            ofs<<ea.parents()[i]->fit().desc()[dim]<<" ";
                    
                        ofs<<"   "<<ea.parents()[i]->fit().value()<<"   "; // Record the value of the parent 
                        ofs<<"   "<<ea.parents()[i]->fit().novelty()<<"   "; // Record novelty of the parent
                        ofs<<"   "<<ea.parents()[i]->fit().local_quality()<<"   "; // Record local quality of the parent
                        ofs<<"   "<<ea.parents()[i]->fit().curiosity()<<"   "; // Record curiosity of the parent

                        for(size_t dim = 0; dim < ea.offspring()[i]->fit().desc().size(); ++dim) // Iterate over the dimensions of the offspring
                            ofs<<ea.offspring()[i]->fit().desc()[dim]<<" ";
                    
                        ofs<<"   "<<ea.offspring()[i]->fit().value()<<"   "; // Record value of the offspring
                        ofs<<"   "<<ea.offspring()[i]->fit().novelty()<<"   "; // Record novelty of th
                        ofs<<"   "<<ea.offspring()[i]->fit().local_quality()<<"   "; // Record local quality of the offspring    
                        ofs<<"   "<<ea.offspring()[i]->fit().curiosity()<<"   "; // Record curiosity of the parent 
                        ofs<<"   "<<ea.added()[i]<<std::endl;
                    }
                }
            protected:
                std::ofstream ofs;

        }; // SFERES_STAT
    } // namespace stat
} // namespace sferes

#endif
