#ifndef STAT_CONTAINER_HPP_
#define STAT_CONTAINER_HPP_

#include <numeric>
#include <sferes/stat/stat.hpp>

//#define MAP_WRITE_PARENTS

namespace sferes{
    namespace stat{
        SFERES_STAT(Container, Stat){
            public:
                template<typename E>
                void refresh(const E& ea){ // Verify that the dump period has been reached
                    if (ea.gen() % Params::pop::dump_period == 0)
                        _write_container(std::string("archive_"), ea); // Generates the results file
                }
                
                template<typename EA>
                void _write_container(const std::string& prefix,
                                      const EA& ea) const{
                    std::cout << "writing..." << prefix << ea.gen() << std::endl;
                    std::string fname = ea.res_dir() + "/"
                                        + prefix
                                        + boost::lexical_cast<std::string>(ea.gen())
                                        + std::string(".dat"); // Name of the result file 

                    std::ofstream ofs(fname.c_str()); // Stream class to write files
                    size_t offset = 0; // Variable used to represent the individual per row in the archive_xx.dat file 
                    ofs.precision(17); // Decimal precision for the next floating value insterted
                    for(auto it = ea.pop().begin(); it != ea.pop().end(); ++it){ // Iterate over the population
                        ofs << offset << "    ";
                        for(size_t dim = 0; dim < (*it)->fit().desc().size(); ++dim) // Iterate over the dimensions
                            ofs << (*it)->fit().desc()[dim] << " "; 
                            //ofs << " " << array(idx)->fit().value() << std::endl;

                        ofs << " " << (*it)->fit().value() << "         ";

                        for(size_t dim = 0; dim < (*it)->size(); ++dim)
                            ofs << (*it)->data(dim) << " ";
                        
                        ofs<<std::endl;
                        ++offset;
                    }
                }
        }; // SFERES_STAT
    } // namespace stat
} // namespace sferes

#endif
