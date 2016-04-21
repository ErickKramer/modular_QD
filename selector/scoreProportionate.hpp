#ifndef _SELECTOR_SCOREPROPORTIONATE_HPP
#define _SELECTOR_SCOREPROPORTIONATE_HPP

#include "tbb/parallel_reduce.h"
#include "tbb/parallel_sort.h"


namespace sferes{
  namespace selector{

    template <typename Phen, typename ValueSelector=getFitness>
    class ScoreProportionate{
    public:
      typedef boost::shared_ptr<Phen> indiv_t;      
      typedef typename std::vector<indiv_t >::iterator it_t;
      typedef typename std::vector<indiv_t >::const_iterator itc_t;
      
      
      /*indiv_t operator()()const
      {
	if(min!=max){ 
	  double r= misc::rand((double) 1.0);
	  itc_t it=_pop.cbegin();
	  double p = (ValueSelector::getValue(*it)-min)/(sum-_pop.size()*min);
	  while (p<r)
	    {
	      it++;
	      p+=(ValueSelector::getValue(*it)-min)/(sum-_pop.size()*min);
	    }
	  return *it;
	}else{ //they all have the same score
	  //tools::rgen_int_t rgen(0, v.size()-1);
	  //return v[rgen.rand()];
	  int x1 = misc::rand< int > (0, _pop.size());
	  return _pop[x1];
	}
	return _pop[0];//should not be reached
	
	}*/
      template<typename EA>
      void operator()(std::vector<indiv_t>& pop,const EA& ea){
	_init(ea.pop());
	if(min!=max){ 
	  for (auto& indiv : pop)
	    {
	      double r= misc::rand((double) 1.0);
	      itc_t it=_pop.cbegin();
	      double p = (ValueSelector::getValue(*it)-min)/(sum-_pop.size()*min);
	      while (p<r)
		{
		  it++;
		  p+=(ValueSelector::getValue(*it)-min)/(sum-_pop.size()*min);
		}
	      indiv= *it;
	    }
	}else{ //they all have the same score
	  for (auto& indiv : pop)
	    {
	      int x1 = misc::rand< int > (0, _pop.size());
	      indiv=_pop[x1];
	    }	  
	}
      }

      struct Comparator{
	template<typename T1, typename T2>
	bool operator()(const T1& t1, const T2& t2)const
	{
	  return ValueSelector::getValue(t1) > ValueSelector::getValue(t2); //descending order
	}
      };


    private:
      
      void _init(const std::vector<indiv_t>& pop)
      {
	_pop= std::vector<indiv_t>(pop);
	if(_pop.size()==0)
	  return;
	tbb::parallel_sort(_pop.begin(),_pop.end(),Comparator());
	max=ValueSelector::getValue(*_pop.begin());
	min=ValueSelector::getValue(*(--_pop.end()));
	  
	if(min!=max){
	  //	  tools::rgen_double_t rgen(0.0, 1.0);
	  //double r = rgen.rand(); 
	  

	  //double sum=0;
	  //for(it_t it=v.begin(); it!=v.end();++it)
	  //  sum+=((*it)->getScore()-min)/(max-min);
	  
	  typedef tbb::blocked_range<it_t> range_type;
	  sum= tbb::parallel_reduce(
					   range_type( _pop.begin(), _pop.end() ), 0.0,
					   [](const range_type& r, double value)->double {
					     return std::accumulate(r.begin(),r.end(),value,[]( double const & current, indiv_t const& p)
								    { return current + ValueSelector::getValue(p); });
					   },
					   std::plus<double>()
					   );
	  
	  
	}
      }
      

      std::vector< indiv_t > _pop;
      double max;
      double min;
      double sum;
    };

  }
}

#endif
