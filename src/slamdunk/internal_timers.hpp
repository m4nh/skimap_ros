
#ifndef SLAM_DUNK_INTERNAL_TIMERS_HPP
#define SLAM_DUNK_INTERNAL_TIMERS_HPP

#ifdef SLAMDUNK_TIMERS_ENABLED
  #include <map>
  #include <boost/timer/timer.hpp>
  #include <boost/date_time/posix_time/posix_time_types.hpp>
  namespace
  {
    static std::map<std::string, std::pair<unsigned,uint64_t> > g_timings_;
    static boost::posix_time::ptime g_base_time_;
    class slam_dunk_cpu_timer : public boost::timer::cpu_timer
    {
      public:
        explicit slam_dunk_cpu_timer(const std::string& title)
          : title_(title)
        { start(); }
        ~slam_dunk_cpu_timer()
        {
          stop();
          std::map<std::string, std::pair<unsigned,uint64_t> >::iterator tIt =
              g_timings_.insert(std::make_pair(title_, std::pair<unsigned,uint64_t>(0,0))).first;
          tIt->second.first += 1;
          tIt->second.second += elapsed().wall;

          // Print average timings every N seconds
          boost::posix_time::ptime current_time = boost::posix_time::second_clock::local_time();
          if(g_base_time_.is_not_a_date_time())
            g_base_time_ = current_time;
          if((current_time - g_base_time_).total_seconds() >= 5)
          {
            std::ostringstream outstr;
            outstr << "[SLAM DUNK TIMERS]\n";
            for(tIt = g_timings_.begin(); tIt != g_timings_.end(); ++tIt)
            {
              outstr << ("- "+tIt->first+": average speed (msec) ") << ((double(tIt->second.second)*1e-6)/double(tIt->second.first)) << std::endl;
              tIt->second.first = 0;
              tIt->second.second = 0;
            }
            std::cout << outstr.str();
            g_base_time_ = current_time;
          }
        }

      private:
        const std::string title_;
    };
  }
  #define SLAM_DUNK_AUTO_CPU_TIMER(title) slam_dunk_cpu_timer scoped_timer(title)
#else // SLAMDUNK_TIMERS_ENABLED
  #define SLAM_DUNK_AUTO_CPU_TIMER(title)
#endif // SLAMDUNK_TIMERS_ENABLED

#endif // SLAM_DUNK_INTERNAL_TIMERS_HPP
