#include "cislunar/core.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
int main(int argc,char** argv){try{
 unsigned steps=2000;std::string output="telemetry.csv",scenario="nominal";
 for(int i=1;i<argc;++i){std::string arg=argv[i];if(arg=="--help"){std::cout<<"cislunar_sim [--steps 1..1000000] [--output path] [--scenario nominal|outage|dropout|thermal]\n";return 0;}
 if(i+1>=argc)throw std::invalid_argument("missing option value");
 std::string value=argv[++i];
 if(arg=="--steps"){std::size_t n=0;auto count=std::stoul(value,&n);if(n!=value.size()||count==0||count>1000000)throw std::invalid_argument("invalid steps");steps=static_cast<unsigned>(count);}
 else if(arg=="--output")output=value;else if(arg=="--scenario")scenario=value;else throw std::invalid_argument("unknown option");}
 if(scenario!="nominal"&&scenario!="outage"&&scenario!="dropout"&&scenario!="thermal")throw std::invalid_argument("unknown scenario");
 std::ofstream log(output);if(!log)throw std::runtime_error("cannot open telemetry output");
 log<<std::setprecision(17)<<"time,x,y,z,vx,vy,vz,ax,ay,az,mode,delta_v,reason\n";
 cislunar::Executive executive;cislunar::Observation obs;obs.state={0.82,0,0.01,0,0.12,0};const auto target=obs.state;const double dt=0.0001;
 for(unsigned i=0;i<steps;++i){bool fault=i>=steps/3&&i<2*steps/3;obs.ground_link=!(fault&&scenario=="outage");obs.sensor_valid=!(fault&&scenario=="dropout");obs.temperature=fault&&scenario=="thermal"?370:290;
 auto d=executive.step(obs,target,dt);log<<i*dt;for(auto v:obs.state)log<<','<<v;for(auto v:d.acceleration)log<<','<<v;log<<','<<cislunar::mode_name(d.mode)<<','<<executive.used_delta_v()<<','<<d.reason<<'\n';
 obs.state=cislunar::propagate(obs.state,d.acceleration,dt);}
 log.close();if(!log)throw std::runtime_error("telemetry write failed");std::cout<<"Wrote "<<steps<<" samples to "<<output<<'\n';return 0;
 }catch(const std::exception& e){std::cerr<<"error: "<<e.what()<<'\n';return 1;}}
