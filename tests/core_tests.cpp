#include "cislunar/core.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace cislunar;
void check(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
int main(){try{
 Observation o;o.state={0.82,0,0.01,0,0.12,0};auto goal=o.state;goal[0]+=1;
 Executive e;auto d=e.step(o,goal,0.001);check(d.mode==Mode::nominal,"nominal");check(std::hypot(d.acceleration[0],d.acceleration[1],d.acceleration[2])<=1.000001e-4,"acceleration clamp");
 o.ground_link=false;check(e.step(o,goal,0.001).mode==Mode::disconnected,"ground independence");
 o.sensor_valid=false;auto before=e.used_delta_v();d=e.step(o,goal,0.001);check(d.mode==Mode::safe&&d.acceleration==Vector{},"dropout safe");check(e.used_delta_v()==before,"safe budget");
 o.sensor_valid=true;o.sensor_age=1;check(e.step(o,goal,0.001).mode==Mode::safe,"stale");o.sensor_age=0;
 o.battery=0.1;check(e.step(o,goal,0.001).mode==Mode::safe,"battery");o.battery=1;
 o.temperature=400;check(e.step(o,goal,0.001).mode==Mode::safe,"thermal");o.temperature=290;
 o.state[0]=std::numeric_limits<double>::quiet_NaN();check(e.step(o,goal,0.001).mode==Mode::safe,"NaN");o.state={0.82,0,0.01,0,0.12,0};
 check(e.step(o,goal,-1).mode==Mode::safe,"invalid dt");
 o.state={-earth_moon_mu,0,0,0,0,0};check(e.step(o,goal,0.001).mode==Mode::safe,"keep out");
 Config cfg;cfg.max_delta_v=1e-12;Executive small(cfg);o.state={0.82,0,0.01,0,0.12,0};check(small.step(o,goal,0.001).mode==Mode::safe,"budget exhaustion");
 ModelStore store;Model m;m.version="v1";m.validation_error=0.1;check(store.stage(m),"stage");check(!store.activate(0.01),"validation rejection");check(!store.active(),"no rejected activation");check(store.activate(0.2),"activate");m.version="v2";check(store.stage(m)&&store.activate(0.2),"replacement");check(store.rollback()&&store.active()->version=="v1","rollback");check(!store.rollback(),"rollback once");
 m.weights[0]=std::numeric_limits<double>::infinity();check(!store.stage(m),"model finite");check(e.step(o,goal,0.001,&m).mode==Mode::safe,"runtime invalid model");
 State s={0.82,0,0.01,0,0.12,0};double initial=jacobi(s);for(int i=0;i<10000;++i)s=propagate(s,{},1e-5);check(std::abs(jacobi(s)-initial)<1e-9,"Jacobi conservation");
 bool threw=false;try{propagate(s,{},0);}catch(const std::invalid_argument&){threw=true;}check(threw,"propagator input guard");
 std::cout<<"Core checks passed\n";return 0;
 }catch(const std::exception& ex){std::cerr<<ex.what()<<'\n';return 1;}}
