#include "cislunar/c_api.h"
#include "cislunar/core.hpp"
#include <algorithm>
struct ca_controller { cislunar::Executive executive; };
extern "C" ca_controller* ca_create(void){try{return new ca_controller;}catch(...){return nullptr;}}
extern "C" void ca_destroy(ca_controller* c){delete c;}
extern "C" int ca_step(ca_controller* c,const ca_observation* o,const double* target,double dt,ca_decision* out){
 if(out)*out=ca_decision{{0,0,0},2,0};
 if(!c||!o||!target||!out)return -1;
 try{cislunar::Observation obs;std::copy(o->state,o->state+6,obs.state.begin());
 obs.sensor_age=o->sensor_age;obs.battery=o->battery;obs.temperature=o->temperature;obs.sensor_valid=o->sensor_valid!=0;obs.ground_link=o->ground_link!=0;
 cislunar::State goal;std::copy(target,target+6,goal.begin());auto d=c->executive.step(obs,goal,dt);
 std::copy(d.acceleration.begin(),d.acceleration.end(),out->acceleration);out->mode=static_cast<int>(d.mode);out->used_delta_v=c->executive.used_delta_v();return 0;
 }catch(...){return -2;}
}
