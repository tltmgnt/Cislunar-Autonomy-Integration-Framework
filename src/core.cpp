#include "cislunar/core.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace cislunar {
namespace {
template<class T> bool all_finite(const T& x) { for(double v:x) if(!std::isfinite(v)) return false; return true; }
double norm(const Vector& v) { return std::hypot(v[0],v[1],v[2]); }
void distances(const State& s,double& a,double& b) {
 a=std::hypot(s[0]+earth_moon_mu,s[1],s[2]);
 b=std::hypot(s[0]-1+earth_moon_mu,s[1],s[2]);
}
}
bool finite(const State& s) { return all_finite(s); }
State derivative(const State& s,const Vector& u) {
 if(!finite(s)||!all_finite(u)) throw std::invalid_argument("non-finite dynamics input");
 double a,b; distances(s,a,b);
 if(a<1e-9||b<1e-9) throw std::domain_error("primary singularity");
 double f=(1-earth_moon_mu)/(a*a*a),g=earth_moon_mu/(b*b*b);
 return {s[3],s[4],s[5],2*s[4]+s[0]-f*(s[0]+earth_moon_mu)-g*(s[0]-1+earth_moon_mu)+u[0],
 -2*s[3]+s[1]-(f+g)*s[1]+u[1],-(f+g)*s[2]+u[2]};
}
State propagate(const State& s,const Vector& u,double dt) {
 if(!std::isfinite(dt)||dt<=0||dt>0.01) throw std::invalid_argument("dt must be in (0,0.01]");
 auto shifted=[&](const State& k,double h){State t=s;for(unsigned i=0;i<6;++i)t[i]+=h*k[i];return t;};
 auto a=derivative(s,u),b=derivative(shifted(a,dt/2),u),c=derivative(shifted(b,dt/2),u),d=derivative(shifted(c,dt),u);
 State out=s;for(unsigned i=0;i<6;++i)out[i]+=dt*(a[i]+2*b[i]+2*c[i]+d[i])/6;
 if(!finite(out)) throw std::overflow_error("propagation overflow");
 return out;
}
double jacobi(const State& s) {
 if(!finite(s)) throw std::invalid_argument("non-finite state");
 double a,b;distances(s,a,b);if(a<1e-9||b<1e-9)throw std::domain_error("primary singularity");
 return s[0]*s[0]+s[1]*s[1]+2*((1-earth_moon_mu)/a+earth_moon_mu/b)-s[3]*s[3]-s[4]*s[4]-s[5]*s[5];
}
bool ModelStore::stage(const Model& m) {
 has_staged_=false;
 if(m.version.empty()||m.version.size()>128||!all_finite(m.weights)||!std::isfinite(m.validation_error)||m.validation_error<0)return false;
 staged_=m;has_staged_=true;return true;
}
bool ModelStore::activate(double limit) {
 if(!has_staged_||!std::isfinite(limit)||limit<0||staged_.validation_error>limit)return false;
 previous_=active_;has_previous_=has_active_;active_=staged_;has_active_=true;has_staged_=false;return true;
}
bool ModelStore::rollback(){if(!has_previous_)return false;active_=previous_;has_active_=true;has_previous_=false;return true;}
Executive::Executive(Config c):config_(c) {
 const double values[]={c.max_acceleration,c.max_delta_v,c.min_primary_distance,c.min_secondary_distance,c.max_sensor_age,c.max_temperature,c.kp,c.kd};
 for(double v:values)if(!std::isfinite(v)||v<=0)throw std::invalid_argument("invalid configuration");
 if(!std::isfinite(c.min_battery)||c.min_battery<0||c.min_battery>1)throw std::invalid_argument("invalid battery bound");
}
Decision Executive::step(const Observation& o,const State& target,double dt,const Model* model) {
 auto safe=[](const char* why){return Decision{{},Mode::safe,why};};
 if(!std::isfinite(dt)||dt<=0||dt>0.01)return safe("invalid timestep");
 if(!o.sensor_valid||!finite(o.state)||!finite(target)||!std::isfinite(o.sensor_age)||o.sensor_age<0||o.sensor_age>config_.max_sensor_age)return safe("untrusted navigation");
 if(!std::isfinite(o.battery)||o.battery<config_.min_battery||o.battery>1||!std::isfinite(o.temperature)||o.temperature<=0||o.temperature>config_.max_temperature)return safe("subsystem limit");
 double a,b;distances(o.state,a,b);
 if(a<=config_.min_primary_distance||b<=config_.min_secondary_distance)return safe("keep-out violation");
 Vector u{};
 for(unsigned i=0;i<3;++i)u[i]=config_.kp*(target[i]-o.state[i])+config_.kd*(target[i+3]-o.state[i+3]);
 if(model){
  if(!all_finite(model->weights))return safe("invalid model output");
  for(unsigned i=0;i<3;++i)for(unsigned j=0;j<6;++j)u[i]+=model->weights[i*6+j]*(target[j]-o.state[j]);
 }
 if(!all_finite(u))return safe("invalid control output");
 double magnitude=norm(u);
 if(!std::isfinite(magnitude))return safe("invalid control magnitude");
 if(magnitude>config_.max_acceleration){for(double& v:u)v*=config_.max_acceleration/magnitude;}
 double expenditure=norm(u)*dt;
 if(used_delta_v_+expenditure>config_.max_delta_v)return safe("maneuver budget exhausted");
 // Endpoint guard is a demonstration check, not continuous-time collision assurance.
 try {auto next=propagate(o.state,u,dt);distances(next,a,b);
 if(a<=config_.min_primary_distance||b<=config_.min_secondary_distance)return safe("predicted keep-out violation");
 }catch(const std::exception&){return safe("prediction failure");}
 used_delta_v_+=expenditure;
 return {u,o.ground_link?Mode::nominal:Mode::disconnected,o.ground_link?"bounded control":"onboard control without ground"};
}
const char* mode_name(Mode m){switch(m){case Mode::nominal:return "nominal";case Mode::disconnected:return "disconnected";case Mode::safe:return "safe";}return "unknown";}
}
