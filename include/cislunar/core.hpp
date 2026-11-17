#pragma once
#include <array>
#include <string>
namespace cislunar {
// Normalized circular restricted three-body coordinates in the rotating frame.
using State = std::array<double, 6>;
using Vector = std::array<double, 3>;
constexpr double earth_moon_mu = 0.012150585609624;
struct Config {
 double max_acceleration = 1e-4;
 double max_delta_v = 0.01;
 double min_primary_distance = 0.02;
 double min_secondary_distance = 0.005;
 double max_sensor_age = 0.01;
 double min_battery = 0.2;
 double max_temperature = 350;
 double kp = 0.002;
 double kd = 0.02;
};
struct Observation {
 State state{};
 double sensor_age = 0;
 double battery = 1;
 double temperature = 290;
 bool sensor_valid = true;
 bool ground_link = true;
};
enum class Mode { nominal, disconnected, safe };
struct Decision { Vector acceleration{}; Mode mode = Mode::safe; std::string reason; };
struct Model {
 // Optional pre-trained linear correction. Demonstration schema, not flight heritage.
 std::string version;
 std::array<double, 18> weights{};
 double validation_error = 0;
};
class ModelStore {
 Model active_, previous_, staged_;
 bool has_active_ = false, has_previous_ = false, has_staged_ = false;
public:
 bool stage(const Model& model);
 bool activate(double maximum_error);
 bool rollback();
 const Model* active() const { return has_active_ ? &active_ : nullptr; }
};
class Executive {
 Config config_;
 double used_delta_v_ = 0;
public:
 explicit Executive(Config config = {});
 Decision step(const Observation&, const State& target, double dt, const Model* model = nullptr);
 double used_delta_v() const { return used_delta_v_; }
};
State derivative(const State&, const Vector& acceleration = {});
State propagate(const State&, const Vector&, double dt);
double jacobi(const State&);
bool finite(const State&);
const char* mode_name(Mode);
}
