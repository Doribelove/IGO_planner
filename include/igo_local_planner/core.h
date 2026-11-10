#pragma once
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace igo_local_planner {
struct Point { double x=0, y=0; };
struct State {
  double x=0, y=0, yaw=0, v=0, omega=0;
  // Wheel angle remains observable at zero velocity, unlike omega / v.
  double steering=std::numeric_limits<double>::quiet_NaN();
};
struct Control { double v=0, curvature=0; };
struct Parameters {
  double wheelbase=.65, radius=1.35, forward=.8, reverse=.3, angular=.6;
  double acceleration=2., angular_acceleration=.5, period=.1, horizon=4.;
  double deceleration=.18, stopping_delay=1.2;
  double steering_rate=.6, steering_weight=.5;
  double xy_tolerance=.5, yaw_tolerance=.3, lookahead=4.;
  double path_weight=5., heading_weight=1., obstacle_weight=5., progress_weight=5.;
  double mean_rate=.65, variance_rate=.25, elite_fraction=.25;
  int knots=10, samples=48, iterations=4, direction=0;
  bool allow_reverse=true, unknown_obstacle=true, overwrite_orientation=true, goal_yaw_required=true;
};
// A coherent local costmap snapshot. The integral image only accelerates hard
// obstacle queries; an intersecting box is checked against the whole polygon.
struct Grid {
  unsigned width=0, height=0;
  double resolution=.1, origin_x=0, origin_y=0;
  bool unknown_obstacle=true;
  std::vector<unsigned char> cells;
  std::vector<unsigned> integral;
  void prepare();
  bool blocked(int x,int y) const;
  bool footprint(const State&,const std::vector<Point>&,double* cost=nullptr) const;
};
struct Result {
  bool valid=false;
  Control command;
  double omega=0, cost=0, elapsed_ms=0;
  unsigned evaluations=0;
  std::vector<State> trajectory;
  std::string reason;
};
class Optimizer {
 public:
  explicit Optimizer(unsigned seed=17): rng_(seed) {}
  Result solve(const State&,const std::vector<State>& reference,const Grid&,
               const std::vector<Point>& footprint,const Parameters&,bool terminal);
  void reset();
  static State advance(const State&,Control,double,const Parameters&);
 private:
  std::mt19937 rng_;
  std::vector<double> warm_;
};
double angle(double);
} // namespace igo_local_planner
