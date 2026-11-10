#include "igo_local_planner/core.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>

namespace igo_local_planner {
namespace {
double clamp(double x,double a,double b) { return std::max(a,std::min(b,x)); }
double sq(double x) { return x*x; }
double wheel_angle(const State& s,const Parameters& p) {
  if(std::isfinite(s.steering)) return s.steering;
  return std::abs(s.v)>1e-8?std::atan(p.wheelbase*s.omega/s.v):0.;
}
bool overlap(const std::vector<Point>& p,double x,double y,double half) {
  // SAT for a convex footprint and a closed grid cell, including cell interior.
  for(size_t i=0;i<p.size();++i) {
    const auto &a=p[i], &b=p[(i+1)%p.size()];
    const double nx=-(b.y-a.y),ny=b.x-a.x;
    double lo=1e100,hi=-1e100;
    for(auto q:p) { double d=q.x*nx+q.y*ny; lo=std::min(lo,d);hi=std::max(hi,d); }
    const double center=x*nx+y*ny,r=half*(std::abs(nx)+std::abs(ny));
    if(center+r<lo-1e-9 || center-r>hi+1e-9) return false;
  }
  return true; // world X/Y axes already tested by the cell bounding box
}
struct Match { double distance=0,yaw=0,arc=0; size_t index=0; };
Match nearest(const State& s,const std::vector<State>& path,const std::vector<double>& arc,
              size_t previous=0) {
  Match best; best.distance=1e100; best.index=previous;
  if(path.size()==1) return {std::hypot(s.x-path[0].x,s.y-path[0].y),path[0].yaw,0,0};
  // Local monotone matching avoids shortcuts at crossing/returning sweep lines.
  for(size_t j=previous;j+1<path.size();++j) {
    if(arc[j]-arc[previous]>2.) break;
    const double dx=path[j+1].x-path[j].x,dy=path[j+1].y-path[j].y;
    const double t=clamp(((s.x-path[j].x)*dx+(s.y-path[j].y)*dy)/(sq(dx)+sq(dy)+1e-15),0,1);
    const double d=std::hypot(s.x-path[j].x-t*dx,s.y-path[j].y-t*dy);
    if(d<best.distance) best={d,angle(path[j].yaw+t*angle(path[j+1].yaw-path[j].yaw)),
                              arc[j]+t*(arc[j+1]-arc[j]),j};
  }
  return best;
}
Control sampled(const std::vector<double>& u,double time,const Parameters& p) {
  const double f=clamp(time/(p.horizon/(p.knots-1)),0,p.knots-1.);
  const int a=int(f),b=std::min(a+1,p.knots-1);const double t=f-a;
  return {(1-t)*u[2*a]+t*u[2*b],(1-t)*u[2*a+1]+t*u[2*b+1]};
}
bool sweep(const State& a,const State& b,const Grid& grid,const std::vector<Point>& polygon,
           double radius,double* soft) {
  // Bound displacement of every footprint point, including the front brush.
  const double motion=std::hypot(b.x-a.x,b.y-a.y)+radius*std::abs(angle(b.yaw-a.yaw));
  const int n=std::max(1,int(std::ceil(motion/(.45*grid.resolution))));
  double maximum=0;
  for(int i=1;i<=n;++i) {
    const double t=double(i)/n;
    State s=a;s.x+=t*(b.x-a.x);s.y+=t*(b.y-a.y);s.yaw=angle(a.yaw+t*angle(b.yaw-a.yaw));
    double c=0;if(!grid.footprint(s,polygon,&c)) return false;maximum=std::max(maximum,c);
  }
  if(soft) *soft=maximum;
  return true;
}
struct Evaluation { double score=1e100;std::vector<State> path;Control first; double omega=0;bool valid=false; };
double approach_speed(double distance,const Parameters& p) {
  // v * delay + v^2/(2*a) fits in the remaining distance. Aim inside, rather
  // than at the outer edge of, the existing positional goal tolerance.
  const double remaining=std::max(0.,distance-std::min(.2,.4*p.xy_tolerance));
  const double ad=p.deceleration*p.stopping_delay;
  return std::max(0.,std::sqrt(ad*ad+2*p.deceleration*remaining)-ad);
}
Control approach(Control u,const State& s,const State& goal,double remaining,const Parameters& p) {
  // Heading-infeasible parking approaches still need room to manoeuvre.
  // Exact coverage orientations are not silently relaxed to accept a stop.
  if(remaining>2*p.xy_tolerance || !p.goal_yaw_required ||
     std::abs(angle(s.yaw-goal.yaw))<=std::max(.6,2*p.yaw_tolerance)) {
    const double limit=approach_speed(remaining,p);
    u.v=clamp(u.v,-limit,limit);
  }
  return u;
}
Evaluation evaluate(const std::vector<double>& u,const State& start,
                    const std::vector<State>& ref,const std::vector<double>& arc,
                    const Grid& grid,const std::vector<Point>& polygon,const Parameters& p,
                    bool terminal) {
  Evaluation out;State s=start;double score=0,t=0,radius=0;
  for(auto q:polygon) radius=std::max(radius,std::hypot(q.x,q.y));
  size_t index=0;out.path.push_back(s);double last_progress=0;
  while(t<p.horizon-1e-9) {
    const double dt=std::min(t==0?p.period:.2,p.horizon-t);
    Control control=sampled(u,t,p);
    if(terminal) {
      const auto match=nearest(s,ref,arc,index);
      const double remaining=std::max(arc.back()-match.arc,std::hypot(s.x-ref.back().x,s.y-ref.back().y));
      control=approach(control,s,ref.back(),remaining,p);
    }
    // The speed setpoint and the predicted physical speed are different while
    // braking. Applying the weak physical brake rate to the setpoint as well
    // creates a feedback staircase and substantially lengthens the real stop.
    auto command_limits=p;command_limits.deceleration=p.acceleration;
    const State command=Optimizer::advance(s,control,dt,command_limits);
    const Control setpoint{command.v,std::abs(command.v)>1e-9?command.omega/command.v:0};
    State next=Optimizer::advance(s,setpoint,dt,p); double obstacle=0;
    if(!sweep(s,next,grid,polygon,radius,&obstacle)) {
      // Infeasible samples can guide the distribution towards longer feasible
      // prefixes, but are NEVER eligible for an output command.
      out.score=1e6+1000.*(p.horizon-t);return out;
    }
    if(t==0) {out.first=setpoint;out.omega=command.omega;}
    const auto match=nearest(next,ref,arc,index);index=match.index;last_progress=match.arc;
    double heading=angle(next.yaw-match.yaw);
    if(p.overwrite_orientation && next.v<0) heading=angle(heading+M_PI);
    score+=dt*(p.path_weight*sq(match.distance)+p.heading_weight*sq(heading)+
               p.obstacle_weight*sq(obstacle/252.)+.04*sq(next.omega)+.02*sq(next.v-s.v)+
               p.steering_weight*sq((next.steering-wheel_angle(s,p))/dt));
    s=next;t+=dt;out.path.push_back(s);
  }
  score-=p.progress_weight*last_progress;
  if(terminal) {
    const auto& goal=ref.back();double distance=std::hypot(s.x-goal.x,s.y-goal.y);
    score+=7.*sq(distance)+(p.goal_yaw_required?3.*sq(angle(s.yaw-goal.yaw)):0.);
    score+=8.*sq(s.v);
  }
  // Check the full stopping continuation, not just the finite prediction horizon.
  for(double delay=0;delay<p.stopping_delay-1e-9 && std::abs(s.v)>1e-6;) {
    const double dt=std::min(.1,p.stopping_delay-delay);
    State next=s;
    next.x+=s.v*std::cos(s.yaw+.5*s.omega*dt)*dt;
    next.y+=s.v*std::sin(s.yaw+.5*s.omega*dt)*dt;
    next.yaw=angle(s.yaw+s.omega*dt);
    if(!sweep(s,next,grid,polygon,radius,nullptr)) {out.score=1e6+std::abs(s.v)*1000.;return out;}
    s=next;delay+=dt;
  }
  while(std::abs(s.v)>1e-6) {
    const double dt=std::min(.1,std::abs(s.v)/p.deceleration);
    Control stop{0,std::abs(s.v)>1e-8?s.omega/s.v:0};
    State next=Optimizer::advance(s,stop,dt,p);
    if(!sweep(s,next,grid,polygon,radius,nullptr)) {out.score=1e6+std::abs(s.v)*1000.;return out;}
    s=next;
  }
  out.valid=true;out.score=score;return out;
}
std::vector<double> seed(const State& start,const std::vector<State>& path,
                         const std::vector<double>& arc,const Parameters& p,int gear,bool terminal) {
  std::vector<double> u(2*p.knots);State s=start;size_t index=0;
  for(int k=0;k<p.knots;++k) {
    const auto m=nearest(s,path,arc,index);index=m.index;
    const double look=.7+std::abs(s.v)*.5;
    size_t j=index;while(j+1<path.size() && arc[j]<m.arc+look) ++j;
    double dx=path[j].x-s.x,dy=path[j].y-s.y;
    const double lateral=-std::sin(s.yaw)*dx+std::cos(s.yaw)*dy;
    double curvature=clamp(2*lateral/std::max(.15,sq(dx)+sq(dy)),-1/p.radius,1/p.radius);
    double speed=gear>0?p.forward:-p.reverse;
    if(terminal) {
      const double remaining=std::max(arc.back()-m.arc,std::hypot(s.x-path.back().x,s.y-path.back().y));
      speed=approach({speed,curvature},s,path.back(),remaining,p).v;
    }
    u[2*k]=speed;u[2*k+1]=curvature;
    s=Optimizer::advance(s,{speed,curvature},p.horizon/(p.knots-1),p);
  }
  return u;
}
} // namespace
double angle(double x) { return std::atan2(std::sin(x),std::cos(x)); }
void Grid::prepare() {
  integral.assign((width+1)*(height+1),0);
  for(unsigned y=0;y<height;++y) {
    unsigned row=0;
    for(unsigned x=0;x<width;++x) {
      row+=blocked(x,y);integral[(y+1)*(width+1)+x+1]=integral[y*(width+1)+x+1]+row;
    }
  }
}
bool Grid::blocked(int x,int y) const {
  if(x<0||y<0||x>=int(width)||y>=int(height)) return true;
  unsigned c=cells[y*width+x];return c==254 || (c==255 && unknown_obstacle);
}
bool Grid::footprint(const State& s,const std::vector<Point>& polygon,double* cost) const {
  if(polygon.size()<3 || integral.empty() || !std::isfinite(s.x+s.y+s.yaw)) return false;
  std::vector<Point> world;world.reserve(polygon.size());
  double xmin=1e100,xmax=-1e100,ymin=1e100,ymax=-1e100;
  for(auto p:polygon) {
    Point q{s.x+std::cos(s.yaw)*p.x-std::sin(s.yaw)*p.y,
            s.y+std::sin(s.yaw)*p.x+std::cos(s.yaw)*p.y};world.push_back(q);
    xmin=std::min(xmin,q.x);xmax=std::max(xmax,q.x);ymin=std::min(ymin,q.y);ymax=std::max(ymax,q.y);
  }
  const int x0=int(std::floor((xmin-origin_x)/resolution)),x1=int(std::floor((xmax-origin_x)/resolution));
  const int y0=int(std::floor((ymin-origin_y)/resolution)),y1=int(std::floor((ymax-origin_y)/resolution));
  if(x0<0||y0<0||x1>=int(width)||y1>=int(height)) return false;
  const unsigned stride=width+1;
  const auto sum=integral[(y1+1)*stride+x1+1]+integral[y0*stride+x0]-
                 integral[y0*stride+x1+1]-integral[(y1+1)*stride+x0];
  if(sum) for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x)
    if(blocked(x,y) && overlap(world,origin_x+(x+.5)*resolution,origin_y+(y+.5)*resolution,resolution*.5)) return false;
  if(cost) {
    *cost=0;
    world.push_back({s.x,s.y});
    for(auto p:world) {
      const int x=int((p.x-origin_x)/resolution),y=int((p.y-origin_y)/resolution);
      if(x>=0&&y>=0&&x<int(width)&&y<int(height)) *cost=std::max(*cost,double(std::min<int>(cells[y*width+x],252)));
    }
  }
  return true;
}
State Optimizer::advance(const State& s,Control u,double dt,const Parameters& p) {
  State n=s;
  const double lo=(p.allow_reverse && p.direction<=0)?-p.reverse:0;
  const double hi=p.direction>=0?p.forward:0;
  // A gear change first reaches zero. Never jump across zero in one command.
  u.v=clamp(u.v,lo,hi);if(s.v*u.v<0) u.v=0;
  const double acceleration=std::abs(u.v)<std::abs(s.v)?std::min(p.acceleration,p.deceleration):p.acceleration;
  n.v=clamp(u.v,s.v-acceleration*dt,s.v+acceleration*dt);
  // Brake along a curve slowly enough to respect BOTH yaw acceleration and
  // minimum radius. Clipping yaw after aggressive linear braking violates the
  // yaw acceleration bound and is especially visible at a reversing cusp.
  const double minimum_speed=p.radius*std::max(0.,std::abs(s.omega)-p.angular_acceleration*dt);
  if(std::abs(n.v)<minimum_speed && std::abs(s.v)>1e-9)
    n.v=std::copysign(std::min(std::abs(s.v),minimum_speed),s.v);
  double limit=std::min(p.angular,std::abs(n.v)/p.radius);
  const double maximum_angle=std::atan(p.wheelbase/p.radius);
  const double previous_angle=wheel_angle(s,p);
  const double angle_lo=std::max(-maximum_angle,previous_angle-p.steering_rate*dt);
  const double angle_hi=std::min(maximum_angle,previous_angle+p.steering_rate*dt);
  const double omega_lo=std::max(-p.angular,s.omega-p.angular_acceleration*dt);
  const double omega_hi=std::min(p.angular,s.omega+p.angular_acceleration*dt);
  auto curvature_interval=[&](double velocity,double& low,double& high) {
    if(angle_lo>angle_hi) return false;
    low=std::tan(angle_lo)/p.wheelbase;high=std::tan(angle_hi)/p.wheelbase;
    if(std::abs(velocity)<1e-9) return omega_lo<=0 && omega_hi>=0;
    low=std::max(low,std::min(omega_lo/velocity,omega_hi/velocity));
    high=std::min(high,std::max(omega_lo/velocity,omega_hi/velocity));
    return low<=high;
  };
  double low=0,high=0;
  bool feasible=curvature_interval(n.v,low,high);
  if(!feasible && curvature_interval(s.v,low,high)) {
    // When accelerating/braking on a curve, preserve wheel-rate AND yaw-rate
    // bounds by reducing the velocity change, rather than snapping the wheels.
    double valid=s.v,invalid=n.v;
    for(int i=0;i<40;++i) {
      const double mid=.5*(valid+invalid);
      if(curvature_interval(mid,low,high)) valid=mid;else invalid=mid;
    }
    n.v=valid;feasible=curvature_interval(n.v,low,high);
  }
  if(feasible) {
    if(std::abs(n.v)<1e-9) {
      n.omega=0;n.steering=previous_angle;
    } else {
      const double curvature=clamp(u.curvature,low,high);
      n.omega=n.v*curvature;n.steering=std::atan(p.wheelbase*curvature);
    }
  } else {
    // An already inconsistent measured state can lie outside the configured
    // envelope. Retain geometric limits, as before, while returning to it.
    n.omega=clamp(n.v*clamp(u.curvature,-1/p.radius,1/p.radius),-limit,limit);
    n.omega=clamp(n.omega,s.omega-p.angular_acceleration*dt,s.omega+p.angular_acceleration*dt);
    n.omega=clamp(n.omega,-limit,limit);
    n.steering=std::abs(n.v)>1e-9?std::atan(p.wheelbase*n.omega/n.v):previous_angle;
  }
  const double yaw_mid=s.yaw+.5*n.omega*dt;
  n.x+=n.v*std::cos(yaw_mid)*dt;n.y+=n.v*std::sin(yaw_mid)*dt;n.yaw=angle(s.yaw+n.omega*dt);
  return n;
}
void Optimizer::reset() { warm_.clear(); }
Result Optimizer::solve(const State& start,const std::vector<State>& ref,const Grid& grid,
                        const std::vector<Point>& polygon,const Parameters& p,bool terminal) {
  const auto begin=std::chrono::steady_clock::now();Result result;
  if(ref.empty() || p.knots<2 || p.samples<4 || p.iterations<1 || p.radius<=0 || p.acceleration<=0 ||
     p.period<=0 || p.horizon<p.period || p.wheelbase<=0 || p.steering_rate<=0 ||
     p.deceleration<=0 || p.stopping_delay<0 || !grid.footprint(start,polygon)) {
    result.reason="invalid input or current footprint in collision";return result;
  }
  std::vector<double> arc(ref.size(),0);
  for(size_t i=1;i<ref.size();++i) arc[i]=arc[i-1]+std::hypot(ref[i].x-ref[i-1].x,ref[i].y-ref[i-1].y);
  Evaluation best;std::vector<double> best_u;
  auto consider=[&](const std::vector<double>& u) {
    auto e=evaluate(u,start,ref,arc,grid,polygon,p,terminal);++result.evaluations;
    if(e.valid && e.score<best.score) {best=e;best_u=u;}return e.score;
  };
  std::vector<int> gears;
  if(p.direction>=0) gears.push_back(1);
  if(p.direction<=0 && p.allow_reverse && p.reverse>0) gears.push_back(-1);
  std::normal_distribution<double> normal(0,1);
  for(int gear:gears) {
    auto mean=seed(start,ref,arc,p,gear,terminal);
    double initial=consider(mean);
    // Distinct local detour seeds keep a straight reference from trapping the
    // distribution at a symmetric obstacle. The global path itself is untouched.
    for(double side:{-1.,1.}) {
      auto detour=mean;
      for(int k=0;k<p.knots;++k) {
        const double f=double(k)/(p.knots-1);
        detour[2*k+1]=side*(f<.5?1.:-1.)/p.radius;
      }
      double value=consider(detour);if(value<initial) {initial=value;mean=detour;}
    }
    // A single full-speed seed is a poor starting distribution when the M2
    // needs a long braking distance. Include coherent slower approaches;
    // random independent knot noise rarely creates one over the whole horizon.
    for(double scale:{.5,.25}) {
      auto limits=p;limits.forward*=scale;limits.reverse*=scale;
      auto slow=seed(start,ref,arc,limits,gear,terminal);
      double value=consider(slow);if(value<initial) {initial=value;mean=slow;}
      for(double side:{-1.,1.}) {
        auto detour=slow;
        for(int k=0;k<p.knots;++k)
          detour[2*k+1]=side*(double(k)/(p.knots-1)<.5?1.:-1.)/p.radius;
        value=consider(detour);if(value<initial) {initial=value;mean=detour;}
      }
    }
    if(warm_.size()==mean.size()) {
      std::vector<double> shifted(mean.size());
      for(int k=0;k<p.knots;++k) {auto u=sampled(warm_,k*p.horizon/(p.knots-1)+p.period,p);shifted[2*k]=u.v;shifted[2*k+1]=u.curvature;}
      if(consider(shifted)<initial) mean=shifted;
    }
    std::vector<double> variance(mean.size());
    for(size_t d=0;d<mean.size();++d) variance[d]=(d%2)?sq(.38/p.radius):sq(gear>0?.38:.22);
    for(int it=0;it<p.iterations;++it) {
      std::vector<std::vector<double>> population(p.samples,mean);
      std::vector<double> scores(p.samples);std::vector<int> order(p.samples);
      for(int i=0;i<p.samples;++i) {
        // Independent Gaussian coordinates. Do not clamp samples before the
        // natural-gradient update: the bounded rollout is the objective map.
        for(size_t d=0;d<mean.size();++d) population[i][d]+=std::sqrt(variance[d])*normal(rng_);
        scores[i]=consider(population[i]);order[i]=i;
      }
      std::stable_sort(order.begin(),order.end(),[&](int a,int b){return scores[a]<scores[b];});
      const int elite=std::max(1,int(p.samples*p.elite_fraction));
      if(scores[order[0]]>=1e99) break;
      // Gaussian IGO Euler step in (mean, diagonal variance) coordinates.
      // Rank utility is uniform on the best quantile, normalized to sum 1.
      // Tied ranks receive their mean utility, preventing arbitrary movement
      // on a flat objective (including equally infeasible trajectories).
      std::vector<double> weights(p.samples,0);
      for(int a=0;a<p.samples;) {
        int b=a+1;while(b<p.samples && scores[order[b]]==scores[order[a]]) ++b;
        double w=double(std::max(0,std::min(b,elite)-a))/(elite*(b-a));
        for(int j=a;j<b;++j) weights[order[j]]=w;
        a=b;
      }
      for(size_t d=0;d<mean.size();++d) {
        double dm=0,dv=0;
        for(int i=0;i<p.samples;++i) {double delta=population[i][d]-mean[d];dm+=weights[i]*delta;dv+=weights[i]*(delta*delta-variance[d]);}
        mean[d]+=p.mean_rate*dm;
        variance[d]=clamp(variance[d]+p.variance_rate*dv,sq(d%2?.025:.025),sq(d%2?1/p.radius:p.forward+p.reverse));
      }
      consider(mean);
    }
  }
  // A stop trajectory is a candidate in the same optimizer, never an extra
  // external command gate. If no feasible control exists, report failure to move_base.
  std::vector<double> stop(2*p.knots,0);consider(stop);
  // Braking along the measured wheel angle can remain feasible beside an
  // obstacle even when straightening the wheels during braking would collide.
  const double held_curvature=std::tan(wheel_angle(start,p))/p.wheelbase;
  for(int k=0;k<p.knots;++k) stop[2*k+1]=held_curvature;
  consider(stop);
  for(double curvature:{-1/p.radius,1/p.radius}) {
    for(int k=0;k<p.knots;++k) stop[2*k+1]=curvature;
    consider(stop);
  }
  result.valid=best.valid;
  if(best.valid) {result.command=best.first;result.omega=best.omega;result.cost=best.score;result.trajectory=best.path;warm_=best_u;}
  else {reset();result.reason="no collision-free Ackermann rollout including stopping tail";}
  result.elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  return result;
}
} // namespace igo_local_planner
