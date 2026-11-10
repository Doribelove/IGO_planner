#include "igo_local_planner/core.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <deque>
#include <stdexcept>
using namespace igo_local_planner;
namespace {
std::string output;
unsigned optimizer_seed=17;
void require(bool value,const char* message) {if(!value) throw std::runtime_error(message);}
std::vector<Point> footprint{{1.32,.5},{1.32,-.5},{-.62,-.5},{-.62,.5}};
Grid grid() {Grid g;g.width=240;g.height=240;g.origin_x=-12;g.origin_y=-12;g.cells.assign(g.width*g.height,0);g.prepare();return g;}
void obstacle(Grid& g,double x0,double y0,double x1,double y1,unsigned char value=254) {
  for(unsigned y=0;y<g.height;++y) for(unsigned x=0;x<g.width;++x) {
    double wx=g.origin_x+(x+.5)*g.resolution,wy=g.origin_y+(y+.5)*g.resolution;
    if(wx>=x0&&wx<=x1&&wy>=y0&&wy<=y1) g.cells[y*g.width+x]=value;
  }g.prepare();
}
std::vector<State> line(double length,int gear=1) {
  std::vector<State> path;
  for(double t=0;t<=length+.0001;t+=.1) path.push_back({gear*t,0,0,0,0});
  return path;
}
void closed_loop(const char* name,const std::vector<State>& path,Parameters p,Grid g,int gear,int steps=400) {
  Optimizer optimizer(optimizer_seed);State s=path.front();size_t nearest=0;std::vector<double> times;
  std::ofstream trace,reference,world;
  if(!output.empty()) {
    trace.open(output+"/"+name+"_trace.csv");trace<<"time,x,y,yaw,v,omega\n0,"<<s.x<<","<<s.y<<","<<s.yaw<<",0,0\n";
    reference.open(output+"/"+name+"_reference.csv");reference<<"x,y,yaw\n";
    for(auto q:path) reference<<q.x<<","<<q.y<<","<<q.yaw<<"\n";
    world.open(output+"/"+name+"_obstacles.csv");world<<"x,y,cost\n";
    for(unsigned y=0;y<g.height;++y) for(unsigned x=0;x<g.width;++x)
      if(g.cells[y*g.width+x]) world<<g.origin_x+(x+.5)*g.resolution<<","<<g.origin_y+(y+.5)*g.resolution<<","<<int(g.cells[y*g.width+x])<<"\n";
  }
  bool reached=false;double peak_cross=0,distance=0;int forward=0,reverse=0;
  for(int k=0;k<steps;++k) {
    double best=1e100;size_t end=std::min(path.size(),nearest+30);
    for(size_t j=nearest;j<end;++j) {double d=std::hypot(s.x-path[j].x,s.y-path[j].y);if(d<best){best=d;nearest=j;}}
    peak_cross=std::max(peak_cross,best);
    end=nearest+1;double arc=0;
    while(end<path.size()&&arc<p.lookahead) {arc+=std::hypot(path[end].x-path[end-1].x,path[end].y-path[end-1].y);++end;}
    std::vector<State> ref(path.begin()+nearest,path.begin()+end);
    auto r=optimizer.solve(s,ref,g,footprint,p,end==path.size());
    require(r.valid,"closed-loop solver has no feasible trajectory");times.push_back(r.elapsed_ms);
    require(std::abs(r.omega)<=std::abs(r.command.v)/p.radius+1e-8,"turn radius violated");
    require(r.command.v<=p.forward+1e-8&&r.command.v>=-p.reverse-1e-8,"speed bound violated");
    require(std::abs(r.command.v-s.v)<=p.acceleration*p.period+1e-8,"acceleration violated");
    require(std::abs(r.omega-s.omega)<=p.angular_acceleration*p.period+1e-8,"yaw acceleration violated");
    const double current_steering=std::isfinite(s.steering)?s.steering:
      (std::abs(s.v)>1e-8?std::atan(p.wheelbase*s.omega/s.v):0.);
    if(std::abs(r.command.v)>1e-8)
      require(std::abs(std::atan(p.wheelbase*r.command.curvature)-current_steering)
              <=p.steering_rate*p.period+1e-8,"front wheel steering rate violated");
    if(gear) require(r.command.v*gear>=-1e-8,"fixed-gear violation");
    if(r.command.v>.01) ++forward;
    if(r.command.v<-.01) ++reverse;
    State next=Optimizer::advance(s,r.command,p.period,p);
    require(g.footprint(next,footprint),"simulated footprint collision");
    distance+=std::hypot(next.x-s.x,next.y-s.y);s=next;
    if(trace) trace<<(k+1)*p.period<<","<<s.x<<","<<s.y<<","<<s.yaw<<","<<s.v<<","<<s.omega<<"\n";
    if(std::hypot(s.x-path.back().x,s.y-path.back().y)<.25 && std::abs(angle(s.yaw-path.back().yaw))<.25) {reached=true;break;}
  }
  std::sort(times.begin(),times.end());
  std::cout<<"{\"case\":\""<<name<<"\",\"reached\":"<<(reached?"true":"false")
    <<",\"steps\":"<<times.size()<<",\"x\":"<<s.x<<",\"y\":"<<s.y<<",\"yaw\":"<<s.yaw
    <<",\"distance\":"<<distance<<",\"peak_cross_track\":"<<peak_cross
    <<",\"forward_commands\":"<<forward<<",\"reverse_commands\":"<<reverse
    <<",\"p50_ms\":"<<times[times.size()/2]<<",\"p95_ms\":"<<times[std::min(times.size()-1,size_t(times.size()*.95))]
    <<",\"max_ms\":"<<times.back()<<"}"<<std::endl;
  require(reached,"closed-loop goal was not reached");
}
void delayed_stop(int gear,Parameters p) {
  // Different plant and controller dynamics: delayed commands and weaker
  // acceleration. A perfect instantaneous kinematic plant hides overshoot.
  Parameters plant;p.direction=gear;p.overwrite_orientation=false;
  plant.acceleration=.8;plant.deceleration=.5;plant.steering_rate=.45;
  Grid g=grid();auto path=line(2.5,gear);Optimizer optimizer(optimizer_seed);State s;
  std::deque<Control> delayed(4,Control{});
  bool braking=false,settled=false;double overshoot=0;int wrong_gear=0;
  for(int k=0;k<300;++k) {
    const double distance=std::hypot(s.x-path.back().x,s.y-path.back().y);
    if(distance<=p.xy_tolerance && std::abs(s.yaw)<=p.yaw_tolerance) braking=true;
    auto limits=p;if(braking) {limits.forward=0;limits.reverse=0;}
    auto r=optimizer.solve(s,path,g,footprint,limits,true);
    require(r.valid,"delayed stopping trajectory infeasible");
    if(r.command.v*gear<-.01) ++wrong_gear;
    delayed.push_back(r.command);auto command=delayed.front();delayed.pop_front();
    s=Optimizer::advance(s,command,p.period,plant);
    overshoot=std::max(overshoot,gear*s.x-2.5);
    if(braking && k>10 && std::abs(s.v)<.01 &&
       std::all_of(delayed.begin(),delayed.end(),[](Control u){return std::abs(u.v)<.01;})) {
      settled=std::hypot(s.x-path.back().x,s.y-path.back().y)<=p.xy_tolerance;
      break;
    }
  }
  std::cout<<"{\"case\":\"delayed_stop_"<<gear<<"\",\"settled\":"<<settled
    <<",\"x\":"<<s.x<<",\"overshoot\":"<<overshoot<<",\"wrong_gear\":"<<wrong_gear<<"}"<<std::endl;
  require(settled && overshoot<=p.xy_tolerance && wrong_gear==0,
          "delayed plant overshot goal or reversed during stop");
}
}
int main(int argc,char** argv) {
  try {
    if(argc>1) output=argv[1];
    Parameters p;Grid g=grid();Optimizer optimizer;
    if(argc>3) {p.deceleration=std::stod(argv[2]);p.stopping_delay=std::stod(argv[3]);}
    if(argc>4) optimizer_seed=static_cast<unsigned>(std::stoul(argv[4]));
    // A point inside the large front attachment must collide even when all
    // polygon vertices and the vehicle centre are free.
    obstacle(g,.85,.15,.95,.25);require(!g.footprint({},footprint),"interior obstacle missed");
    g=grid();obstacle(g,.85,.15,.95,.25,255);require(!g.footprint({},footprint),"unknown interior missed");
    g.unknown_obstacle=false;g.prepare();require(g.footprint({},footprint),"unknown policy ignored");
    g=grid();obstacle(g,1.4,-2,1.5,2);auto blocked=optimizer.solve({},line(4),g,footprint,p,true);
    require(blocked.valid&&blocked.command.v<=.1,"blocked path must brake without crossing the wall");
    State moving;moving.v=.8;g=grid();obstacle(g,1.4,-2,1.5,2);
    auto impossible=optimizer.solve(moving,line(4),g,footprint,p,true);
    require(!impossible.valid,"unavoidable stopping collision must report failure");
    State zero;zero=Optimizer::advance(zero,{0,100},.1,p);require(zero.omega==0,"in-place spin generated");
    State stopped;stopped.steering=.3;
    auto hold=Optimizer::advance(stopped,{0,-1/p.radius},p.period,p);
    require(hold.steering==stopped.steering,"zero Twist changed held wheel angle");
    auto launch=Optimizer::advance(stopped,{.8,-1/p.radius},p.period,p);
    require(std::abs(launch.steering-stopped.steering)<=p.steering_rate*p.period+1e-8,
            "launch ignored measured wheel angle while stationary");
    State alternate;alternate.steering=.3;
    for(int i=0;i<200;++i) {
      auto next=Optimizer::advance(alternate,{i%40<20?.8:-.3,i%2?1/p.radius:-1/p.radius},p.period,p);
      require(std::abs(next.steering-alternate.steering)<=p.steering_rate*p.period+1e-8,"alternating wheel command rate");
      require(std::abs(next.v-alternate.v)<=p.acceleration*p.period+1e-8,"alternating linear acceleration");
      require(std::abs(next.omega-alternate.omega)<=p.angular_acceleration*p.period+1e-8,"alternating yaw acceleration");
      require(alternate.v*next.v>=0,"alternating direction skipped zero");
      alternate=next;
    }
    State forward;forward.v=.02;auto cusp=Optimizer::advance(forward,{-.3,0},.1,p);
    require(cusp.v>=0 && cusp.v<forward.v,"direction switch crossed zero before stopping");
    for(int i=0;i<20 && cusp.v>0;++i) cusp=Optimizer::advance(cusp,{-.3,0},.1,p);
    require(cusp.v==0,"direction switch failed to reach zero speed");
    State braking;braking.v=.6;braking.omega=.6/p.radius;
    auto braked=Optimizer::advance(braking,{0,0},.1,p);
    require(std::abs(braked.omega-braking.omega)<=p.angular_acceleration*.1+1e-8,"curved braking yaw acceleration");
    require(std::abs(braked.omega)<=std::abs(braked.v)/p.radius+1e-8,"curved braking radius");
    delayed_stop(1,p);delayed_stop(-1,p);
    p.direction=1;p.overwrite_orientation=false;
    closed_loop("straight_forward",line(5),p,grid(),1);
    p.direction=-1;closed_loop("straight_reverse",line(2,-1),p,grid(),-1);
    std::vector<State> arc;
    for(double a=0;a<=1.2+.00001;a+=.04) arc.push_back({2*std::sin(a),2*(1-std::cos(a)),a,0,0});
    p.direction=1;closed_loop("forward_radius_2m",arc,p,grid(),1);
    for(auto& s:arc) {s.x=-s.x;s.y=-s.y;}
    p.direction=-1;closed_loop("reverse_radius_2m",arc,p,grid(),-1);
    p.direction=0;closed_loop("automatic_reverse",line(2,-1),p,grid(),0);
    g=grid();
    for(unsigned y=0;y<g.height;++y) for(unsigned x=0;x<g.width;++x) {
      const double dx=std::max(0.,std::abs(g.origin_x+(x+.5)*g.resolution-3.7)-.2);
      const double dy=std::max(0.,std::abs(g.origin_y+(y+.5)*g.resolution)-.2);
      const double d=std::hypot(dx,dy);
      if(d<1.) g.cells[y*g.width+x]=d==0?254:static_cast<unsigned char>(220*std::exp(-3*d));
    }g.prepare();p.direction=0;
    closed_loop("local_obstacle_detour",line(7),p,g,0,600);
    std::cout<<"{\"tests\":\"passed\",\"hardware_motion\":false}"<<std::endl;
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<std::endl;return 1;}
}
