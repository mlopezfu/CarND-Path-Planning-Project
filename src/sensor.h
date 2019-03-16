#include <iostream>
//#include "json.hpp"


class Sensor {
public:
  
  static double delta_s;
  static double max_speed;
  static double front_reference_speed;
  static bool try_to_change;
  static int lane;
  
  static int best_lane; // We will try to find the best lane, and make it our objective
  //static int best_lane_sr; // We will try to find the best lane, and make it our objective
  static double ref_vel;
  double s;
  void setNewData(std::string jsonData);

  /**
  * Constructors
  */
  Sensor();
  //Sensor(std::string jsonData);
  /**
  * Destructor
  */
  //virtual ~Sensor();
private:
  bool debug;
  int carLane(float d);
  static double prior_distance_ahead;
};
  