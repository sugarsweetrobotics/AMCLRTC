#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include "NavigationDataType.hh"

#include "amcl/map/map.h"
#include "amcl/pf/pf.h"
#include "amcl/sensors/amcl_odom.h"
#include "amcl/sensors/amcl_laser.h"

/// PF

struct pf_config {
  int min_particles_;
  int max_particles_;
  double alpha_slow_;
  double alpha_fast_;
  pf_init_model_fn_t init_model_;
  bool selective_resampling_;
  double pf_err_;
  double pf_z_;
  double init_pose_[3];
  double init_cov_[3];
  double d_thresh_, a_thresh_;
};


inline pf_config pf_config_init() {
  pf_config config;
  config.init_pose_[0] = 0.0;
  config.init_pose_[1] = 0.0;
  config.init_pose_[2] = 0.0;
  config.init_cov_[0] = 0.5 * 0.5;
  config.init_cov_[1] = 0.5 * 0.5;
  config.init_cov_[2] = (M_PI/12.0) * (M_PI/12.0);
  return config;
};


/// Map

map_t* convertMap(const NAVIGATION::OccupancyGridMap& ogmap);


/// Laser

struct beam_model_config {

};

struct likelihood_field_model_config {
  //  double z_hit_, z_rand_, sigma_hit_;
  bool do_beamskip_;
  double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
  double max_dist_;
};

struct laser_config {
  int max_beams_;
  amcl::laser_model_t model_type_;
  std::string model_type_str_;
  double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;  
  beam_model_config beam_model_;
  likelihood_field_model_config likelihood_model_;
  double max_range_;
  double min_range_;  
};


struct odom_config {
  amcl::odom_model_t model_type_;
  std::string model_type_str_;
  double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
};


amcl::AMCLLaser* initLaser(const laser_config& config, map_t* map);

amcl::AMCLOdom* initOdom(const odom_config& config);

pf_t* initPF(const pf_config& config, map_t* map);

amcl::AMCLLaserData* convertLaser(amcl::AMCLLaser* laser, const RTC::RangeData& range, const laser_config& config);
