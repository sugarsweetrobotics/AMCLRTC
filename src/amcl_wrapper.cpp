
#define _USE_MATH_DEFINED
#include <math.h>

#ifdef WIN32
#include "portable_utils.hpp"
#endif

#include <iostream>
#include <fstream>

#include "amcl_wrapper.h"


#if WIN32
#ifdef min

#define MIN(x, y) ((x > y) ? (y) : (x))
#define MAX(x, y) ((x > y) ? (x) : (y))

#else

#define MIN(x, y) std::min((x), (y))
#define MAX(x, y) std::max((x), (y))


#endif

#else // WIN32

#define MIN(x, y) std::min((x), (y))
#define MAX(x, y) std::max((x), (y))

#endif // WIN32


#define _DEBUG

map_t* convertMap(const NAVIGATION::OccupancyGridMap& ogmap) {
#ifdef _DEBUG
  std::cout << "convertMap() called" << std::endl;
#endif
  map_t* map = map_alloc();
  if (!map) {
    return nullptr;
  }
  map->size_x = ogmap.config.sizeOfMap.w / ogmap.config.sizeOfGrid.w;
  map->size_y = ogmap.config.sizeOfMap.l / ogmap.config.sizeOfGrid.l;
  map->scale = ogmap.config.sizeOfGrid.w;
  map->origin_x = ogmap.config.globalPositionOfTopLeft.position.x + map->size_x / 2 * map->scale;
  map->origin_y = -ogmap.config.globalPositionOfTopLeft.position.y + map->size_y / 2 * map->scale; 

#ifdef _DEBUG
  std::cout << "ogmap :" << std::endl;
  std::cout << "  globalPositionOfTopLeft.position.x : " << ogmap.config.globalPositionOfTopLeft.position.x << std::endl;
  std::cout << "  globalPositionOfTopLeft.position.y : " << ogmap.config.globalPositionOfTopLeft.position.y << std::endl;  
  std::cout << "map_t :" << std::endl;
  std::cout << "  size_x   : " << map->size_x << std::endl;
  std::cout << "  size_y   : " << map->size_y << std::endl;
  std::cout << "  scale    : " << map->scale << std::endl;
  std::cout << "  origin_x : " << map->origin_x << std::endl;
  std::cout << "  origin_y : " << map->origin_y << std::endl;
#endif
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  const int size = map->size_x*map->size_y;

  //  std::ofstream fout("output.csv");
  for (int i = 0;i < map->size_y;i++) {
    for(int j = 0;j < map->size_x;j++) {
      auto ogmapIndex = i * map->size_x + j;
      auto mapIndex = i * map->size_x + j;
      if(ogmap.cells[ogmapIndex] >= 200) {
	map->cells[mapIndex].occ_state = -1; // Free
      } else if(ogmap.cells[ogmapIndex] <= 100) {
	map->cells[mapIndex].occ_state = +1; // Occupied
      } else {
	map->cells[mapIndex].occ_state = 0;  // Unknown
      }
      //fout << (int)ogmap.cells[ogmapIndex] << ",";
    }
    //    fout << std::endl;
  }
  return map;
}


amcl::AMCLLaser* initLaser(const laser_config& config, map_t* map) {
#ifdef _DEBUG
  std::cout << "initLaser() called" << std::endl;
  std::cout << "laser_config: " << std::endl;
  std::cout << "  model_type_str_ : " << config.model_type_str_.c_str() << std::endl;  
  std::cout << "  max_beams_      : " << config.max_beams_ << std::endl;
  std::cout << "  max_range_      : " << config.max_range_ << std::endl;
  std::cout << "  min_range_      : " << config.min_range_ << std::endl;
  std::cout << "  z_hit_          : " << config.z_hit_ << std::endl;
  std::cout << "  z_short_        : " << config.z_short_ << std::endl;
  std::cout << "  z_max_          : " << config.z_max_ << std::endl;
  std::cout << "  z_rand_         : " << config.z_rand_ << std::endl;
  std::cout << "  z_sigma_hit_    : " << config.sigma_hit_ << std::endl;
  std::cout << "  z_lambda_short_ : " << config.lambda_short_ << std::endl;
  std::cout << "  beam_model_     : " << "{}" << std::endl;
  std::cout << "  likelihood_model_ :" << std::endl;
  std::cout << "    max_dist_            : " << config.likelihood_model_.max_dist_ << std::endl;
  std::cout << "    do_beamskip_         : " << config.likelihood_model_.do_beamskip_ << std::endl;
  std::cout << "    beam_skip_distance_  : " << config.likelihood_model_.beam_skip_distance_ << std::endl;
  std::cout << "    beam_skip_threshold_ : " << config.likelihood_model_.beam_skip_threshold_ << std::endl;
  std::cout << "    beam_skip_error_threshold_ : " << config.likelihood_model_.beam_skip_error_threshold_ << std::endl;    
#endif
  auto model_type_ = amcl::LASER_MODEL_LIKELIHOOD_FIELD;
  if (config.model_type_str_ == "likelihood_field") {
    model_type_ = amcl::LASER_MODEL_LIKELIHOOD_FIELD;
  } else if (config.model_type_str_ == "beam") {
    model_type_ = amcl::LASER_MODEL_BEAM;
  } else if (config.model_type_str_ == "likelihood_field_prob") {
    model_type_ = amcl::LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  } else {
    std::cout << "amcl_wrapper: initLaser failed. Unknown laser_model_type field. (" << config.model_type_str_.c_str() << std::endl;
    return nullptr;
  }

  amcl::AMCLLaser* laser = new amcl::AMCLLaser(config.max_beams_, map);
  if (!laser) {
    return nullptr;
  }
  
  if(model_type_ == amcl::LASER_MODEL_BEAM) {
    std::cout << "amcl_wrapper: initLaser with BEAM model" << std::endl;
    laser->SetModelBeam(config.z_hit_,
			config.z_short_,
			config.z_max_,
			config.z_rand_,
			config.sigma_hit_,
			config.lambda_short_, 0.0);
  } else if(model_type_ == amcl::LASER_MODEL_LIKELIHOOD_FIELD_PROB) {
    std::cout << "amcl_wrapper: initLaser with LIKELIHOOD_FIELD_PROB model" << std::endl;    
    laser->SetModelLikelihoodFieldProb(config.z_hit_,
				       config.z_rand_,
				       config.sigma_hit_,
				       config.likelihood_model_.max_dist_, 
				       config.likelihood_model_.do_beamskip_,
				       config.likelihood_model_.beam_skip_distance_, 
				       config.likelihood_model_.beam_skip_threshold_,
				       config.likelihood_model_.beam_skip_error_threshold_);
  } else if(model_type_ == amcl::LASER_MODEL_LIKELIHOOD_FIELD) {
    std::cout << "amcl_wrapper: initLaser with LIKELIHOOD_FIELD model" << std::endl;        
    //RTC_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser->SetModelLikelihoodField(config.z_hit_,
				   config.z_rand_,
				   config.sigma_hit_,
				   config.likelihood_model_.max_dist_);
  } else {
    std::cout << "amcl_wrapper: unknown model initialization. failed." << std::endl;
    return nullptr;
  }
  return laser;
}


///
/// オドメトリモデルの初期化
///
amcl::AMCLOdom* initOdom(const odom_config& config) {

#ifdef _DEBUG
  std::cout << "initOdom() called" << std::endl;
  std::cout << "odom_config :" << std::endl;
  std::cout << "  model_type_str_ : " << config.model_type_str_.c_str() << std::endl;
  std::cout << "  alpha1_ : " << config.alpha1_ << std::endl;
  std::cout << "  alpha2_ : " << config.alpha2_ << std::endl;
  std::cout << "  alpha3_ : " << config.alpha3_ << std::endl;
  std::cout << "  alpha4_ : " << config.alpha4_ << std::endl;
  std::cout << "  alpha5_ : " << config.alpha5_ << std::endl;          
#endif
  

  auto model_type = amcl::ODOM_MODEL_DIFF;
  if (config.model_type_str_ == "diff") {
    model_type = amcl::ODOM_MODEL_DIFF;
  } else if (config.model_type_str_ == "omni") {
    model_type = amcl::ODOM_MODEL_OMNI;
  } else if (config.model_type_str_ == "diff-corrected") {
    model_type = amcl::ODOM_MODEL_DIFF_CORRECTED;
  } else if (config.model_type_str_ == "omni-corrected") {
    model_type = amcl::ODOM_MODEL_OMNI_CORRECTED;
  } else {
    std::cout << "initOdom failed. config file has invalid model type (" << config.model_type_str_.c_str() << std::endl;
    return nullptr;
  }
  
  amcl::AMCLOdom* odom = new amcl::AMCLOdom();
  if (!odom) {
    std::cout << "failed to allocate memory" << std::endl;
    return nullptr;
  }
  odom->SetModel(model_type,
		 config.alpha1_,
		 config.alpha2_,
		 config.alpha3_,
		 config.alpha4_,
		 config.alpha5_);
  return odom;
}

static pf_vector_t uniformPoseGenerator(void* arg)
{
  map_t* map = (map_t*)arg;
#if NEW_UNIFORM_SAMPLING
  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int,int> free_point = free_space_indices[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale)/2.0 - map->origin_x;
  max_x = (map->size_x * map->scale)/2.0 + map->origin_x;
  min_y = (map->size_y * map->scale)/2.0 - map->origin_y;
  max_y = (map->size_y * map->scale)/2.0 + map->origin_y;

  pf_vector_t p;

  for(;;)
  {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i,j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
      break;
  }
#endif
  return p;
}


pf_t* initPF(const pf_config& config, map_t* map) {

#ifdef _DEBUG
  std::cout << "initPF() called." << std::endl;
  std::cout << "pf_config:" << std::endl;
  std::cout << "  min_particles_: "<< config.min_particles_ << std::endl;
  std::cout << "  max_particles_: "<< config.max_particles_ << std::endl;
  std::cout << "  alpha_slow_   : "<< config.alpha_slow_ << std::endl;
  std::cout << "  alpha_fast_   : "<< config.alpha_fast_ << std::endl;
  std::cout << "  pf_err_       : "<< config.pf_err_ << std::endl;
  std::cout << "  pf_z_         : "<< config.pf_z_ << std::endl;
  std::cout << "  init_pose_    : ("<< config.init_pose_[0] << ", " << config.init_pose_[1] << ", " << config.init_pose_[2] << ")" << std::endl;
  std::cout << "  init_cov_     : ("<< config.init_cov_[0] << ", " << config.init_cov_[1] << ", " << config.init_cov_[2] << ")" << std::endl;      
  //  config.init_model_ = (pf_init_model_fn_t)uniformPoseGenerator;
  //  config.selective_resampling_ = false;
#endif  
  pf_t* pf = pf_alloc(config.min_particles_,
		       config.max_particles_,
		       config.alpha_slow_,
		       config.alpha_fast_,
		       (pf_init_model_fn_t)uniformPoseGenerator,
		       (void *)map);
  //  pf_set_selective_resampling(pf, config.selective_resampling_);
  pf_set_selective_resampling(pf, false);
  pf->pop_err = config.pf_err_;
  pf->pop_z = config.pf_z_;

  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = config.init_pose_[0];
  pf_init_pose_mean.v[1] = config.init_pose_[1];
  pf_init_pose_mean.v[2] = config.init_pose_[2];
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = config.init_cov_[0];
  pf_init_pose_cov.m[1][1] = config.init_cov_[1];
  pf_init_pose_cov.m[2][2] = config.init_cov_[2];
  pf_init(pf, pf_init_pose_mean, pf_init_pose_cov);

  return pf;
}


amcl::AMCLLaserData* convertLaser(amcl::AMCLLaser* laser, const RTC::RangeData& range, const laser_config& config) {
  amcl::AMCLLaserData* pldata = new amcl::AMCLLaserData();
  amcl::AMCLLaserData& ldata = *pldata;
  ldata.sensor = laser;
  
  pf_vector_t laser_pose;
  laser_pose.v[0] = range.geometry.geometry.pose.position.x;
  laser_pose.v[1] = range.geometry.geometry.pose.position.y;
  if (range.geometry.geometry.pose.orientation.y != 0.0) {
    //RTC_WARN(("AMCLRTC currrent version does not support angular offset of Ranger sensor. Value is %f", range.geometry.geometry.pose.orientation.y));
  }
  laser_pose.v[2] = 0; // Angular Offset is not allowed currently
  //  std::cout << " - laser_offset(" << laser_pose.v[0] << "," << laser_pose.v[1] << "," << laser_pose.v[2] << std::endl;
  laser->SetLaserPose(laser_pose);
  ldata.range_count = range.ranges.length();
  if(config.max_range_ > 0.0)
    ldata.range_max = MIN(range.config.maxRange, config.max_range_);
  else
    ldata.range_max = range.config.maxRange;

  const double range_min = MAX(range.config.minRange, config.min_range_);

  const double angle_min = range.config.minAngle;
  const double angle_increment = range.config.angularRes;
  /*
  std::cout << "LaserData:" << std::endl;
  std::cout << "  range_max      : " << ldata.range_max << std::endl;
  std::cout << "  range_min      : " << range_min << std::endl;  
  std::cout << "  angle_min      : " << angle_min << std::endl;
  std::cout << "  angle_increment: " << angle_increment << std::endl;
  std::cout << "  range_count    : " << ldata.range_count << std::endl;
  */
  ldata.ranges = new double[ldata.range_count][2];

  bool inv_rotate = false;
  for(int i = 0;i < ldata.range_count;i++) {
    // amcl doesn't (yet) have a concept of min range.  So we'll map short
    // readings to max range.
    if(range.ranges[i] <= range_min) {
      if (inv_rotate) {
	ldata.ranges[ldata.range_count - i - 1][0] = ldata.range_max;
      } else {
	ldata.ranges[i][0] = ldata.range_max;
      }
    } else {
      if (inv_rotate) {
	ldata.ranges[ldata.range_count - i - 1][0] = range.ranges[i];
      } else {
	ldata.ranges[i][0] = range.ranges[i];
      }

    }
    //    std::cout << "range: " << ldata.ranges[i][0] << std::endl;
    // Compute bearing
    ldata.ranges[i][1] = -( angle_min + (i * angle_increment) );
  }
  return pldata;
}

