// -*- C++ -*-
/*!
 * @file  AMCLRTC.h
 * @brief Adaptive Monte Carlo Localization (AMCL) module
 * @date  $Date$
 *
 * $Id$
 */

#ifndef AMCLRTC_H
#define AMCLRTC_H


#define _USE_MATH_DEFINES
#include <math.h>
#include <mutex>
#include <algorithm>


#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "MobileRobotNavigationSVC_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
#include "ExtendedDataTypesStub.h"
#include "InterfaceDataTypesStub.h"

// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include "amcl/map/map.h"
#include "amcl/pf/pf.h"
#include "amcl/sensors/amcl_odom.h"
#include "amcl/sensors/amcl_laser.h"

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



/*!
 * @class AMCLRTC
 * @brief Adaptive Monte Carlo Localization (AMCL) module
 *
 */
class AMCLRTC
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  AMCLRTC(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~AMCLRTC();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 
   * - Name:  debug_level
   * - DefaultValue: 1
   */
  long int m_debug_level;

  double m_initial_pose_x;
  double m_initial_pose_y;
  double m_initial_pose_th;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::TimedPose2D m_robotPose;
  /*!
   */
  RTC::InPort<RTC::TimedPose2D> m_robotPoseIn;
  RTC::RangeData m_range;
  /*!
   */
  RTC::InPort<RTC::RangeData> m_rangeIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedPose2D m_estimatedPose;
  /*!
   */
  RTC::OutPort<RTC::TimedPose2D> m_estimatedPoseOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  /*!
   */
  RTC::CorbaPort m_mclServicePort;
  /*!
   */
  RTC::CorbaPort m_mapServicePort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  /*!
   */
  NAVIGATION_MonteCarloLocalizationSVC_impl m_NAVIGATION_MonteCarloLocalization;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  /*!
   */
  RTC::CorbaConsumer<NAVIGATION::OccupancyGridMapServer> m_NAVIGATION_OccupancyGridMapServer;
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>


  RTC::TimedPose2D m_oldPose;
public:
  std::mutex m_pf_lock;
private:
  
  map_t* map_;
  //  char* mapdata;
  //    int sx, sy;
  //    double resolution;

  // Particle filter
  pf_t *pf_;
  pf_config pf_config_;
  // bool pf_init_;
  
  //  pf_vector_t pf_odom_pose_;

  //  int resample_count_;  
  //  double init_pose_[3];
  //  double init_cov_[3];

  // Odometry Model
  amcl::AMCLOdom* odom_;  
  odom_config odom_config_;
  
  /*
    double pf_err_, pf_z_;
    int resample_interval_;
    double alpha_slow_, alpha_fast_;
    bool selective_resampling_;
  */
  //    double laser_min_range_;


  //int max_beams_;
  //int min_particles_, max_particles_;

  /*
    odom_model_t odom_model_type_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
  */


    //beam skip related params
  //bool do_beamskip_;
  //double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
  //double laser_likelihood_max_dist_;

  //laser_model_t laser_model_type_;
  //    bool tf_broadcast_;


  
#if NEW_UNIFORM_SAMPLING
  std::vector<std::pair<int,int> > free_space_indices;
#endif
  

  amcl::AMCLLaser* laser_;
  laser_config laser_config_;
  //std::vector<amcl::AMCLLaser*> lasers_;
  //  std::vector<bool> lasers_update_;
  //  std::map<std::string, int> frame_to_laser_;

  void handleMapMessage(const NAVIGATION::OccupancyGridMap& map);
  bool handleScan();
  
  void freeMapDependentMemory() {
    if( map_ != NULL ) {
      map_free( map_ );
      map_ = NULL;
    }
    if( pf_ != NULL ) {
      pf_free( pf_ );
      pf_ = NULL;
    }
    delete odom_;
    odom_ = NULL;
    delete laser_;
    laser_ = NULL;
  }

public:
  bool setMCLInfo(NAVIGATION::MCLInfo_var& info) {
    std::lock_guard<std::mutex> guard(m_pf_lock);
    const pf_sample_set_t& set = pf_->sets[pf_->current_set];
    const long len = set.cluster_count;
    info->particles.length(len);
    for(int i = 0;i < len;i++) {
      info->particles[i].pose.position.x = set.samples[i].pose.v[0];
      info->particles[i].pose.position.y = set.samples[i].pose.v[1];
      info->particles[i].pose.heading    = set.samples[i].pose.v[2]; 
    }
    return true;
  }

};


inline amcl::AMCLLaser* initLaser(const laser_config& config, map_t* map) {
  amcl::AMCLLaser* laser = new amcl::AMCLLaser(config.max_beams_, map);

  if(config.model_type_ == amcl::LASER_MODEL_BEAM) {
    laser->SetModelBeam(config.z_hit_,
			config.z_short_,
			config.z_max_,
			config.z_rand_,
			config.sigma_hit_,
			config.lambda_short_, 0.0);
  } else if(config.model_type_ == amcl::LASER_MODEL_LIKELIHOOD_FIELD_PROB) {
    laser->SetModelLikelihoodFieldProb(config.z_hit_,
				       config.z_rand_,
				       config.sigma_hit_,
				       config.likelihood_model_.max_dist_, 
				       config.likelihood_model_.do_beamskip_,
				       config.likelihood_model_.beam_skip_distance_, 
				       config.likelihood_model_.beam_skip_threshold_,
				       config.likelihood_model_.beam_skip_error_threshold_);
  } else if(config.model_type_ == amcl::LASER_MODEL_LIKELIHOOD_FIELD) {
    //RTC_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser->SetModelLikelihoodField(config.z_hit_,
				   config.z_rand_,
				   config.sigma_hit_,
				   config.likelihood_model_.max_dist_);
  }
  return laser;
}

inline amcl::AMCLOdom* initOdom(const odom_config& config) {
  amcl::AMCLOdom* odom = new amcl::AMCLOdom();
  odom->SetModel(config.model_type_,
		 config.alpha1_,
		 config.alpha2_,
		 config.alpha3_,
		 config.alpha4_,
		 config.alpha5_);
  return odom;
}

inline pf_t* initPF(const pf_config& config, map_t* map) {
  pf_t* pf = pf_alloc(config.min_particles_,
		       config.max_particles_,
		       config.alpha_slow_,
		       config.alpha_fast_,
		       config.init_model_,//(pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
		       (void *)map);
  pf_set_selective_resampling(pf, config.selective_resampling_);
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

inline amcl::AMCLLaserData convertLaser(amcl::AMCLLaser* laser, const RTC::RangeData& range, const laser_config& config) {
  amcl::AMCLLaserData ldata;
  ldata.sensor = laser;

  pf_vector_t laser_pose;
  laser_pose.v[0] = range.geometry.geometry.pose.position.x;
  laser_pose.v[1] = range.geometry.geometry.pose.position.y;
  if (range.geometry.geometry.pose.orientation.y != 0.0) {
    //RTC_WARN(("AMCLRTC currrent version does not support angular offset of Ranger sensor. Value is %f", range.geometry.geometry.pose.orientation.y));
  }
  laser_pose.v[2] = 0; // Angular Offset is not allowed currently
  ldata.range_count = range.ranges.length();
  if(config.max_range_ > 0.0)
    ldata.range_max = MIN(range.config.maxRange, config.max_range_);
  else
    ldata.range_max = range.config.maxRange;

  const double range_min = MAX(range.config.minRange, config.min_range_);

  const double angle_min = range.config.minAngle;
  const double angle_increment = range.config.angularRes;

  ldata.ranges = new double[ldata.range_count][2];
  for(int i = 0;i < ldata.range_count;i++) {
    // amcl doesn't (yet) have a concept of min range.  So we'll map short
    // readings to max range.
    if(range.ranges[i] <= range_min)
      ldata.ranges[i][0] = ldata.range_max;
    else
      ldata.ranges[i][0] = range.ranges[i];
    // Compute bearing
    ldata.ranges[i][1] = angle_min + (i * angle_increment);
  }
  return ldata;
}

inline map_t* convertMap(const NAVIGATION::OccupancyGridMap& ogmap) {
  map_t* map = map_alloc();

  map->size_x = ogmap.config.sizeOfMap.w / ogmap.config.sizeOfGrid.w;
  map->size_y = ogmap.config.sizeOfMap.l / ogmap.config.sizeOfGrid.l;
  map->scale = ogmap.config.sizeOfMap.w;
  map->origin_x = ogmap.config.globalPositionOfTopLeft.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = ogmap.config.globalPositionOfTopLeft.position.y + (map->size_y / 2) * map->scale;
  
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  const int size = map->size_x*map->size_y;
  for (int i = 0;i < size;i++) {
    if(ogmap.cells[i] == 0) {
      map->cells[i].occ_state = -1;
    } else if(ogmap.cells[i] == 100) {
      map->cells[i].occ_state = +1;
    } else {
      map->cells[i].occ_state = 0;
    }
  }
  return map;
}

extern "C"
{
  DLL_EXPORT void AMCLRTCInit(RTC::Manager* manager);
};

#endif // AMCLRTC_H
