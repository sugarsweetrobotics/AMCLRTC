﻿// -*- C++ -*-
/*!
 * @file  AMCLRTC.cpp
 * @brief Adaptive Monte Carlo Localization (AMCL) module
 * @date $Date$
 *
 * $Id$
 */

#include "AMCLRTC.h"


// Module specification
// <rtc-template block="module_spec">
static const char* amclrtc_spec[] =
  {
    "implementation_id", "AMCLRTC",
    "type_name",         "AMCLRTC",
    "description",       "Adaptive Monte Carlo Localization (AMCL) module",
    "version",           "1.0.0",
    "vendor",            "sugarsweetrobotics",
    "category",          "Navigation",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug_level", "1",
    "conf.default.initial_pose_x", "0.0",
    "conf.default.initial_pose_y", "0.0",
    "conf.default.initial_pose_th", "0.0",
    // Widget
    "conf.__widget__.debug_level", "text",
    // Constraints

    "conf.__type__.debug_level", "long",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
AMCLRTC::AMCLRTC(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_robotPoseIn("robotPose", m_robotPose),
    m_rangeIn("range", m_range),
    m_estimatedPoseOut("estimatedPose", m_estimatedPose),
    m_mclServicePort("mclService"),
    m_mapServicePort("mapService")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
AMCLRTC::~AMCLRTC()
{
}

inline pf_vector_t convertPose(const RTC::Pose2D& x) {
  pf_vector_t d;
  d.v[0] = x.position.x;
  d.v[1] = x.position.y;
  d.v[2] = x.heading;
  return d;
}

inline RTC::Pose2D convertPose(const pf_vector_t& x) {
  RTC::Pose2D d;
  d.position.x = x.v[0];
  d.position.y = x.v[1];
  d.heading = x.v[2];
  return d;
}

inline double angular_diff(const double x, const double y) {
  double d = x - y;
  while(d > M_PI) d-=2*M_PI;
  while(d < -M_PI) d+=2*M_PI;
  return d;
}

inline pf_vector_t diff(const RTC::Pose2D& x0, const RTC::Pose2D& x1) {
  pf_vector_t delta = pf_vector_zero();
  delta.v[0] = x0.position.x - x1.position.x;
  delta.v[1] = x0.position.y - x1.position.y;
  delta.v[2] = angular_diff(x0.heading, x1.heading);
  return delta;
}


bool checkNeedUpdate(const pf_vector_t& delta, const pf_config& config) {
  return (fabs(delta.v[0]) > config.d_thresh_ ||
	  fabs(delta.v[1]) > config.d_thresh_ ||
	  fabs(delta.v[2]) > config.a_thresh_);
}

// Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;
  // Mean of pose esimate
  pf_vector_t pf_pose_mean;
  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;
} amcl_hyp_t;

inline bool estimatePose(pf_t* pf, RTC::Pose2D& estimatedPose) {
  double max_weight = 0.0;
  int max_weight_count = -1;
  std::vector<amcl_hyp_t> hyps;
  
  const long len = pf->sets[pf->current_set].cluster_count;
  for(int i = 0;i < len;++i) {
    amcl_hyp_t hyp;
    if (!pf_get_cluster_stats(pf, i, &hyp.weight, &hyp.pf_pose_mean, &hyp.pf_pose_cov)) {
      return false;
    }
    if (max_weight < hyp.weight) {
      max_weight = hyp.weight;
      max_weight_count = i;
    }
    hyps.emplace_back(hyp);
  }

  if (max_weight < 0.0) {
    return false;
  }

  estimatedPose.position.x = hyps[max_weight_count].pf_pose_mean.v[0];
  estimatedPose.position.y = hyps[max_weight_count].pf_pose_mean.v[1];
  estimatedPose.heading = hyps[max_weight_count].pf_pose_mean.v[2];

  return true;
}

bool AMCLRTC::handleScan() {
  std::lock_guard<std::mutex> guard(m_pf_lock);  
  const pf_vector_t delta = diff(m_robotPose.data, m_estimatedPose.data);
  amcl::AMCLOdomData odata;
  odata.pose = convertPose(m_robotPose.data);
  odata.delta = delta;
  odom_->UpdateAction(pf_, (amcl::AMCLSensorData*)&odata);
  
  amcl::AMCLLaserData laserData = convertLaser(laser_, m_range, laser_config_);
  laser_->UpdateSensor(pf_, (amcl::AMCLSensorData*)&laserData);

  const bool needUpdatePF = checkNeedUpdate(delta, pf_config_);
  if (needUpdatePF) {
    pf_update_resample(pf_);
    if (estimatePose(pf_, m_estimatedPose.data)) {

      return true;
    }
  }
  return false;
}

void AMCLRTC::handleMapMessage(const NAVIGATION::OccupancyGridMap& map) {
  //const nav_msgs::OccupancyGrid& msg;
  //boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  auto pixel_x = map.config.sizeOfMap.l / map.config.sizeOfGrid.l;
  auto pixel_y = map.config.sizeOfMap.w / map.config.sizeOfGrid.w;
  RTC_INFO(("AMCLRTC::handleMapMessage(map w=%d,h=%d)", pixel_x, pixel_y));
  freeMapDependentMemory();
  
  // Clear queued laser objects because they hold pointers to the existing
  // map, #5202.
  //lasers_.clear();
  //lasers_update_.clear();
  //frame_to_laser_.clear();

  map_ = convertMap(map);

#if NEW_UNIFORM_SAMPLING
  // Index of free space
  free_space_indices.resize(0);
  for(int i = 0; i < map_->size_x; i++)
    for(int j = 0; j < map_->size_y; j++)
      if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
        free_space_indices.push_back(std::make_pair(i,j));
#endif
  // Create the particle filter
  delete pf_;
  pf_ = initPF(pf_config_, map_);
  //  pf_init_ = false;  
  // Instantiate the sensor objects
  // Odometry
  delete odom_;
  odom_ = initOdom(odom_config_);
  // Laser
  delete laser_;
  laser_ = initLaser(laser_config_, map_);
  
  // In case the initial pose message arrived before the first map,
  // try to apply the initial pose now that the map has arrived.
  // applyInitialPose();
}

RTC::ReturnCode_t AMCLRTC::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("robotPose", m_robotPoseIn);
  addInPort("range", m_rangeIn);

  // Set OutPort buffer
  addOutPort("estimatedPose", m_estimatedPoseOut);

  // Set service provider to Ports
  m_mclServicePort.registerProvider("NAVIGATION_MonteCarloLocalization", "NAVIGATION::MonteCarloLocalization", m_NAVIGATION_MonteCarloLocalization);

  // Set service consumers to Ports
  m_mapServicePort.registerConsumer("NAVIGATION_OccupancyGridMapServer", "NAVIGATION::OccupancyGridMapServer", m_NAVIGATION_OccupancyGridMapServer);

  // Set CORBA Service Ports
  addPort(m_mclServicePort);
  addPort(m_mapServicePort);

  m_NAVIGATION_MonteCarloLocalization.setRTC(this);
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug_level", m_debug_level, "1");
  bindParameter("initial_pose_x", m_initial_pose_x, "0.0");
  bindParameter("initial_pose_y", m_initial_pose_y, "0.0");
  bindParameter("initial_pose_th", m_initial_pose_th, "0.0");

  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t AMCLRTC::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AMCLRTC::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AMCLRTC::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t AMCLRTC::onActivated(RTC::UniqueId ec_id)
{
  NAVIGATION::OccupancyGridMap_var map;
  NAVIGATION::OccupancyGridMapRequestParam param;
  param.globalPositionOfCenter.position.x = 0;
  param.globalPositionOfCenter.position.y = 0;
  param.globalPositionOfCenter.heading = 0;
  param.sizeOfMap.l = 100;
  param.sizeOfMap.w = 100;
  param.sizeOfGrid.l = 0.05;
  param.sizeOfGrid.w = 0.05;
  auto retval = m_NAVIGATION_OccupancyGridMapServer->requestLocalMap(param, map);


  while(!m_robotPoseIn.isNew()) {
    m_robotPoseIn.read();
    m_oldPose = m_robotPose;
  }
  m_estimatedPose.data.position.x = m_initial_pose_x;
  m_estimatedPose.data.position.y = m_initial_pose_y;
  m_estimatedPose.data.heading = m_initial_pose_th;
  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t AMCLRTC::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t AMCLRTC::onExecute(RTC::UniqueId ec_id)
{
  if (m_robotPoseIn.isNew()) {
    m_robotPoseIn.read();
    
  }

  if (m_rangeIn.isNew()) {
    m_rangeIn.read();
    if (handleScan()) {
      setTimestamp(m_estimatedPose);
      m_estimatedPoseOut.write();
      m_oldPose = m_estimatedPose;
    }
  }

  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t AMCLRTC::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AMCLRTC::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AMCLRTC::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AMCLRTC::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AMCLRTC::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void AMCLRTCInit(RTC::Manager* manager)
  {
    coil::Properties profile(amclrtc_spec);
    manager->registerFactory(profile,
                             RTC::Create<AMCLRTC>,
                             RTC::Delete<AMCLRTC>);
  }

};


