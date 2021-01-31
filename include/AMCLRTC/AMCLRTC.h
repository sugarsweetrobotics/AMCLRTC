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

#include "amcl_wrapper.h"






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
  RTC::TimedPose2D m_receivedPoseWhenUpdated;
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

  // マップ取得後の処理を行う．
  bool handleMapMessage(const NAVIGATION::OccupancyGridMap& map);
  
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
    //s td::lock_guard<std::mutex> guard(m_pf_lock);
    //  std::cout << "[AMCLRTC::setMCLInfo called" << std::endl;
    const pf_sample_set_t& set = pf_->sets[pf_->current_set];
    const long len = set.sample_count;
    info->particles.length(len);
    double maxWeight = -1;
    int maxWeightIndex = -1;
    for(int i = 0;i < len;i++) {
      info->particles[i].pose.position.x = set.samples[i].pose.v[0];
      info->particles[i].pose.position.y = set.samples[i].pose.v[1];
      info->particles[i].pose.heading    = set.samples[i].pose.v[2];
      info->particles[i].weight          = set.samples[i].weight;
      if (maxWeight < set.samples[i].weight) {
	maxWeight = set.samples[i].weight;
	maxWeightIndex = i;
      }
      info->maxWeight = maxWeight;
      info->maxWeightIndex = maxWeightIndex;
    }
    return true;
  }

};







extern "C"
{
  DLL_EXPORT void AMCLRTCInit(RTC::Manager* manager);
};

#endif // AMCLRTC_H
