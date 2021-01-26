// -*- C++ -*-
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

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug_level", m_debug_level, "1");
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
  return RTC::RTC_OK;
}


RTC::ReturnCode_t AMCLRTC::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t AMCLRTC::onExecute(RTC::UniqueId ec_id)
{
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


