// -*- C++ -*-
/*!
 * @file  AMCLRTCTest.cpp
 * @brief Adaptive Monte Carlo Localization (AMCL) module
 * @date $Date$
 *
 * $Id$
 */

#include "AMCLRTCTest.h"

// Module specification
// <rtc-template block="module_spec">
static const char* amclrtc_spec[] =
  {
    "implementation_id", "AMCLRTCTest",
    "type_name",         "AMCLRTCTest",
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
AMCLRTCTest::AMCLRTCTest(RTC::Manager* manager)
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
AMCLRTCTest::~AMCLRTCTest()
{
}



RTC::ReturnCode_t AMCLRTCTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("estimatedPose", m_estimatedPoseIn);

  // Set OutPort buffer
  addOutPort("robotPose", m_robotPoseOut);
  addOutPort("range", m_rangeOut);

  // Set service provider to Ports
  m_mapServicePort.registerProvider("NAVIGATION_OccupancyGridMapServer", "NAVIGATION::OccupancyGridMapServer", m_NAVIGATION_OccupancyGridMapServer);

  // Set service consumers to Ports
  m_mclServicePort.registerConsumer("NAVIGATION_MonteCarloLocalization", "NAVIGATION::MonteCarloLocalization", m_NAVIGATION_MonteCarloLocalization);

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
RTC::ReturnCode_t AMCLRTCTest::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AMCLRTCTest::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AMCLRTCTest::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t AMCLRTCTest::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t AMCLRTCTest::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t AMCLRTCTest::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t AMCLRTCTest::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AMCLRTCTest::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AMCLRTCTest::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AMCLRTCTest::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AMCLRTCTest::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void AMCLRTCTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(amclrtc_spec);
    manager->registerFactory(profile,
                             RTC::Create<AMCLRTCTest>,
                             RTC::Delete<AMCLRTCTest>);
  }

};


