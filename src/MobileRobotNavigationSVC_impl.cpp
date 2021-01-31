// -*-C++-*-
/*!
 * @file  MobileRobotNavigationSVC_impl.cpp
 * @brief Service implementation code of MobileRobotNavigation.idl
 *
 */

#include <mutex>
#include "AMCLRTC.h"
#include "MobileRobotNavigationSVC_impl.h"


/*
 * Example implementational code for IDL interface NAVIGATION::OccupancyGridMapServer
 */
NAVIGATION_OccupancyGridMapServerSVC_impl::NAVIGATION_OccupancyGridMapServerSVC_impl()
{
  // Please add extra constructor code here.
}


NAVIGATION_OccupancyGridMapServerSVC_impl::~NAVIGATION_OccupancyGridMapServerSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
NAVIGATION::MAP_RETURN_STATUS NAVIGATION_OccupancyGridMapServerSVC_impl::requestLocalMap(const NAVIGATION::OccupancyGridMapRequestParam& param, NAVIGATION::OccupancyGridMap_out map)
{
  return NAVIGATION::MAP_RETVAL_NOT_IMPL;
}



// End of example implementational code

/*
 * Example implementational code for IDL interface NAVIGATION::MonteCarloLocalization
 */
NAVIGATION_MonteCarloLocalizationSVC_impl::NAVIGATION_MonteCarloLocalizationSVC_impl()
{
  // Please add extra constructor code here.
}


NAVIGATION_MonteCarloLocalizationSVC_impl::~NAVIGATION_MonteCarloLocalizationSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
NAVIGATION::MCL_RETURN_STATUS NAVIGATION_MonteCarloLocalizationSVC_impl::resetParticles(const NAVIGATION::ParticleResetParam& param)
{
  
  return NAVIGATION::MCL_RETVAL_NOT_IMPL;
}


NAVIGATION::MCL_RETURN_STATUS NAVIGATION_MonteCarloLocalizationSVC_impl::requestParticles(NAVIGATION::MCLInfo_out particles)
{
  //  std::cout << "MCLSVC::requestParticles called" << std::endl;
  NAVIGATION::MCLInfo_var info(new NAVIGATION::MCLInfo());
  m_pRTC->setMCLInfo(info);
  particles = info._retn();
  //  std::cout << "MCLSVC::requestParticles exit" << std::endl;
  return NAVIGATION::MCL_RETVAL_OK;
}

NAVIGATION::MCL_RETURN_STATUS NAVIGATION_MonteCarloLocalizationSVC_impl::setParticles(const NAVIGATION::MCLInfo& particles)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <NAVIGATION::MCL_RETURN_STATUS NAVIGATION_MonteCarloLocalizationSVC_impl::setParticles(const NAVIGATION::MCLInfo& particles)>"
#endif
  return NAVIGATION::MCL_RETVAL_NOT_IMPL;
}



// End of example implementational code



