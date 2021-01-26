// -*-C++-*-
/*!
 * @file  MobileRobotNavigationSVC_impl.h
 * @brief Service implementation header of MobileRobotNavigation.idl
 *
 */

#include "InterfaceDataTypesSkel.h"
#include "BasicDataTypeSkel.h"
#include "ExtendedDataTypesSkel.h"

#include "MobileRobotNavigationSkel.h"

#ifndef MOBILEROBOTNAVIGATIONSVC_IMPL_H
#define MOBILEROBOTNAVIGATIONSVC_IMPL_H
 
/*!
 * @class NAVIGATION_OccupancyGridMapServerSVC_impl
 * Example class implementing IDL interface NAVIGATION::OccupancyGridMapServer
 */
class NAVIGATION_OccupancyGridMapServerSVC_impl
 : public virtual POA_NAVIGATION::OccupancyGridMapServer,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~NAVIGATION_OccupancyGridMapServerSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   NAVIGATION_OccupancyGridMapServerSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~NAVIGATION_OccupancyGridMapServerSVC_impl();

   // attributes and operations
   NAVIGATION::MAP_RETURN_STATUS requestLocalMap(const NAVIGATION::OccupancyGridMapRequestParam& param, NAVIGATION::OccupancyGridMap_out map);

};

/*!
 * @class NAVIGATION_MonteCarloLocalizationSVC_impl
 * Example class implementing IDL interface NAVIGATION::MonteCarloLocalization
 */
class NAVIGATION_MonteCarloLocalizationSVC_impl
 : public virtual POA_NAVIGATION::MonteCarloLocalization,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~NAVIGATION_MonteCarloLocalizationSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   NAVIGATION_MonteCarloLocalizationSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~NAVIGATION_MonteCarloLocalizationSVC_impl();

   // attributes and operations
   NAVIGATION::MCL_RETURN_STATUS resetParticles(const NAVIGATION::ParticleResetParam& param);
   NAVIGATION::MCL_RETURN_STATUS requestParticles(NAVIGATION::MCLInfo_out particles);
   NAVIGATION::MCL_RETURN_STATUS setParticles(const NAVIGATION::MCLInfo& particles);

};



#endif // MOBILEROBOTNAVIGATIONSVC_IMPL_H


