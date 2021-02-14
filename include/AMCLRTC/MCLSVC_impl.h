// -*-C++-*-
/*!
 * @file  MCLSVC_impl.h
 * @brief Service implementation header of MCL.idl
 *
 */

#include "NavigationDataTypeSkel.h"
#include "InterfaceDataTypesSkel.h"
#include "BasicDataTypeSkel.h"
#include "ExtendedDataTypesSkel.h"

#include "MCLSkel.h"

#ifndef MCLSVC_IMPL_H
#define MCLSVC_IMPL_H
 
class AMCLRTC;
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
AMCLRTC* m_pRTC;

public:
  void setRTC(AMCLRTC* pRTC) { m_pRTC = pRTC; }
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
   NAVIGATION::MCL_RETURN_STATUS updateMap(const NAVIGATION::OccupancyGridMap& map);

};



#endif // MCLSVC_IMPL_H


