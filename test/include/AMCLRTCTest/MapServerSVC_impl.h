// -*-C++-*-
/*!
 * @file  MapServerSVC_impl.h
 * @brief Service implementation header of MapServer.idl
 *
 */

#include "NavigationDataTypeSkel.h"
#include "InterfaceDataTypesSkel.h"
#include "BasicDataTypeSkel.h"
#include "ExtendedDataTypesSkel.h"

#include "MapServerSkel.h"

#ifndef MAPSERVERSVC_IMPL_H
#define MAPSERVERSVC_IMPL_H
 
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
   NAVIGATION::MAP_RETURN_STATUS updateLocalMap(const NAVIGATION::OccupancyGridMapRequestParam& param, const NAVIGATION::OccupancyGridMap& map);
   NAVIGATION::MAP_RETURN_STATUS requestLocalMap(const NAVIGATION::OccupancyGridMapRequestParam& param, NAVIGATION::OccupancyGridMap_out map);
   NAVIGATION::MAP_RETURN_STATUS getWholeMapConig(NAVIGATION::OccupancyGridMapConfig& config);

};

/*!
 * @class NAVIGATION_OccupancyGridMapClientSVC_impl
 * Example class implementing IDL interface NAVIGATION::OccupancyGridMapClient
 */
class NAVIGATION_OccupancyGridMapClientSVC_impl
 : public virtual POA_NAVIGATION::OccupancyGridMapClient,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~NAVIGATION_OccupancyGridMapClientSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   NAVIGATION_OccupancyGridMapClientSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~NAVIGATION_OccupancyGridMapClientSVC_impl();

   // attributes and operations
   NAVIGATION::MAP_RETURN_STATUS notifyUpdateMap(const NAVIGATION::OccupancyGridMapConfig& config);

};



#endif // MAPSERVERSVC_IMPL_H


