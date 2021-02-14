// -*-C++-*-
/*!
 * @file  MapServerSVC_impl.cpp
 * @brief Service implementation code of MapServer.idl
 *
 */

#include "MapServerSVC_impl.h"

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
NAVIGATION::MAP_RETURN_STATUS NAVIGATION_OccupancyGridMapServerSVC_impl::updateLocalMap(const NAVIGATION::OccupancyGridMapRequestParam& param, const NAVIGATION::OccupancyGridMap& map)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <NAVIGATION::MAP_RETURN_STATUS NAVIGATION_OccupancyGridMapServerSVC_impl::updateLocalMap(const NAVIGATION::OccupancyGridMapRequestParam& param, const NAVIGATION::OccupancyGridMap& map)>"
#endif
  return 0;
}

NAVIGATION::MAP_RETURN_STATUS NAVIGATION_OccupancyGridMapServerSVC_impl::requestLocalMap(const NAVIGATION::OccupancyGridMapRequestParam& param, NAVIGATION::OccupancyGridMap_out map)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <NAVIGATION::MAP_RETURN_STATUS NAVIGATION_OccupancyGridMapServerSVC_impl::requestLocalMap(const NAVIGATION::OccupancyGridMapRequestParam& param, NAVIGATION::OccupancyGridMap_out map)>"
#endif
  return 0;
}

NAVIGATION::MAP_RETURN_STATUS NAVIGATION_OccupancyGridMapServerSVC_impl::getWholeMapConig(NAVIGATION::OccupancyGridMapConfig& config)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <NAVIGATION::MAP_RETURN_STATUS NAVIGATION_OccupancyGridMapServerSVC_impl::getWholeMapConig(NAVIGATION::OccupancyGridMapConfig& config)>"
#endif
  return 0;
}



// End of example implementational code

/*
 * Example implementational code for IDL interface NAVIGATION::OccupancyGridMapClient
 */
NAVIGATION_OccupancyGridMapClientSVC_impl::NAVIGATION_OccupancyGridMapClientSVC_impl()
{
  // Please add extra constructor code here.
}


NAVIGATION_OccupancyGridMapClientSVC_impl::~NAVIGATION_OccupancyGridMapClientSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
NAVIGATION::MAP_RETURN_STATUS NAVIGATION_OccupancyGridMapClientSVC_impl::notifyUpdateMap(const NAVIGATION::OccupancyGridMapConfig& config)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <NAVIGATION::MAP_RETURN_STATUS NAVIGATION_OccupancyGridMapClientSVC_impl::notifyUpdateMap(const NAVIGATION::OccupancyGridMapConfig& config)>"
#endif
  return 0;
}



// End of example implementational code



