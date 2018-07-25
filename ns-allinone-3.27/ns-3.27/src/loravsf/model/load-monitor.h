/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Marco Stellin
 */

#ifndef LOAD_MONITOR_H
#define LOAD_MONITOR_H

#include "ns3/object.h"
#include "ns3/ipv4-l3-protocol.h"
#include <queue>

namespace ns3 {

/**
 * This class estimate the load of a node in a given interval of time
 */

class LoadMonitor : public Object
{
public:

  static TypeId GetTypeId (void);
  LoadMonitor(Ptr<Ipv4L3Protocol> protocol);
  LoadMonitor ();
  virtual ~LoadMonitor();

  void SetLoadInterval (Time interval);
  double GetLoad (void);
  bool IsBusy (void);

private:

  void ForwardedPacket (const Ipv4Header &header, Ptr< const Packet > packet, uint32_t interface);

  Ptr<Ipv4L3Protocol> m_protocol;
  Time m_interval;
  std::queue<Time> m_load;
};

}

#endif /* DEVICE_STATUS_H */
