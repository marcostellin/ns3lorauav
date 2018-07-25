/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008 INRIA
 *
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
 * Author: Mohamed Amine Ismail <amine.ismail@sophia.inria.fr>
 */
#ifndef UDP_RELAY_SERVER_HELPER_H
#define UDP_RELAY_SERVER_HELPER_H

#include <stdint.h>
#include "ns3/application-container.h"
#include "ns3/node-container.h"
#include "ns3/object-factory.h"
#include "ns3/ipv4-address.h"
#include "ns3/udp-server.h"
#include "ns3/udp-relay.h"
namespace ns3 {
/**
 * \ingroup udpclientserver
 * \brief Create a server application which waits for input UDP packets
 *        and uses the information carried into their payload to compute
 *        delay and to determine if some packets are lost.
 */

class UdpRelayHelper
{

public:
  /**
   * Create UdpClientHelper which will make life easier for people trying
   * to set up simulations with udp-client-server.
   *
   */
  UdpRelayHelper ();

  /**
   *  Create UdpClientHelper which will make life easier for people trying
   * to set up simulations with udp-client-server. Use this variant with
   * addresses that do not include a port value (e.g., Ipv4Address and
   * Ipv6Address).
   *
   * \param ip The IP address of the remote UDP server
   * \param port The port number of the remote UDP server
   */

  UdpRelayHelper (Address ip, uint16_t port);
  /**
   *  Create UdpClientHelper which will make life easier for people trying
   * to set up simulations with udp-client-server. Use this variant with
   * addresses that do include a port value (e.g., InetSocketAddress and
   * Inet6SocketAddress).
   *
   * \param addr The address of the remote UDP server
   */

  UdpRelayHelper (Address addr);

  /**
   * Record an attribute to be set in each Application after it is is created.
   *
   * \param name the name of the attribute to set
   * \param value the value of the attribute to set
   */
  void SetAttribute (std::string name, const AttributeValue &value);

  /**
     * \param c the nodes
     *
     * Create one UDP client application on each of the input nodes
     *
     * \returns the applications created, one application per input node.
     */
  ApplicationContainer Install (NodeContainer c);

private:
  ObjectFactory m_factory; //!< Object factory.
};


} // namespace ns3

#endif /* UDP_CLIENT_SERVER_H */
