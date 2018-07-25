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
#include "udp-relay-server-helper.h"
#include "ns3/udp-server.h"
#include "ns3/udp-relay.h"
#include "ns3/udp-trace-client.h"
#include "ns3/uinteger.h"
#include "ns3/string.h"

namespace ns3 {


UdpRelayHelper::UdpRelayHelper ()
{
  m_factory.SetTypeId (UdpRelay::GetTypeId ());
}

UdpRelayHelper::UdpRelayHelper (Address address, uint16_t port)
{
  m_factory.SetTypeId (UdpRelay::GetTypeId ());
  SetAttribute ("RemoteAddress", AddressValue (address));
  SetAttribute ("RemotePort", UintegerValue (port));
}

UdpRelayHelper::UdpRelayHelper (Address address)
{
  m_factory.SetTypeId (UdpRelay::GetTypeId ());
  SetAttribute ("RemoteAddress", AddressValue (address));
}

void
UdpRelayHelper::SetAttribute (std::string name, const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
UdpRelayHelper::Install (NodeContainer c)
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      Ptr<Node> node = *i;
      Ptr<UdpRelay> client = m_factory.Create<UdpRelay> ();
      node->AddApplication (client);
      apps.Add (client);
    }
  return apps;
}


} // namespace ns3
