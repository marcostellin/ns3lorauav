/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 * Author: Marco Stellin
 */

#include "ns3/load-monitor.h"
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LoadMonitor");
NS_OBJECT_ENSURE_REGISTERED (LoadMonitor);

TypeId
LoadMonitor::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LoadMonitor")
    .SetParent<Object> ()
    .AddConstructor<LoadMonitor> ();

    return tid;
}

LoadMonitor::LoadMonitor ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

LoadMonitor::LoadMonitor (Ptr<Ipv4L3Protocol> protocol )
{
  NS_LOG_FUNCTION_NOARGS ();

  m_protocol = protocol;
  NS_ASSERT (m_protocol != 0);
  m_protocol -> TraceConnectWithoutContext ("UnicastForward", MakeCallback (&LoadMonitor::ForwardedPacket, this));

}

LoadMonitor::~LoadMonitor ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void
LoadMonitor::ForwardedPacket (const Ipv4Header &header, Ptr< const Packet > packet, uint32_t interface)
{
  if (!m_load.empty ())
  {
    while (!m_load.empty () && m_load.front () < Simulator::Now () - m_interval)
      m_load.pop ();
  }

  m_load.push (Simulator::Now ());

  NS_LOG_INFO ("Size: " << m_load.size ());

  std::queue<Time> test = m_load;

  // uint32_t cnt = 0;
  // while (!test.empty ())
  // {
  //   NS_LOG_INFO (cnt << " : " << test.front ());
  //   test.pop ();
  //   cnt++;
  // }
}

void 
LoadMonitor::SetLoadInterval (Time interval)
{
  m_interval = interval;
}

double 
LoadMonitor::GetLoad (void)
{
  std::queue<Time> copy = m_load;

  if (copy.empty () )
    return 0;

  while (!copy.empty() && copy.front () < Simulator::Now () - m_interval)
      copy.pop ();
  
  return (double)copy.size () / m_interval.GetSeconds ();
}

bool
LoadMonitor::IsBusy (void)
{
  std::queue<Time> copy = m_load;

  if (copy.empty () )
    return false;

  while (!copy.empty() && copy.front () < Simulator::Now () - m_interval)
      copy.pop ();
  
  return copy.size () > 0;
}

}
