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

#include "particle.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/node-list.h"
#include <math.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("Particle");
NS_OBJECT_ENSURE_REGISTERED (Particle);

TypeId
Particle::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Particle")
    .SetParent<Object> ()
    .AddConstructor<Particle> ();

    return tid;
}

Particle::Particle ()
{
  NS_LOG_FUNCTION_NOARGS ();
  m_deltaRmax = 2 * M_PI;
  m_deltaVmax = m_h * m_vmax;
  Particle::InitializeParticle ();
}

Particle::~Particle ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void
Particle::InitializeParticle ()
{
  Ptr<UniformRandomVariable> gen_r = CreateObject<UniformRandomVariable> ();
  gen_r -> SetAttribute ("Min", DoubleValue (0));
  gen_r -> SetAttribute ("Max", DoubleValue (2 * M_PI));

  Ptr<UniformRandomVariable> gen_v = CreateObject<UniformRandomVariable> ();
  gen_v -> SetAttribute ("Min", DoubleValue (0));
  gen_v -> SetAttribute ("Max", DoubleValue (m_h * m_vmax));

  Ptr<UniformRandomVariable> gen_deltaV = CreateObject<UniformRandomVariable> ();
  gen_deltaV -> SetAttribute ("Min", DoubleValue (-m_vmax));
  gen_deltaV -> SetAttribute ("Max", DoubleValue (m_vmax));

  Ptr<UniformRandomVariable> gen_deltaR = CreateObject<UniformRandomVariable> ();
  gen_deltaR -> SetAttribute ("Min", DoubleValue (-2 * M_PI));
  gen_deltaR -> SetAttribute ("Max", DoubleValue (2 * M_PI));

  for (uint16_t i = 0; i < m_ataNodes.size (); i++)
  {
    Ptr<Node> node = NodeList::GetNode (m_ataNodes[i]);
    Ptr<MobilityModel> mob = node -> GetObject<MobilityModel> ();
    NodeState state;
    state.id = m_ataNodes[i];
    state.pos = mob -> GetPosition ();
    state.r = gen_r -> GetValue ();
    state.v = gen_v -> GetValue ();
    state.deltaR = gen_deltaR -> GetValue ();
    state.deltaV = gen_deltaV -> GetValue ();

    m_nodeStates.push_back (state);
  }
}

void
Particle::PrintParticleState ()
{
  for (uint16_t i = 0; i < m_nodeStates.size (); i++)
  {
    NodeState state = m_nodeStates[i];
    NS_LOG_INFO (state.id << ": " << "position=" << state.pos << " r=" << state.r << " v=" << state.v << " deltaR=" << state.deltaR << " deltaV=" << state.deltaV);
  }
}

void
Particle::SetH (uint16_t h)
{
  m_h = h;
}

void
Particle::SetVmax (double vmax)
{
  m_vmax = vmax;
}

void
Particle::AddAtaNodes(std::vector<uint32_t> ids)
{
  m_ataNodes = ids;
}

void
Particle::AddAtgNodes(std::vector<uint32_t> ids)
{
  m_atgNodes = ids;
}

}