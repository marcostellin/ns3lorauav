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

#include "ns3/pso-manager.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"
#include "ns3/node-list.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("PsoManager");
NS_OBJECT_ENSURE_REGISTERED (PsoManager);

TypeId
PsoManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PsoManager")
    .SetParent<Object> ()
    .AddConstructor<PsoManager> ()

    .AddAttribute ("Speed", 
                   "Max speed of UAVs",
                   DoubleValue (1.39),
                   MakeDoubleAccessor (&PsoManager::m_speed),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("IntervalT", 
                   "Optimization process period",
                   TimeValue (Seconds (5.0)),
                   MakeTimeAccessor (&PsoManager::m_t),
                   MakeTimeChecker ())

    .AddAttribute ("H", 
                   "Time steps horizon",
                   UintegerValue (2),
                   MakeUintegerAccessor (&PsoManager::m_h),
                   MakeUintegerChecker<uint16_t> ())

    .AddAttribute ("Mu", 
                   "Number of particles",
                   UintegerValue (4),
                   MakeUintegerAccessor (&PsoManager::m_mu),
                   MakeUintegerChecker<uint32_t> ());

    return tid;
}

PsoManager::PsoManager ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

PsoManager::~PsoManager ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void
PsoManager::DoInitialize ()
{
  DoInitializePrivate ();
}

void
PsoManager::DoInitializePrivate ()
{
  //Initialize empty particles
  for (uint16_t i = 0; i < m_mu; i++)
  {
    Ptr<Particle> p = CreateObject<Particle> ();
    p -> AddAtaNodes (m_ataNodes);
    p -> AddAtgNodes (m_atgNodes);
    p -> SetH (m_h);
    p -> SetVmax (m_speed * m_t.GetSeconds ());

    m_particles.push_back (p);
  }

  //Schedule PSO
  Simulator::Schedule (m_t, &PsoManager::RunPso, this);
}

void
PsoManager::RunPso ()
{
  InitializePopulation ();
}

void
PsoManager::InitializePopulation ()
{
  for (uint16_t i = 0; i < m_particles.size (); i++)
  {
    Ptr<Particle> p = m_particles[i];
    p -> InitializeParticle ();
    NS_LOG_INFO ("Particle " << i);
    p -> PrintParticleState ();
  }
}

void
PsoManager::AddAtaNode(uint32_t id)
{
  m_ataNodes.push_back(id); 
}

void
PsoManager::AddAtgNode(uint32_t id)
{
  m_atgNodes.push_back(id); 
}

}