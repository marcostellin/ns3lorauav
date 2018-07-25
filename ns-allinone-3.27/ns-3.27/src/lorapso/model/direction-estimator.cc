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

#include "direction-estimator.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/node.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("DirectionEstimator");
NS_OBJECT_ENSURE_REGISTERED (DirectionEstimator);

TypeId
DirectionEstimator::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::DirectionEstimator")
    .SetParent<Object> ()
    .AddConstructor<DirectionEstimator> ();

    return tid;
}

DirectionEstimator::DirectionEstimator () : MAX_POS (2)
{
  NS_LOG_FUNCTION_NOARGS ();
}

DirectionEstimator::DirectionEstimator (Time t) : MAX_POS (2)
{
  NS_LOG_FUNCTION_NOARGS ();
  m_index = 0;
  m_t = t;
  DirectionEstimator::InitializeDirectionEstimator ();
}

DirectionEstimator::~DirectionEstimator ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void
DirectionEstimator::InitializeDirectionEstimator ()
{
  Simulator::Schedule (m_t, &DirectionEstimator::UpdatePositions, this);
}

void
DirectionEstimator::UpdatePositions ()
{
  Ptr<DirectionEstimator>  est = Ptr<DirectionEstimator> (this);
  Ptr<Node> node = est -> GetObject<Node> ();
  Ptr<MobilityModel> mob = node -> GetObject<MobilityModel> ();

  Vector pos = mob -> GetPosition ();

  if (m_positions.size () >= MAX_POS)
  {
    m_positions.pop ();
  }

  m_positions.push (pos);

  Simulator::Schedule (m_t, &DirectionEstimator::UpdatePositions, this);

}

Vector
DirectionEstimator::GetEstimatedVelocity ()
{
  if (m_positions.size () < MAX_POS )
    return Vector (0,0,0);

  std::queue<Vector> cpy = m_positions;

  Vector x1 = cpy.front ();
  cpy.pop ();
  Vector x2 = cpy.front ();
  cpy.pop ();

  return x2 - x1;
}

Vector
DirectionEstimator::GetEstimatedPosition () //Position at next time step based on collected data
{
  Ptr<DirectionEstimator>  est = Ptr<DirectionEstimator> (this);
  Ptr<Node> node = est -> GetObject<Node> ();
  Ptr<MobilityModel> mob = node -> GetObject<MobilityModel> ();

  Vector pos = mob -> GetPosition ();
  Vector vel = GetEstimatedVelocity ();

  Vector nextPos;

  nextPos.x = pos.x + vel.x;
  nextPos.y = pos.y + vel.y;

  return nextPos; 

}

}