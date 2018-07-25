/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006,2007 INRIA
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
#include "virtual-springs-2d-mobility-model.h"
#include "ns3/enum.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/node-list.h"
#include <cmath>
#include <algorithm>
#include <queue>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("VirtualSprings2d");

NS_OBJECT_ENSURE_REGISTERED (VirtualSprings2dMobilityModel);

TypeId
VirtualSprings2dMobilityModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::VirtualSprings2dMobilityModel")
    .SetParent<MobilityModel> ()
    .SetGroupName ("Mobility")
    .AddConstructor<VirtualSprings2dMobilityModel> ()

    .AddAttribute ("Time",
                   "Change current direction and speed after moving for this delay.",
                   TimeValue (Seconds (1.0)),
                   MakeTimeAccessor (&VirtualSprings2dMobilityModel::m_modeTime),
                   MakeTimeChecker ())

    .AddAttribute ("Speed",
                   "The speed of the aerial nodes",
                   DoubleValue (17.0),
                   MakeDoubleAccessor (&VirtualSprings2dMobilityModel::m_speed),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("Tolerance",
                   "Minimum force to cause displacement",
                   DoubleValue (30.0),
                   MakeDoubleAccessor (&VirtualSprings2dMobilityModel::m_tol),
                   MakeDoubleChecker<double> ())
                   
                   
    .AddAttribute ("kAta",
                   "Stifness of AtA springs",
                   DoubleValue (0.5),
                   MakeDoubleAccessor (&VirtualSprings2dMobilityModel::m_kAta),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("l0Ata",
                   "Natural length of AtA springs",
                   DoubleValue (150),
                   MakeDoubleAccessor (&VirtualSprings2dMobilityModel::m_l0Ata),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("l0Atg",
                   "Natural length of AtG springs",
                   DoubleValue (20),
                   MakeDoubleAccessor (&VirtualSprings2dMobilityModel::m_l0Atg),
                   MakeDoubleChecker<double> ())
    
                   
    .AddAttribute ("TxRangeAta",
                   "Transmission Range of AtA links",
                   DoubleValue (300),
                   MakeDoubleAccessor (&VirtualSprings2dMobilityModel::m_rangeAta),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("TxRangeAtg",
                   "Transmission Range of AtG links",
                   DoubleValue (100),
                   MakeDoubleAccessor (&VirtualSprings2dMobilityModel::m_rangeAtg),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("BsPosition", "Position of BS",
                       VectorValue (Vector (2000.0, 2000.0, 0.0)), // ignored initial value.
                       MakeVectorAccessor (&VirtualSprings2dMobilityModel::m_bsPos),
                       MakeVectorChecker ());         
                   
                   
  return tid;
}

void
VirtualSprings2dMobilityModel::DoInitialize (void)
{
  DoInitializePrivate ();
  MobilityModel::DoInitialize ();
}

void
VirtualSprings2dMobilityModel::DoInitializePrivate (void)
{
  m_helper.Update();
  double speed = m_speed;
  //Vector myPos = m_helper.GetCurrentPosition();

  //NS_LOG_DEBUG(Simulator::Now());
  
  //Compute the direction of the movement
  Vector forceAta = VirtualSprings2dMobilityModel::ComputeAtaForce();
  Vector forceAtg = VirtualSprings2dMobilityModel::ComputeAtgForce();
  Vector force = forceAta + forceAtg;

  //NS_LOG_DEBUG("ForceMod = " << force.GetLength());

  double k = speed/std::sqrt(force.x*force.x + force.y*force.y);
  Vector vector (k*force.x,
  				 k*force.y,
  				 0.0);

  //NS_LOG_DEBUG("Velocity = " << vector);

  if (force.GetLength () > m_tol)
  {
    //Compute the future position based on current velocity and direction
    Vector myPos = m_helper.GetCurrentPosition();
    Vector nextPos;
    nextPos.x = myPos.x + vector.x * m_modeTime.GetSeconds ();
    nextPos.y = myPos.y + vector.y * m_modeTime.GetSeconds ();

    m_helper.SetVelocity(vector);
    m_helper.Unpause();

    // if (VirtualSprings2dMobilityModel::HasPathToBs (nextPos))
    // {
    //   m_helper.SetVelocity(vector);
    //   m_helper.Unpause();
    // } 
    // else 
    // {
    //   NS_LOG_DEBUG ("NOT IN RANGE");
    //   m_helper.Pause ();
    // }
 
  }
  else 
  {
    m_helper.Pause();
  }

  Time delayLeft = m_modeTime;

  DoWalk (delayLeft);
  
}

bool
VirtualSprings2dMobilityModel::HasPathToBs (Vector myPos)
{
  //Apply DFS to find any path to the BS
  int visited [m_ataNodes.size()] = {};
  std::queue<int> toVisit;

  if (CalculateDistance(myPos, m_bsPos) < m_rangeAta)
    return true;

  for (uint16_t j = 0; j < m_ataNodes.size(); j++)
  {
    Ptr<Node> node = NodeList::GetNode(m_ataNodes[j]);
    Ptr<MobilityModel> otherMob = node -> GetObject<MobilityModel>();
    Vector otherPos = otherMob -> GetPosition();
    otherPos.z = 0;
    double dist = CalculateDistance(myPos, otherPos);
    //NS_LOG_DEBUG("otherPos=" << otherPos << " distace= " << dist);

    if (dist > 0  && dist < m_rangeAta && !visited[j])
    {   
      toVisit.push(j);
      NS_LOG_DEBUG("Insert in queue " << j);
    }
   }

   int cur;
   while (!toVisit.empty()) 
   { 
     cur = toVisit.front();
     toVisit.pop();
     NS_LOG_DEBUG ("Currently visiting " << cur);

     Ptr<Node> node = NodeList::GetNode(m_ataNodes[cur]);
     Ptr<MobilityModel> mob = node -> GetObject<MobilityModel>();
     Vector pos = mob -> GetPosition();
     pos.z = 0;

    if (CalculateDistance(pos, m_bsPos) < m_rangeAta)
      return true;

    for (uint16_t j = 0; j < m_ataNodes.size(); j++)
    {
      if (j != cur)
      {
        Ptr<Node> node2 = NodeList::GetNode(m_ataNodes[j]);
        Ptr<MobilityModel> otherMob = node2 -> GetObject<MobilityModel>();
        Vector otherPos = otherMob -> GetPosition();
        otherPos.z = 0;
        double dist = CalculateDistance(pos, otherPos);

        if (dist > 0  && dist < m_rangeAta && !visited[j])
        {   
          toVisit.push(j);
          NS_LOG_DEBUG("Insert in queue " << j);
        }
      }
    }

     visited[cur] = 1;
   }

   return false;

}


Vector
VirtualSprings2dMobilityModel::ComputeAtaForce()
{
  Vector myPos = m_helper.GetCurrentPosition ();
  //NS_LOG_DEBUG("myPos=" << myPos);

  double fx = 0;
  double fy = 0;
  
  for (uint16_t j = 0; j < m_ataNodes.size(); j++)
  {
    Ptr<Node> node = NodeList::GetNode(m_ataNodes[j]);
    Ptr<MobilityModel> otherMob = node -> GetObject<MobilityModel>();
    Vector otherPos = otherMob -> GetPosition();
    double dist = CalculateDistance(myPos, otherPos);
    //NS_LOG_DEBUG("otherPos=" << otherPos << " distace= " << dist);

    if (dist > 0  && dist < m_rangeAta)
    {   

        double disp = dist - m_l0Ata;
        Vector diff = otherPos - myPos;
        double force = m_kAta*disp;
        double k = force/std::sqrt(diff.x*diff.x + diff.y*diff.y);

        fx += k*diff.x;
        fy += k*diff.y;

    }
   }
  

  //NS_LOG_DEBUG("ATA_FORCE=" << Vector(fx,fy,0));

  return Vector(fx,fy,0);
}

Vector
VirtualSprings2dMobilityModel::ComputeAtgForce()
{
  Vector myPos = m_helper.GetCurrentPosition ();
  myPos.z = 0;

  double kAtg = VirtualSprings2dMobilityModel::ComputeKatg();

  double fx = 0;
  double fy = 0;

  for (uint16_t j = 0; j < m_atgNodes.size(); j++)
  {
    Ptr<Node> node = NodeList::GetNode(m_atgNodes[j]);
    Ptr<MobilityModel> mob = node -> GetObject<MobilityModel>();
    Vector pos = mob -> GetPosition();
    pos.z = 0;

    double dist = CalculateDistance(myPos, pos);
    //NS_LOG_DEBUG("atgNodePos=" << pos << " distace= " << dist);

    if (dist < m_rangeAtg)
    {   

        double disp = dist - m_l0Atg;
        Vector diff = pos - myPos;
        double force = kAtg*disp;
        double k = force/std::sqrt(diff.x*diff.x + diff.y*diff.y);

        fx += k*diff.x;
        fy += k*diff.y;

        // double theta = difference.y / difference.x;
        // fx += -m_kAta*disp*cos(theta);
    }
   }

   //NS_LOG_DEBUG("ATG_FORCE=" << Vector(fx,fy,0));

   return Vector(fx,fy,0);

}

int
VirtualSprings2dMobilityModel::GetMaxNodesNeighbours()
{
  Vector myPos = m_helper.GetCurrentPosition ();
  myPos.z = 0;

  int curMax = 0;
  for (uint16_t j = 0; j < m_ataNodes.size(); j++)
  {
    Ptr<Node> ataNode = NodeList::GetNode(m_ataNodes[j]);
    Ptr<MobilityModel> ataNodeMob = ataNode -> GetObject<MobilityModel>();
    Vector ataNodePos = ataNodeMob -> GetPosition();
    ataNodePos.z = 0;

    double dist = CalculateDistance(myPos, ataNodePos);

    if (dist < m_rangeAta)
    {
      int numAtgNodes = VirtualSprings2dMobilityModel::ComputeNumGroudNodes(ataNode);

      if (numAtgNodes > curMax){
        curMax = numAtgNodes;
      }
    }
  }

  //NS_LOG_DEBUG("Max nodes of neighbours = " << curMax);

  return curMax;
}

double
VirtualSprings2dMobilityModel::ComputeKatg()
{
  int coveredNodes = VirtualSprings2dMobilityModel::ComputeNumGroudNodes();
  int maxNodesNeighbours = VirtualSprings2dMobilityModel::GetMaxNodesNeighbours();

  if (!maxNodesNeighbours)
  {
    //NS_LOG_DEBUG("kAtg = 1" );
    return 1;
  }

  //NS_LOG_DEBUG("kAtg = " << coveredNodes/maxNodesNeighbours);

  return coveredNodes/maxNodesNeighbours;
}

int
VirtualSprings2dMobilityModel::ComputeNumGroudNodes(Ptr<Node> node )
{
  Ptr<MobilityModel> mob = node -> GetObject<MobilityModel>();
  Vector pos = mob -> GetPosition();
  pos.z = 0;

  int counter = 0;
  for (uint16_t j = 0; j < m_atgNodes.size(); j++)
  {
    Ptr<Node> atgNode = NodeList::GetNode(m_atgNodes[j]);
    Ptr<MobilityModel> atgNodeMob = atgNode -> GetObject<MobilityModel>();
    Vector atgNodePos = atgNodeMob -> GetPosition();
    atgNodePos.z = 0;
    double dist = CalculateDistance(pos, atgNodePos);

    if (dist < m_rangeAtg)
    {
      counter++;
    }
  }

  //NS_LOG_DEBUG("Nodes covered=" << counter);

  return counter;
}

int
VirtualSprings2dMobilityModel::ComputeNumGroudNodes()
{
  Vector myPos = m_helper.GetCurrentPosition ();
  myPos.z = 0;

  int counter = 0;

  for (uint16_t j = 0; j < m_atgNodes.size(); j++)
  {
    Ptr<Node> node = NodeList::GetNode(m_atgNodes[j]);
    Ptr<MobilityModel> mob = node -> GetObject<MobilityModel>();
    Vector pos = mob -> GetPosition();
    pos.z = 0;
    double dist = CalculateDistance(myPos, pos);

    if (dist < m_rangeAtg)
    {
      counter++;
    }
  }

  //NS_LOG_DEBUG("Nodes covered=" << counter);

  return counter;

}

void
VirtualSprings2dMobilityModel::AddAtaNode(uint32_t id)
{
  m_ataNodes.push_back(id); 
}

// void
// VirtualSprings2dMobilityModel::SetAtgNodes(std::vector<Ptr<Node>> nodes)
// {
//   m_atgNodes = nodes;
// }

void
VirtualSprings2dMobilityModel::AddAtgNode(uint32_t id)
{
  m_atgNodes.push_back(id); 
}


void
VirtualSprings2dMobilityModel::DoWalk (Time delayLeft)
{
  m_event.Cancel ();

  m_event = Simulator::Schedule (delayLeft, &VirtualSprings2dMobilityModel::DoInitializePrivate, this);

  NotifyCourseChange ();

  /*
  Vector position = m_helper.GetCurrentPosition ();
  Vector speed = m_helper.GetVelocity ();
  Vector nextPosition = position;
  nextPosition.x += speed.x * delayLeft.GetSeconds ();
  nextPosition.y += speed.y * delayLeft.GetSeconds ();
  m_event.Cancel ();
  if (m_bounds.IsInside (nextPosition))
    {
      m_event = Simulator::Schedule (delayLeft, &VirtualSprings2dMobilityModel::DoInitializePrivate, this);
    }
  else
    {
      nextPosition = m_bounds.CalculateIntersection (position, speed);
      Time delay = Seconds ((nextPosition.x - position.x) / speed.x);
      m_event = Simulator::Schedule (delay, &VirtualSprings2dMobilityModel::Rebound, this,
                                     delayLeft - delay);
    }
  NotifyCourseChange ();
  */
}

/*
void
VirtualSprings2dMobilityModel::Rebound (Time delayLeft)
{
  m_helper.UpdateWithBounds (m_bounds);
  Vector position = m_helper.GetCurrentPosition ();
  Vector speed = m_helper.GetVelocity ();
  switch (m_bounds.GetClosestSide (position))
    {
    case Rectangle::RIGHT:
    case Rectangle::LEFT:
      speed.x = -speed.x;
      break;
    case Rectangle::TOP:
    case Rectangle::BOTTOM:
      speed.y = -speed.y;
      break;
    }
  m_helper.SetVelocity (speed);
  m_helper.Unpause ();
  DoWalk (delayLeft);
  
}

*/

void
VirtualSprings2dMobilityModel::DoDispose (void)
{
  // chain up
  MobilityModel::DoDispose ();
}
Vector
VirtualSprings2dMobilityModel::DoGetPosition (void) const
{
  m_helper.Update();
  return m_helper.GetCurrentPosition ();	
  /*
  m_helper.UpdateWithBounds (m_bounds);
  return m_helper.GetCurrentPosition ();
  */
}
void
VirtualSprings2dMobilityModel::DoSetPosition (const Vector &position)
{ 
  m_helper.SetPosition (position);
  Simulator::Remove (m_event);
  m_event = Simulator::ScheduleNow (&VirtualSprings2dMobilityModel::DoInitializePrivate, this);

  /*
  NS_ASSERT (m_bounds.IsInside (position));
  m_helper.SetPosition (position);
  Simulator::Remove (m_event);
  m_event = Simulator::ScheduleNow (&VirtualSprings2dMobilityModel::DoInitializePrivate, this);
  */
}
Vector
VirtualSprings2dMobilityModel::DoGetVelocity (void) const
{
  
  return m_helper.GetVelocity ();
  
}



} // namespace ns3
