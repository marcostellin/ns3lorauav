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
#include "virtual-springs-qos-2d-mobility-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/enum.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/wifi-phy.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/node-list.h"
#include <cmath>
#include <algorithm>
#include <queue>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("VirtualSprings2dQos");

NS_OBJECT_ENSURE_REGISTERED (VirtualSpringsQos2dMobilityModel);

TypeId
VirtualSpringsQos2dMobilityModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::VirtualSpringsQos2dMobilityModel")
    .SetParent<MobilityModel> ()
    .SetGroupName ("Mobility")
    .AddConstructor<VirtualSpringsQos2dMobilityModel> ()

    .AddAttribute ("Time",
                   "Change current direction and speed after moving for this delay.",
                   TimeValue (Seconds (1.0)),
                   MakeTimeAccessor (&VirtualSpringsQos2dMobilityModel::m_modeTime),
                   MakeTimeChecker ())

    .AddAttribute ("Speed",
                   "The speed of the aerial nodes",
                   DoubleValue (17.0),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_speed),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("Tolerance",
                   "Minimum force to cause displacement",
                   DoubleValue (30.0),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_tol),
                   MakeDoubleChecker<double> ())
                   
                   
    .AddAttribute ("kAta",
                   "Stifness of AtA springs",
                   DoubleValue (0.5),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_kAta),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("txPowerGNodes",
                   "Transmission Power of Ground Nodes (dbm)",
                   DoubleValue (14),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_txPowGNodes),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("txPowerANodes",
                   "Transmission Power of Aerial Nodes (dbm)",
                   DoubleValue (16.0206),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_txPowANodes),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("lbReqAta",
                   "Required Link Budget for AtA links",
                   DoubleValue (15),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_lbReqAta),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("lbReqAtg",
                   "Required Link Budget for AtG links",
                   DoubleValue (15),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_lbReqAtg),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("Decay",
                   "DecayExponent",
                   DoubleValue (0.5),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_alpha),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("FreqAta",
                   "Frequency of AtA links",
                   DoubleValue (2.4e09),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_freq),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("MinLossAta",
                   "Minimum Loss of AtA links",
                   DoubleValue (0),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_minLoss),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("SysLossAta",
                   "System Loss of AtA links",
                   DoubleValue (1),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_sysLoss),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("PathLoss",
                   "Path loss of AtG links",
                   DoubleValue (5.2),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_pathExp),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("RefLoss",
                   "Reference loss of AtG links",
                   DoubleValue (31.23),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_refLoss),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("RefDist",
                   "Reference distance of AtG links",
                   DoubleValue (1),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_refDist),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("SensAta",
                   "Sensibility of AtA links (dbm)",
                   DoubleValue (-91),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_sensAta),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("SensAtg",
                   "Sensibility of AtA links (dbm)",
                   DoubleValue (-130),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_sensAtg),
                   MakeDoubleChecker<double> ())
                   
    .AddAttribute ("TxRangeAta",
                   "Transmission Range of AtA links",
                   DoubleValue (300),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_rangeAta),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("TxRangeAtg",
                   "Transmission Range of AtG links",
                   DoubleValue (100),
                   MakeDoubleAccessor (&VirtualSpringsQos2dMobilityModel::m_rangeAtg),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("BsPosition", "Position of BS",
                   VectorValue (Vector (200.0, 200.0, 0.0)), // ignored initial value.  
                   MakeVectorAccessor (&VirtualSpringsQos2dMobilityModel::m_bsPos),
                   MakeVectorChecker ());         
                   
                   
  return tid;
}

void
VirtualSpringsQos2dMobilityModel::DoInitialize (void)
{
	m_l0Ata = 150;
	m_l0Atg = 50;

  DoInitializePrivate ();
  MobilityModel::DoInitialize ();
}

void
VirtualSpringsQos2dMobilityModel::DoInitializePrivate (void)
{
  m_helper.Update();
  double speed = m_speed;
  //Vector myPos = m_helper.GetCurrentPosition();

  //NS_LOG_DEBUG(Simulator::Now());
  
  //Compute the direction of the movement
  Vector forceAta = VirtualSpringsQos2dMobilityModel::ComputeAtaForce();
  Vector forceAtg = VirtualSpringsQos2dMobilityModel::ComputeAtgForce();
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

    if (VirtualSpringsQos2dMobilityModel::HasPathToBs (nextPos))
    {
      m_helper.SetVelocity(vector);
      m_helper.Unpause();
    } 
    else 
    {
      NS_LOG_DEBUG ("NOT IN RANGE");
      m_helper.Pause ();
    }
 
  }
  else 
  {
    m_helper.Pause();
  }

  Time delayLeft = m_modeTime;

  DoWalk (delayLeft);
  
}

bool
VirtualSpringsQos2dMobilityModel::HasPathToBs (Vector myPos)
{
  //Apply DFS to find any path to the BS
  int visited [m_ataNodes.size()] = {};
  int inQueue [m_ataNodes.size()] = {};
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

    if (dist > 0  && dist < m_rangeAta && !visited[j] && !inQueue[j])
    {   
      toVisit.push(j);
      inQueue[j] = 1;
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

        if (dist > 0  && dist < m_rangeAta && !visited[j] &&!inQueue[j])
        {   
          toVisit.push(j);
          inQueue[j] = 1;
          NS_LOG_DEBUG("Insert in queue " << j);
        }
      }
    }

     visited[cur] = 1;
   }

   return false;

}

double
VirtualSpringsQos2dMobilityModel::GetRxPowerAta (double distance)
{
	double lambda = 299792458.0/m_freq;
	//NS_LOG_DEBUG (lambda);
	if (distance < 3*lambda)
  {
    NS_LOG_WARN ("distance not within the far field region => inaccurate propagation loss value");
  }
  if (distance <= 0)
  {
    return m_txPowANodes - m_minLoss;
  }
  
  double numerator = lambda * lambda;
  double denominator = 16 * M_PI * M_PI * distance * distance * m_sysLoss;
  double lossDb = -10 * log10 (numerator / denominator);
  //NS_LOG_DEBUG ("distance=" << distance<< "m, loss=" << lossDb <<"dB");
  
  return m_txPowANodes - std::max (lossDb, m_minLoss);
}

double
VirtualSpringsQos2dMobilityModel::GetRxPowerAtg (double distance)
{
	if (distance <= m_refDist)
  {
    return m_txPowGNodes - m_refLoss;
  }
  
  double pathLossDb = 10 * m_pathExp * std::log10 (distance / m_refDist);
  double rxc = -m_refLoss - pathLossDb;
  NS_LOG_DEBUG ("distance="<<distance<<"m, reference-attenuation="<< -m_refLoss<<"dB, "<<
                "attenuation coefficient="<<rxc<<"db");
  
  return m_txPowGNodes + rxc;
}


Vector
VirtualSpringsQos2dMobilityModel::ComputeAtaForce()
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
    double rxPower = GetRxPowerAta (dist);

    if (rxPower > m_sensAta)
    {   
    		double lb = rxPower - m_sensAta;
    		//double base = std::max (m_lbReqAta, lb) / std::min(m_lbReqAta, lb);
        //double disp = std::pow (base ,m_alpha) - 1;
        double disp = m_lbReqAta - lb;
        Vector diff = otherPos - myPos;
        double force = m_kAta*disp;

        //NS_LOG_DEBUG ("Lb " << lb << " Base " << base << " Disp " << disp << " Force " << force);
        double k = force/std::sqrt(diff.x*diff.x + diff.y*diff.y);

        fx += k*diff.x;
        fy += k*diff.y;
    }


   }

   //NS_LOG_DEBUG (fx << "," << fy);
  

  //NS_LOG_DEBUG("ATA_FORCE=" << Vector(fx,fy,0));

  return Vector(fx,fy,0);
}

Vector
VirtualSpringsQos2dMobilityModel::ComputeAtgForce()
{
  Vector myPos = m_helper.GetCurrentPosition ();
  myPos.z = 0;

  double kAtg = VirtualSpringsQos2dMobilityModel::ComputeKatg();

  double fx = 0;
  double fy = 0;

  for (uint16_t j = 0; j < m_atgNodes.size(); j++)
  {
    Ptr<Node> node = NodeList::GetNode(m_atgNodes[j]);
    Ptr<MobilityModel> mob = node -> GetObject<MobilityModel>();
    Vector pos = mob -> GetPosition();
    pos.z = 0;

    double dist = CalculateDistance(myPos, pos);
    double rxPower = GetRxPowerAtg (dist);
    //NS_LOG_DEBUG("atgNodePos=" << pos << " distace= " << dist);

    if (rxPower > m_sensAtg)
    {   

        double lb = rxPower - m_sensAtg;
    		//double base = std::max (m_lbReqAta, lb) / std::min(m_lbReqAta, lb);
        //double disp = std::pow (base ,m_alpha) - 1;
        double disp = m_lbReqAtg - lb;
        Vector diff = pos - myPos;
        double force = kAtg*disp;

        //NS_LOG_DEBUG ("Lb " << lb << " Base " << base << " Disp " << disp << " Force " << force);
        double k = force/std::sqrt(diff.x*diff.x + diff.y*diff.y);

        fx += k*diff.x;
        fy += k*diff.y;
    }
   }

   //NS_LOG_DEBUG("ATG_FORCE=" << Vector(fx,fy,0));

   return Vector(fx,fy,0);

}

int
VirtualSpringsQos2dMobilityModel::GetMaxNodesNeighbours()
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
      int numAtgNodes = VirtualSpringsQos2dMobilityModel::ComputeNumGroudNodes(ataNode);

      if (numAtgNodes > curMax){
        curMax = numAtgNodes;
      }
    }
  }

  //NS_LOG_DEBUG("Max nodes of neighbours = " << curMax);

  return curMax;
}

double
VirtualSpringsQos2dMobilityModel::ComputeKatg()
{
  int coveredNodes = VirtualSpringsQos2dMobilityModel::ComputeNumGroudNodes();
  int maxNodesNeighbours = VirtualSpringsQos2dMobilityModel::GetMaxNodesNeighbours();

  if (!maxNodesNeighbours)
  {
    //NS_LOG_DEBUG("kAtg = 1" );
    return 1;
  }

  //NS_LOG_DEBUG("kAtg = " << coveredNodes/maxNodesNeighbours);

  return coveredNodes/maxNodesNeighbours;
}

int
VirtualSpringsQos2dMobilityModel::ComputeNumGroudNodes(Ptr<Node> node )
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
VirtualSpringsQos2dMobilityModel::ComputeNumGroudNodes()
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
VirtualSpringsQos2dMobilityModel::AddAtaNode(uint32_t id)
{
  m_ataNodes.push_back(id); 
}

// void
// VirtualSpringsQos2dMobilityModel::SetAtgNodes(std::vector<Ptr<Node>> nodes)
// {
//   m_atgNodes = nodes;
// }

void
VirtualSpringsQos2dMobilityModel::AddAtgNode(uint32_t id)
{
  m_atgNodes.push_back(id); 
}


void
VirtualSpringsQos2dMobilityModel::DoWalk (Time delayLeft)
{
  m_event.Cancel ();

  m_event = Simulator::Schedule (delayLeft, &VirtualSpringsQos2dMobilityModel::DoInitializePrivate, this);

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
      m_event = Simulator::Schedule (delayLeft, &VirtualSpringsQos2dMobilityModel::DoInitializePrivate, this);
    }
  else
    {
      nextPosition = m_bounds.CalculateIntersection (position, speed);
      Time delay = Seconds ((nextPosition.x - position.x) / speed.x);
      m_event = Simulator::Schedule (delay, &VirtualSpringsQos2dMobilityModel::Rebound, this,
                                     delayLeft - delay);
    }
  NotifyCourseChange ();
  */
}

/*
void
VirtualSpringsQos2dMobilityModel::Rebound (Time delayLeft)
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
VirtualSpringsQos2dMobilityModel::DoDispose (void)
{
  // chain up
  MobilityModel::DoDispose ();
}
Vector
VirtualSpringsQos2dMobilityModel::DoGetPosition (void) const
{
  m_helper.Update();
  return m_helper.GetCurrentPosition ();	
  /*
  m_helper.UpdateWithBounds (m_bounds);
  return m_helper.GetCurrentPosition ();
  */
}
void
VirtualSpringsQos2dMobilityModel::DoSetPosition (const Vector &position)
{ 
  m_helper.SetPosition (position);
  Simulator::Remove (m_event);
  m_event = Simulator::ScheduleNow (&VirtualSpringsQos2dMobilityModel::DoInitializePrivate, this);

  /*
  NS_ASSERT (m_bounds.IsInside (position));
  m_helper.SetPosition (position);
  Simulator::Remove (m_event);
  m_event = Simulator::ScheduleNow (&VirtualSpringsQos2dMobilityModel::DoInitializePrivate, this);
  */
}
Vector
VirtualSpringsQos2dMobilityModel::DoGetVelocity (void) const
{
  
  return m_helper.GetVelocity ();
  
}



} // namespace ns3
