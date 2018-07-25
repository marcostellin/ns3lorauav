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
#include "ns3/constant-position-mobility-model.h"
#include "ns3/enum.h"
#include "ns3/double.h"
#include "ns3/integer.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/node-list.h"
#include "ns3/boolean.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/lora-net-device.h"
#include "ns3/gateway-lora-phy.h"
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

    .AddAttribute ("LinkBudgetAta",
                   "Required link budget for AtA links",
                   DoubleValue (10.0),
                   MakeDoubleAccessor (&VirtualSprings2dMobilityModel::m_lbReqAta),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("LinkBudgetAtg",
                   "Required link budget for AtG links",
                   DoubleValue (10.0),
                   MakeDoubleAccessor (&VirtualSprings2dMobilityModel::m_lbReqAtg),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("PredictionMode",
                   "0=No prediction, 1=simple prediction, 2=direction clustering, 3=kmeans clustering",
                   IntegerValue (0),
                   MakeIntegerAccessor (&VirtualSprings2dMobilityModel::m_predictionMode),
                   MakeIntegerChecker<uint16_t> ())

    .AddAttribute ("KMode",
                   "0=Di Felice, 1=LoRaUAV",
                   IntegerValue (0),
                   MakeIntegerAccessor (&VirtualSprings2dMobilityModel::m_kMode),
                   MakeIntegerChecker<uint16_t> ())

    .AddAttribute ("ConnMode",
                   "0=Not Active, 1=Active",
                   IntegerValue (0),
                   MakeIntegerAccessor (&VirtualSprings2dMobilityModel::m_connMode),
                   MakeIntegerChecker<uint16_t> ())

    .AddAttribute ("BsPosition", "Position of BS",
                    VectorValue (Vector (200.0, 200.0, 0.0)), 
                    MakeVectorAccessor (&VirtualSprings2dMobilityModel::m_bsPos),
                    MakeVectorChecker ())

    .AddAttribute ("BsAddress", "IP Address of the base station",
                    Ipv4AddressValue("10.1.1.1"),
                    MakeIpv4AddressAccessor (&VirtualSprings2dMobilityModel::m_bsAddr),
                    MakeIpv4AddressChecker ())

    .AddAttribute ("KatgPlusMode", "K_ATG boost option",
                    BooleanValue (true), // ignored initial value.
                    MakeBooleanAccessor (&VirtualSprings2dMobilityModel::m_kAtgPlusMode),
                    MakeBooleanChecker ())

    .AddTraceSource ("NodesInRange",
                     "Nodes in range have changed",
                      MakeTraceSourceAccessor (&VirtualSprings2dMobilityModel::m_nodesInRangeTrace),
                     "ns3::VirtualSprings2dMobilityModel::TracedCallback")

    .AddTraceSource ("ConnectionState",
                     "Fired when connection to BS lost",
                      MakeTraceSourceAccessor (&VirtualSprings2dMobilityModel::m_disconnectedTimeTrace),
                     "ns3::VirtualSprings2dMobilityModel::TracedCallback");           
                   
                   
  return tid;
}

VirtualSprings2dMobilityModel::VirtualSprings2dMobilityModel () : M_PERC_SHARED(0.5), M_SEC_RETAIN_EDS (20.0), M_MAX_PAUSE (30.0), M_KATG_PLUS (20), M_SEC_CHECK_CONN (1.0)
{
  NS_LOG_FUNCTION (this);
}

void
VirtualSprings2dMobilityModel::CheckConnectivity ()
{
  olsr::RoutingTableEntry entry = VirtualSprings2dMobilityModel::HasPathToBs();

  if (entry.distance == 0)
    m_disconnectedTimeTrace (m_id, Seconds(M_SEC_CHECK_CONN));

  Simulator::Schedule (Seconds(M_SEC_CHECK_CONN), &VirtualSprings2dMobilityModel::CheckConnectivity, this);

}

void
VirtualSprings2dMobilityModel::CheckCoverage ()
{
  Ptr<VirtualSprings2dMobilityModel> mob = Ptr<VirtualSprings2dMobilityModel> (this);
  //Vector pos = mob -> GetPosition ();

  std::vector<int> nodeList;
  for (uint16_t i = 0; i < m_atgNodes.size (); i++)
  {
    Ptr<Node> ed = NodeList::GetNode (m_atgNodes[i]);
    Ptr<MobilityModel> oMob = ed -> GetObject<MobilityModel> ();
    //Vector oPos = oMob -> GetPosition ();

    double lb = m_estimator -> GetAtgLinkBudget ( mob, oMob);

    if (lb > 0)
      nodeList.push_back (m_atgNodes[i]); 
  }  

  m_nodesInRangeTrace (nodeList);

  Simulator::Schedule (Seconds (1), &VirtualSprings2dMobilityModel::CheckCoverage, this);
}

void
VirtualSprings2dMobilityModel::UpdateHistory ()
{
  m_monitor -> UpdateHistory (Seconds(M_SEC_RETAIN_EDS));
  Simulator::Schedule (Seconds (10), &VirtualSprings2dMobilityModel::UpdateHistory, this);
}

void
VirtualSprings2dMobilityModel::DoInitialize (void)
{
  //Initialize private variables
  m_pause = 0;
  m_hops = 0;
  m_persist = 0;
  m_rangeApprox = 0;
  m_lastNode = false;
  m_detach = false;

  Ptr<VirtualSprings2dMobilityModel> mob = Ptr<VirtualSprings2dMobilityModel> (this);
  Ptr<Node> node = mob -> GetObject<Node> ();

  //Set Node Id
  m_id = node -> GetId ();

  VirtualSprings2dMobilityModel::InitializeMonitors ();

  //Schedule Recurrent tasks
  Simulator::ScheduleNow (&VirtualSprings2dMobilityModel::CheckConnectivity, this);
  Simulator::Schedule (Seconds (10), &VirtualSprings2dMobilityModel::UpdateHistory, this);

  if (m_predictionMode == 1)
    Simulator::Schedule (Seconds (10), &VirtualSprings2dMobilityModel::UpdateHistory, this);

  Simulator::Schedule (Seconds (1), &VirtualSprings2dMobilityModel::CheckCoverage, this);

  DoInitializePrivate ();
  MobilityModel::DoInitialize ();
}

void
VirtualSprings2dMobilityModel::DoInitializePrivate (void)
{
  m_helper.Update();

  //Update list of neighbouring UAVs
  VirtualSprings2dMobilityModel::SetNeighboursList ();

  //Update list of neighbouring EDs
  VirtualSprings2dMobilityModel::SetEdsList ();

  //Update load
  m_load = m_loadMonitor -> GetLoad ();

  //Check for lost clusters
  if (m_predictionMode == 2)
  {
    m_monitor -> UpdateLostEds (Seconds(65));
    m_monitor -> FilterLostEds (Seconds (40), Minutes (3) );
    std::vector<ClusterInfo> clusters = m_monitor -> GetClusterInfo ();

    std::vector<Token> tokens = GenerateTokens (clusters);

    for (uint16_t i = 0; i < tokens.size (); i++)
    {
      SendToken (tokens[i]);
      m_manager -> AddToken (tokens[i]);
    }
  }

  if (m_predictionMode == 3)
  {
    uint16_t maxClusters = 3;
    m_monitor -> UpdateLostEds (Minutes(3));
    m_monitor -> FilterLostEds (Seconds (40), Minutes (3) );

    uint32_t len = m_monitor -> GetEdsLostSize ();
    point pt = (point) malloc(sizeof(point_t)*len);

    uint16_t numClust = m_monitor -> CreateClusters (pt, len, maxClusters);

    std::vector<ClusterInfo> clusters = m_monitor -> ComputeClustersInfo (pt, len, numClust);

    std::vector<Token> tokens = GenerateTokens (clusters);

    for (uint16_t i = 0; i < tokens.size (); i++)
    {
      SendToken (tokens[i]);
      m_manager -> AddToken (tokens[i]);
    }

    free(pt);
  }

  olsr::RoutingTableEntry entry = VirtualSprings2dMobilityModel::HasPathToBs();
    
  //If a path to the BS exists
  if (m_connMode == 1 && entry.distance != 0) 
  {
    m_hops = entry.distance;
    VirtualSprings2dMobilityModel::UpdateRangeApprox ();
    double rangeRatio = GetDistanceFromBs () / m_rangeApprox;
    m_persist = (m_hops - 1) * (uint32_t) rangeRatio ;

    //Send the tokens
    if (m_token.expires != Seconds (0))
    {
      SendToken (m_token);
      m_manager -> AddToken (m_token);
      Token newToken;
      m_token = newToken;
    }

    //Move according to forces if m_pause intervals have passed...
    if (m_pause == 0 || m_load == 0) 
    {
      m_pause = 0;
      VirtualSprings2dMobilityModel::DoMoveForces ();
    }

    //Or don't do anything if m_pause intervals still have to pass.
    if (m_pause > 0)
    {
      m_pause --;
      m_helper.Pause ();
    }
  }

  if (m_connMode == 1 && entry.distance  == 0)
  {
    //Go back and try to recover connectivity...
    if (m_persist == 0)
    {
      //Create Token
      if (m_predictionMode == 1)
      {
        if (m_lastNode && m_token.expires == Seconds(0) && m_eds.size () > 0)
        {
          m_token = VirtualSprings2dMobilityModel::GenerateToken ();
        }
      }

      //Set the pause time
      m_pause = VirtualSprings2dMobilityModel::SetPause(m_hops);

      //Go back to BS
      VirtualSprings2dMobilityModel::GoBack ();

    }

    //Try to go on and see if connectivity is recovered after m_persist intervals...
    if (m_persist > 0)
    {
      if (!m_eds.empty ())
      {
        //Find most similar node in the neighbourhood
        uint32_t simNode = FindMostSimilarNode ();

        if (simNode == 0 || m_id > simNode)
          m_persist = m_persist / 2;
        else
          m_persist = m_persist * 2;

        if (simNode == 0)
          m_lastNode = true;
        else
          m_lastNode = false;

        NS_LOG_INFO ("Persist on route for " << m_persist << " intervals");
      }
      else
      {
        m_persist --;
      }
        
      VirtualSprings2dMobilityModel::DoMoveForces ();
    }  

  }

  if (m_connMode != 1 && entry.distance != 0)
    VirtualSprings2dMobilityModel::DoMoveForces ();

  if (m_connMode != 1 && entry.distance == 0)
    VirtualSprings2dMobilityModel::GoBack ();
 
  Time delayLeft = m_modeTime;

  DoWalk (delayLeft);
  
}

void
VirtualSprings2dMobilityModel::InitializeMonitors ()
{
  Ptr<Node> node = NodeList::GetNode (m_id);
  Ptr<NetDevice> netDevice = node -> GetDevice (0);
  Ptr<LoraNetDevice> loraNetDevice = netDevice -> GetObject<LoraNetDevice> ();
  NS_ASSERT (loraNetDevice != 0);
  Ptr<GatewayLoraPhy> gwPhy = loraNetDevice->GetPhy ()->GetObject<GatewayLoraPhy> ();
  m_monitor = Ptr<LoraEdsMonitor>( new LoraEdsMonitor (gwPhy));

  //Initialize LoadMonitor
  m_loadMonitor = CreateObject<LoadMonitor> ( node -> GetObject<Ipv4L3Protocol> () );

  m_loadMonitor -> SetLoadInterval (Minutes (3));

  //Intialize SeedManager
  m_manager = CreateObject<SeedsManager> ();
}

olsr::RoutingTableEntry
VirtualSprings2dMobilityModel::HasPathToBs ()
{
  std::vector<ns3::olsr::RoutingTableEntry> entries = m_routing -> ns3::olsr::RoutingProtocol::GetRoutingTableEntries ();

  for (uint16_t i = 0; i < entries.size(); i++)
  {
    if (entries[i].destAddr.IsEqual (m_bsAddr))
    {
      return entries[i];
    }
  }

  return olsr::RoutingTableEntry ();

}

void
VirtualSprings2dMobilityModel::UpdateRangeApprox ()
{
  double rangeApprox = GetDistanceFromFurthestNeighbour ();

  if (rangeApprox > m_rangeApprox)
    m_rangeApprox = rangeApprox;

}

void
VirtualSprings2dMobilityModel::GoBack ()
{
  NS_LOG_INFO ("Going Back!");
  Vector curPos = m_helper.GetCurrentPosition ();
  Vector diff = m_bsPos - curPos;
  double h = m_speed*2/std::sqrt(diff.x*diff.x + diff.y*diff.y);
  Vector vec (h*diff.x,
              h*diff.y,
              0.0);

  m_helper.SetVelocity(vec);
  m_helper.Unpause ();
}

void
VirtualSprings2dMobilityModel::DoMoveForces ()
{
  Vector force = VirtualSprings2dMobilityModel::ComputeTotalForce ();
  Vector vel = VirtualSprings2dMobilityModel::GetDirection (force);

  if (force.GetLength () > m_tol)
  {
    m_helper.SetVelocity(vel);
    m_helper.Unpause ();
  }
  else
  {
    m_helper.Pause ();
  }
}

Vector
VirtualSprings2dMobilityModel::ComputeTotalForce ()
{
  //Compute the direction of the movement
  Vector forceAta = VirtualSprings2dMobilityModel::ComputeAtaForce ();
  Vector forceAtg = VirtualSprings2dMobilityModel::ComputeAtgForce ();
  Vector forceSeed;

  if ( (m_eds.size () == 0 && !m_loadMonitor -> IsBusy () ))
  {
    forceSeed = VirtualSprings2dMobilityModel::ComputeSeedForce ();
  }

  Vector force = forceAta + forceAtg + forceSeed;

  NS_LOG_DEBUG ("AtG Force = " << forceAtg << ", AtA force = " << forceAta << ", Seed Force = " << forceSeed);

  return force;
}

Vector
VirtualSprings2dMobilityModel::GetDirection (Vector force)
{
  double k = m_speed/std::sqrt(force.x*force.x + force.y*force.y);
  Vector vector (k*force.x,
                 k*force.y,
                 0.0);

  return vector;
}

Token
VirtualSprings2dMobilityModel::GenerateToken ()
{
  NS_LOG_INFO ("Creating token...");
  Token token;
  token.lastPos = m_monitor -> GetLastPosition ();
  token.lastTime = m_monitor -> GetLastTime ();
  token.weight = m_eds.size ();
  token.speed = m_monitor -> GetSpeedVector ();
  token.expires = Simulator::Now () + Minutes (10);
  token.id = m_id;

  return token;
}

std::vector<Token>
VirtualSprings2dMobilityModel::GenerateTokens (std::vector<ClusterInfo> clusters)
{
  std::vector<Token> tokens;

  for (std::vector<ClusterInfo>::iterator it = clusters.begin (); it != clusters.end (); ++it)
  {
    ClusterInfo c = (*it);
    Token t;
    t.lastPos = c.lastPosition;
    t.lastTime = c.lastTime;
    t.weight = c.weight;
    t.speed = c.velocity;
    t.expires = Simulator::Now () + Minutes (10);
    t.id = m_id*10 + c.id;

    tokens.push_back (t);
    NS_LOG_LOGIC ("Generated token " << t.id << " lastPos=" << t.lastPos << " speed=" << t.speed );
  }

  return tokens;
}

uint32_t
VirtualSprings2dMobilityModel::SetPause (uint32_t hops)
{
  if (hops > 0)
  {
    double pause = M_MAX_PAUSE * m_rangeApprox / GetDistanceFromBs ();
    NS_LOG_INFO ("Stop for " << (uint32_t)pause << " intervals");
    return (uint32_t)pause;
  }

  return 0;
}


void
VirtualSprings2dMobilityModel::SetNeighboursList () //Associate immediately ip to ataNodes for faster lookup //Consider getting EdsList once and then use it at same period
{
  std::vector<ns3::olsr::RoutingTableEntry> entries = m_routing -> ns3::olsr::RoutingProtocol::GetRoutingTableEntries ();

  m_neighbours.clear ();
  m_allNeighbours.clear ();

  for (uint16_t i = 0; i < entries.size(); i++)
  {
    for (uint16_t j = 0; j < m_ataNodes.size (); j++)
    {
        Ptr<Node> node = NodeList::GetNode(m_ataNodes[j]);
        Ptr<VirtualSprings2dMobilityModel> mob = node -> GetObject<VirtualSprings2dMobilityModel> ();
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> (); 
        Ipv4Address addr = ipv4->GetAddress (1, 0).GetLocal (); 

        if (addr.IsEqual (entries[i].destAddr))
        {
          if (mob -> GetEdsList ().size () > 0 || entries[i].distance == 1)
            m_allNeighbours.push_back (node -> GetId());

          if (entries[i].distance == 1)
          {
            m_neighbours.push_back (node -> GetId ());
          }
        }

    }
  }
}

void
VirtualSprings2dMobilityModel::SendToken (Token token)
{
  std::vector<ns3::olsr::RoutingTableEntry> entries = m_routing -> ns3::olsr::RoutingProtocol::GetRoutingTableEntries ();

  for (uint16_t i = 0; i < entries.size(); i++)
  {
    for (uint16_t j = 0; j < m_ataNodes.size (); j++)
    {
      Ptr<Node> node = NodeList::GetNode(m_ataNodes[j]);
      Ptr<VirtualSprings2dMobilityModel> mob = node -> GetObject<VirtualSprings2dMobilityModel> ();
      Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> (); 
      Ipv4Address addr = ipv4->GetAddress (1, 0).GetLocal (); 

      if (addr.IsEqual (entries[i].destAddr))
      {
        mob -> ReceiveToken (token);
      }
    }
  }
}

void
VirtualSprings2dMobilityModel::ReceiveToken (Token token)
{
  m_manager -> AddToken (token);
}

void
VirtualSprings2dMobilityModel::SetEdsList ()
{
  std::map<uint32_t, EdsEntry> curNodes = m_monitor -> GetEdsList (Seconds (M_SEC_RETAIN_EDS));

  m_eds = curNodes;
}


Vector
VirtualSprings2dMobilityModel::ComputeAtaForce()
{
  Vector myPos = m_helper.GetCurrentPosition ();

  double fx = 0;
  double fy = 0;
  
  for (uint16_t j = 0; j < m_neighbours.size(); j++)
  {
    Ptr<Node> node = NodeList::GetNode(m_neighbours[j]);
    Ptr<MobilityModel> otherMob = node -> GetObject<MobilityModel>();
    Vector otherPos = otherMob -> GetPosition();

    double lb = m_estimator -> GetAtaLinkBudget ( Ptr<MobilityModel> (this), otherMob);

    double k_ata = m_kAta + m_ataNodes.size ()/m_neighbours.size () - 1;

    if ( m_kMode == 0 )
      k_ata = m_kAta;

    double disp = m_lbReqAta - lb;
    Vector diff = otherPos - myPos;
    double force = k_ata * disp;
    double k = force/std::sqrt(diff.x*diff.x + diff.y*diff.y);

    fx += k*diff.x;
    fy += k*diff.y;
  }

  return Vector(fx,fy,0);
}

Vector
VirtualSprings2dMobilityModel::ComputeAtgForce()
{
  Vector myPos = m_helper.GetCurrentPosition ();
  myPos.z = 0;

  //double kAtg = VirtualSprings2dMobilityModel::ComputeKatg();

  double fx = 0;
  double fy = 0;

  uint32_t maxCard = VirtualSprings2dMobilityModel::GetMaxCardinality ();

  NS_LOG_DEBUG ("MaxCard=" << maxCard);

  for (std::map<uint32_t, EdsEntry>::iterator it = m_eds.begin (); it != m_eds.end (); ++it)
  {

    Vector pos ( (*it).second.x,
                 (*it).second.y,
                 0
               );

    Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel> ();
    mob -> SetPosition (pos);
    double lb = m_estimator -> GetAtgLinkBudget ( Ptr<MobilityModel> (this), mob);

    //double dist = CalculateDistance(myPos, pos);

    double kAtgPlus = 1;
    
    if (m_kAtgPlusMode)
    {
      uint16_t cnt = 1;

      for (uint16_t i = 0; i < m_neighbours.size (); i++)
      {
        Ptr<Node> ataNode = NodeList::GetNode(m_neighbours[i]);
        Ptr<VirtualSprings2dMobilityModel> ataMob = ataNode -> GetObject<VirtualSprings2dMobilityModel> ();
        std::map<uint32_t, EdsEntry> otherList = ataMob -> GetEdsList ();

        //otherList.insert (m_eds.begin (), m_eds.end ());

        std::map<uint32_t, EdsEntry>::iterator oIt;
        oIt = otherList.find ( it -> first );

        if ( oIt != otherList.end () )
        {
          cnt ++;
        }
      }

      //cnt > 1 ? kAtgPlus = 1 : kAtgPlus = M_KATG_PLUS;
      kAtgPlus = (double)maxCard / (double)cnt;
    }

    if (m_kMode == 0 )
      kAtgPlus = VirtualSprings2dMobilityModel::ComputeKatg ();
    
    double disp = m_lbReqAtg - lb;
    Vector diff = pos - myPos;
    double force = (kAtgPlus)* 5 * disp;
    double k = force/std::sqrt(diff.x*diff.x + diff.y*diff.y);

    fx += k*diff.x;
    fy += k*diff.y;
  }

   return Vector(fx,fy,0);

}

Vector
VirtualSprings2dMobilityModel::ComputeSeedForce ()
{
  Vector pos = m_helper.GetCurrentPosition ();

  double fx = 0;
  double fy = 0;

  std::vector<Seed> seeds = m_manager -> GetSeeds ();

  for (std::vector<Seed>::iterator it = seeds.begin (); it != seeds.end (); ++it)
  { 
    Seed seed = (*it);
    Vector seedPos = seed.pos;
    double k_seed = seed.weight;

    if (CalculateDistance (pos, seed.origin) > 1000)
    {
       continue;
    }

    Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel> ();
    mob -> SetPosition (seedPos);
    double lb = m_estimator -> GetAtgLinkBudget ( Ptr<MobilityModel> (this), mob );

    double disp = m_lbReqAtg - lb;
    Vector diff = seedPos - pos;
    double force = k_seed * disp;
    double k = force/std::sqrt(diff.x*diff.x + diff.y*diff.y);

    fx += k*diff.x;
    fy += k*diff.y;

    //NS_LOG_DEBUG ("Computing seed force to " << seedPos << " = " << Vector (fx, fy, 0));
  }

  return Vector(fx,fy,0);
}

uint32_t
VirtualSprings2dMobilityModel::GetMaxCardinality ()
{
  uint32_t maxCard = 1;

  for (std::map<uint32_t, EdsEntry>::iterator it = m_eds.begin (); it != m_eds.end (); ++it)
  {
    uint16_t cnt = 1;

    for (uint16_t i = 0; i < m_neighbours.size (); i++)
    {
      Ptr<Node> ataNode = NodeList::GetNode(m_neighbours[i]);
      Ptr<VirtualSprings2dMobilityModel> ataMob = ataNode -> GetObject<VirtualSprings2dMobilityModel> ();
      std::map<uint32_t, EdsEntry> otherList = ataMob -> GetEdsList ();

      std::map<uint32_t, EdsEntry>::iterator oIt;
      oIt = otherList.find ( it -> first );

      if ( oIt != otherList.end () )
      {
        cnt ++;
      }
    }

    if (cnt > maxCard)
      maxCard = cnt;
  }

  return maxCard;
}

int
VirtualSprings2dMobilityModel::GetMaxNodesNeighbours()
{
  uint32_t curMax = 0;

  for (uint16_t j = 0; j < m_neighbours.size(); j++)
  {
    Ptr<Node> ataNode = NodeList::GetNode(m_neighbours[j]);
    Ptr<VirtualSprings2dMobilityModel> mob = ataNode -> GetObject<VirtualSprings2dMobilityModel> ();

    uint32_t numAtgNodes = mob -> GetCoveredEds ();

    if (numAtgNodes > curMax){
      curMax = numAtgNodes;
    }
  }

  return curMax;
}

double
VirtualSprings2dMobilityModel::ComputeKatg()
{
  double coveredNodes = m_eds.size ();
  double maxNodesNeighbours = VirtualSprings2dMobilityModel::GetMaxNodesNeighbours();

  if (!maxNodesNeighbours)
  {
    return 1;
  }

  return coveredNodes/maxNodesNeighbours;
}

double
VirtualSprings2dMobilityModel::GetDistanceFromFurthestNeighbour ()
{ 
  double maxDist = 0.0;
  Vector myPos = m_helper.GetCurrentPosition ();

  for (uint16_t i = 0; i < m_neighbours.size (); i++)
  {
    Ptr<Node> node = NodeList::GetNode (m_neighbours[i]);
    Ptr<MobilityModel> mob = node -> GetObject<MobilityModel> ();
    Vector pos = mob -> GetPosition ();
    pos.z = 0;

    double dist = CalculateDistance (myPos, pos);

    if (dist > maxDist)
      maxDist = dist;

  }

  return maxDist;
}

double
VirtualSprings2dMobilityModel::GetDistanceFromBs ()
{ 
  Vector myPos = m_helper.GetCurrentPosition ();
  
  double dist = CalculateDistance (myPos, m_bsPos);

  return dist;
}

/* Return the id of the UAV that share at least PERC_SHARED nodes with the current UAV */

uint32_t
VirtualSprings2dMobilityModel::FindMostSimilarNode ()
{
  uint32_t maxShared = 0;
  uint32_t maxId = 0;

  for (uint16_t i = 0; i < m_neighbours.size (); i++)
  {
    uint32_t numShared = NumSharedEds ( m_neighbours[i] );

    if  (numShared == maxShared && m_neighbours[i] > maxId)
      maxId = m_neighbours[i];

    if (numShared > maxShared)
    {
      maxShared = numShared;
      maxId = m_neighbours[i];
    }
  }

  if ((double)maxShared/m_eds.size () < M_PERC_SHARED)
    return 0;

  return maxId;
}

/* Compute the number of shared EDs with the UAV identified by ataId */

uint32_t
VirtualSprings2dMobilityModel::NumSharedEds (uint32_t ataId)
{
  Ptr<Node> node = NodeList::GetNode (ataId);
  Ptr<VirtualSprings2dMobilityModel> mob = node -> GetObject<VirtualSprings2dMobilityModel> ();
  std::map<uint32_t, EdsEntry> neighEdsList = mob -> GetEdsList ();

  uint32_t counter = 0;
  for (std::map<uint32_t, EdsEntry>::iterator it = neighEdsList.begin (); it != neighEdsList.end (); ++it)
  {
    if ( m_eds.find (it -> first) != m_eds.end () )
      counter ++;
  }

  return counter;
}

void
VirtualSprings2dMobilityModel::SetLinkBudgetEstimator (Ptr<LinkBudgetEstimator> estimator)
{
  m_estimator = estimator;
}

void
VirtualSprings2dMobilityModel::SetOlsrRouting (Ptr<olsr::RoutingProtocol> routing)
{
  m_routing = routing;
}

uint32_t
VirtualSprings2dMobilityModel::GetCoveredEds ()
{
  return m_eds.size ();
}

std::map<uint32_t, EdsEntry>
VirtualSprings2dMobilityModel::GetEdsList ()
{
  return m_monitor -> GetEdsList (Seconds (20));
}

void
VirtualSprings2dMobilityModel::AddAtaNode(uint32_t id)
{
  m_ataNodes.push_back(id); 
}

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
}


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
}

void
VirtualSprings2dMobilityModel::DoSetPosition (const Vector &position)
{ 
  m_helper.SetPosition (position);
  Simulator::Remove (m_event);
  m_event = Simulator::ScheduleNow (&VirtualSprings2dMobilityModel::DoInitializePrivate, this);
}

Vector
VirtualSprings2dMobilityModel::DoGetVelocity (void) const
{
  
  return m_helper.GetVelocity ();
  
}



} // namespace ns3


// double
// VirtualSprings2dMobilityModel::ComputeKata(Ptr<Node> ataNode)
// {
//   // Ptr<VirtualSprings2dMobilityModel> ataMob = ataNode -> GetObject<VirtualSprings2dMobilityModel> ();
//   // std::map<uint32_t, EdsEntry> edsList = ataMob -> GetEdsList ();

//   // if (edsList.size () > 0 && m_rangeApprox > 0)
//   // {
//   //   double rangeRatio = GetDistanceFromBs () / m_rangeApprox;
//   //   return m_kAta + rangeRatio;
//   // }
//   // else
//     return m_kAta;
// }


//KATG PLUS
/*
    //double kAtgPlus = 1;
    // if (m_kAtgPlusMode)
    // {
    //   uint16_t cnt = 1;

    //   for (uint16_t i = 0; i < m_neighbours.size (); i++)
    //   {
    //         Ptr<Node> ataNode = NodeList::GetNode(m_neighbours[i]);
    //         Ptr<MobilityModel> ataMob = ataNode -> GetObject<MobilityModel> ();
    //         Vector ataPos = ataMob -> GetPosition ();
    //         ataPos.z = 0;

    //         double ataDist = CalculateDistance (pos, ataPos);

    //         if (ataDist < m_rangeAtg)
    //         {
    //           cnt++;
    //         }

    //       }

    //       cnt > 1 ? kAtgPlus = 1 : kAtgPlus = 10;

    //       //NS_LOG_DEBUG ("K_ATG_PLUS = " << kAtgPlus);
    // }
    //double force = ( kAtg * kAtgPlus ) * disp;

    //double disp = dist - m_l0Atg;

*/

// void
// VirtualSprings2dMobilityModel::SetEdsList ()
// {
//   std::map<uint32_t, Time> curNodes = m_monitor -> GetEdsList ();

//   m_eds.clear ();

//   for (std::map<uint32_t, Time>::iterator it = curNodes.begin(); it != curNodes.end (); ++it)
//   {
//     Time time = (*it).second;

//     if (time + Seconds(20) > Simulator::Now () )
//       m_eds.push_back ((*it).first);
//   }
// }


// bool
// VirtualSprings2dMobilityModel::HasOnlyAtaForces (uint32_t id, double tolerance)
// {
//   double maxShared = 0;
//   uint32_t minId = 9999;

//   for (uint16_t i = 0; i < m_neighbours.size (); i++)
//   {
//     uint32_t numShared = NumSharedEds ( m_neighbours[i] );

//     if  (numShared == maxShared && m_neighbours[i] < minId)
//       minId = m_neighbours[i];

//     if (numShared > maxShared)
//     {
//       maxShared = numShared;
//       minId = m_neighbours[i];
//     }

//     // if (numShared == maxShared && m_neighbours[i] < minId)
//     //   minId = m_neighbours[i];
//   }

//   //NS_LOG_LOGIC ("Node " << id << " share " << maxShared << " EDs with node " << minId);

//   if (maxShared / (double)m_eds.size () > tolerance && id < minId )
//     return true;

//   return false;

// }

// void
// VirtualSprings2dMobilityModel::DoInitializePrivate (void)
// {
//   m_helper.Update();
//   double speed = m_speed;

//   //Update list of neighbouring UAVs
//   VirtualSprings2dMobilityModel::SetNeighboursList ();

//   //Update list of neighbouring EDs
//   VirtualSprings2dMobilityModel::SetEdsList ();

//   //Compute the direction of the movement
//   Vector forceAta = VirtualSprings2dMobilityModel::ComputeAtaForce ();
//   Vector forceAtg = VirtualSprings2dMobilityModel::ComputeAtgForce ();
//   Vector forceSeed = VirtualSprings2dMobilityModel::ComputeSeedForces ();
  
//   Vector force = forceAta + forceAtg + forceSeed;

//   NS_LOG_DEBUG ("AtG Force = " << forceAtg << ", AtA force = " << forceAta);

//   double k = speed/std::sqrt(force.x*force.x + force.y*force.y);
//   Vector vector (k*force.x,
//                 k*force.y,
//                 0.0);

//   //Move only if force is bigger than a threshold to avoid oscillations
//   if (force.GetLength () > m_tol)
//   {
//     //Compute the future position based on current velocity and direction
//     Vector myPos = m_helper.GetCurrentPosition();
//     Vector nextPos;
//     nextPos.x = myPos.x + vector.x * m_modeTime.GetSeconds ();
//     nextPos.y = myPos.y + vector.y * m_modeTime.GetSeconds ();

//     olsr::RoutingTableEntry entry = VirtualSprings2dMobilityModel::HasPathToBs();
    
//     //If a path to the BS exists
//     if (entry.distance != 0) 
//     {
//       m_hops = entry.distance;
//       double rangeApprox = GetDistanceFromBs () / GetDistanceFromFurthestNeighbour ();
//       m_persist = 3*(entry.distance - 1) * (uint32_t)rangeApprox ;

//       //Send Seed if any
//       if (m_seed.expires != 0){
//         VirtualSprings2dMobilityModel::SendSeed ();
//         NS_LOG_LOGIC ("Sending the seed");
//       }

//       //Move according to forces if m_pause intervals have passed...
//       if (m_pause == 0) 
//       {
//         m_helper.SetVelocity(vector);
//         m_helper.Unpause ();
//       }

//       //Or don't do anything if m_pause intervals still have to pass.
//       if (m_pause > 0)
//       {

//         m_pause --;
//         m_helper.Pause ();
//       }

//     }

//     if (entry.distance  == 0)
//     {
//       //Go back and try to recover connectivity...
//       if (m_persist == 0)
//       {
//         NS_LOG_INFO ("Not in range of BS. Going Back!");
//         Vector curPos = m_helper.GetCurrentPosition ();
//         Vector diff = m_bsPos - curPos;
//         double h = speed*2/std::sqrt(diff.x*diff.x + diff.y*diff.y);
//         Vector vec (h*diff.x,
//                     h*diff.y,
//                     0.0);

//         NS_LOG_INFO ("m_hops = " << m_hops);
//         m_pause = VirtualSprings2dMobilityModel::SetPause(m_hops);

//         //Generate Seed
//         if (!m_eds.empty () && m_seed.expires == 0 && FindMostSimilarNode() == 0 )
//         {
//           NS_LOG_LOGIC ("Creating the seed...");
//           Vector center = VirtualSprings2dMobilityModel::ComputeCenterMassEds ();

//           NS_LOG_LOGIC ("The mass center is " << center);
//           m_seed.center = center;
//           m_seed.expires = 15;
//         }

//         m_helper.SetVelocity(vec);
//         m_helper.Unpause ();
//       }

//       //Try to go on and see if connectivity is recovered after m_persist intervals...
//       if (m_persist > 0)
//       {
//         NS_LOG_INFO ("Persit on route for " << m_persist << " intervals");

//         if (!m_eds.empty ())
//         {
//           uint32_t simNode = FindMostSimilarNode ();
//           //NS_LOG_LOGIC ("Most similar is " << simNode);

//           if (simNode != 0)
//           {
//             if (m_id > simNode)
//               m_persist = m_persist / 2;
//             else
//               m_persist = m_persist * 2;
//           }
//         }
//         else
//         {
//           m_persist --;
//         }
        
//         m_helper.SetVelocity(vector);
//         m_helper.Unpause ();
//       }  

//     }
 
//   }
//   else //Stop if force is too small
//   {
//     m_helper.Pause();
//   }

//   Time delayLeft = m_modeTime;

//   DoWalk (delayLeft);
  
// }

//void
// VirtualSprings2dMobilityModel::AddSeed (Seed seed)
// {
//   m_seeds.push_back (seed);
// }

// void
// VirtualSprings2dMobilityModel::SendSeed ()
// {
//   std::vector<ns3::olsr::RoutingTableEntry> entries = m_routing -> ns3::olsr::RoutingProtocol::GetRoutingTableEntries ();

//   for (uint16_t i = 0; i < entries.size(); i++)
//   {
//     for (uint16_t j = 0; j < m_ataNodes.size (); j++)
//     {
//         Ptr<Node> node = NodeList::GetNode(m_ataNodes[j]);
//         Ptr<VirtualSprings2dMobilityModel> mob = node -> GetObject<VirtualSprings2dMobilityModel> ();
//         Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> (); 
//         Ipv4Address addr = ipv4->GetAddress (1, 0).GetLocal (); 

//         if (addr.IsEqual (entries[i].destAddr) && entries[i].distance == 1)
//         {
//           mob -> AddSeed (m_seed);
//           NS_LOG_LOGIC ("Sent seed to " << node -> GetId ());
//         }

//     }
//   }

//   m_seeds.push_back (m_seed);
//   m_seed = Seed ();
// }

// Vector
// VirtualSprings2dMobilityModel::ComputeCenterMassEds ()
// { 
//   Vector res;

//   for (std::map<uint32_t, EdsEntry>::iterator it = m_eds.begin (); it != m_eds.end (); ++it )
//   {
//     res.x = res.x + (*it).second.x;
//     res.y = res.y + (*it).second.y;
//   }

//   res.x = res.x / m_eds.size ();
//   res.y = res.y / m_eds.size ();

//   return res;
// }

// // uint32_t
// // VirtualSprings2dMobilityModel::SetPause (uint32_t hops)
// // {
// //   if (hops > 0)
// //   {
// //     double pause = 100.0 * std::exp (-m_modeTime.GetSeconds () / 10.0) /(std::pow(hops, 3)) + 1;
// //     NS_LOG_INFO ("Stop for " << (uint32_t)pause << " intervals");
// //     return (uint32_t)pause;
// //   }

// //   return 0;
// // }

// Vector
// VirtualSprings2dMobilityModel::ComputeSeedForces ()
// {
//   Vector pos = m_helper.GetCurrentPosition ();

//   double fx = 0;
//   double fy = 0;

//   double k_seed = 50;

//   for (std::list<Seed>::iterator it = m_seeds.begin (); it != m_seeds.end (); ++it)
//   { 
//     Seed seed = (*it);
//     if ( seed.expires != 0 )
//     {

//       Vector seedPos = seed.center;
//       Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel> ();
//       mob -> SetPosition (seedPos);
//       double lb = m_estimator -> GetAtgLinkBudget ( Ptr<MobilityModel> (this), mob);

//       double disp = m_lbReqAtg - lb;
//       Vector diff = seedPos - pos;
//       double force = k_seed * disp;
//       double k = force/std::sqrt(diff.x*diff.x + diff.y*diff.y);

//       fx += k*diff.x;
//       fy += k*diff.y;

//       (*it).expires--;

//       NS_LOG_LOGIC ("Computing seed force to " << seedPos << " = " << Vector (fx, fy, 0));
//     }

//   }

//   return Vector(fx,fy,0);

// }
