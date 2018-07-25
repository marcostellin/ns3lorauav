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
#ifndef VIRTUAL_SPRINGS_2D_MOBILITY_MODEL_H
#define VIRTUAL_SPRINGS_2D_MOBILITY_MODEL_H

#include "ns3/object.h"
#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/rectangle.h"
#include "ns3/random-variable-stream.h"
#include "ns3/traced-callback.h"
#include "mobility-model.h"
#include "constant-velocity-helper.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/node-container.h"
#include "ns3/lora-eds-monitor.h"

namespace ns3 {


/**
 * \ingroup mobility
 * \brief 2D random walk mobility model.
 *
 * Each instance moves with a speed and direction choosen at random
 * with the user-provided random variables until
 * either a fixed distance has been walked or until a fixed amount
 * of time. If we hit one of the boundaries (specified by a rectangle),
 * of the model, we rebound on the boundary with a reflexive angle
 * and speed. This model is often identified as a brownian motion
 * model.
 */
class VirtualSprings2dMobilityModel : public MobilityModel 
{
public:
  /**
   * Register this type with the TypeId system.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  typedef void (* TracedCallback)(const std::vector<uint32_t> &vector);
  void AddAtaNode(uint32_t nodes);
  void AddAtgNode(uint32_t nodes);
  void SetOlsrRouting(Ptr<olsr::RoutingProtocol> routing);
  void SetGwLoraPhy (Ptr<GatewayLoraPhy> gwPhy);


private:
  /**
   * \brief Performs the rebound of the node if it reaches a boundary
   * \param timeLeft The remaining time of the walk
   */
  void Rebound (Time timeLeft);
  /**
   * Walk according to position and velocity, until distance is reached,
   * time is reached, or intersection with the bounding box
   */
  void DoWalk (Time timeLeft);
  /**
   * Perform initialization of the object before MobilityModel::DoInitialize ()
   */
  void DoInitializePrivate (void);
  Vector ComputeAtaForce (void);
  Vector ComputeAtgForce (void);
  int GetMaxNodesNeighbours(void);
  double ComputeKatg(void);
  int ComputeNumGroudNodes(Ptr<Node> node);
  int ComputeNumGroudNodes(void);
  bool HasPathToBs (Vector myPos);
  olsr::RoutingTableEntry HasPathToBs ();
  uint32_t SetPause (uint32_t hops);
  void SetNeighboursList (void);


  virtual void DoDispose (void);
  virtual void DoInitialize (void);
  virtual Vector DoGetPosition (void) const;
  virtual void DoSetPosition (const Vector &position);
  virtual Vector DoGetVelocity (void) const;

  ConstantVelocityHelper m_helper; //!< helper for this object
  EventId m_event; //!< stored event ID 
  Time m_modeTime; //!< Change current direction and speed after this delay

  Vector m_bsPos;
  Ipv4Address m_bsAddr;
  Vector m_prevPos;
  uint32_t m_pause;
  uint32_t m_hops;
  uint32_t m_persist;

  double m_speed; //!< Speed of aerial nodes
  double m_kAta; //<! stifness of aerial springs
  double m_rangeAta; //<! tx range of AtA communications
  double m_rangeAtg; //<! tx range of ArG communications
  double m_l0Ata;
  double m_l0Atg;
  double m_tol;

  bool m_kAtgPlusMode;
  
  std::vector<uint32_t> m_ataNodes;
  std::vector<uint32_t> m_atgNodes;
  std::vector<uint32_t> m_neighbours;
  //std::vector<int> m_nodesCovered;

  //LoraMonitor
  LoraEdsMonitor m_monitor;

  Ptr<olsr::RoutingProtocol> m_routing;

  //Trace sources
  ns3::TracedCallback<const std::vector<int> &> m_nodesInRangeTrace;
};


} // namespace ns3

#endif /* VIRTUAL_SPRINGS_2D_MOBILITY_MODEL_H */
