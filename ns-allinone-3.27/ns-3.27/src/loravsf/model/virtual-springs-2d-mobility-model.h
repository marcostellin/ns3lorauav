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
#include "ns3/mobility-model.h"
#include "ns3/constant-velocity-helper.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/node-container.h"
#include "ns3/lora-eds-monitor.h"
#include "ns3/link-budget-estimator.h"
#include "ns3/load-monitor.h"
#include "ns3/seeds-manager.h"

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

// struct Seed
// {
//   Seed (): center (Vector ()), expires (0) {}

//   Vector center;
//   uint16_t expires;
// };


class VirtualSprings2dMobilityModel : public MobilityModel 
{
public:
  /**
   * Register this type with the TypeId system.
   * \return the object TypeId
   */
  VirtualSprings2dMobilityModel ();
  static TypeId GetTypeId (void);

  typedef void (* TracedCallback)(const std::vector<uint32_t> &vector);

  void                         AddAtaNode             (uint32_t nodes);
  void                         AddAtgNode             (uint32_t nodes);
  //void                         AddIpv4Address         (Ipv4Address addr);
  void                         SetOlsrRouting         (Ptr<olsr::RoutingProtocol> routing);
  uint32_t                     GetCoveredEds          (void);
  void                         SetLinkBudgetEstimator (Ptr<LinkBudgetEstimator> estimator);
  std::map<uint32_t, EdsEntry> GetEdsList             (void);

  void                         ReceiveToken           (Token token);
  void                         SendToken              (Token token);

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
  void GoBack (void);
  void DoMoveForces (void);

  /**
   * Perform initialization of the object before MobilityModel::DoInitialize ()
   */
  void     DoInitializePrivate    (void);
  void     InitializeMonitors     (void);

  Vector   ComputeAtaForce        (void);
  Vector   ComputeAtgForce        (void);
  Vector   ComputeSeedForce       (void);
  Vector   ComputeTotalForce      (void);
  Vector   GetDirection           (Vector force);

  int      GetMaxNodesNeighbours  (void);
  double   ComputeKatg            (void);
  uint32_t NumSharedEds           (uint32_t ataId);
  double   ComputeKata            (Ptr<Node> ataNode);

  olsr::RoutingTableEntry HasPathToBs                      (void);
  uint32_t                SetPause                         (uint32_t hops);
  void                    SetNeighboursList                (void);
  void                    SetEdsList                       (void);
  double                  GetDistanceFromFurthestNeighbour (void);
  double                  GetDistanceFromBs                (void);
  uint32_t                FindMostSimilarNode              (void);
  uint32_t                GetMaxCardinality ();
  
  void                    UpdateRangeApprox (void);

  void                    CheckConnectivity (void);
  void                    UpdateHistory (void);
  Token                   GenerateToken (void);
  void                    Reattach (void);
  std::vector<Token>      GenerateTokens (std::vector<ClusterInfo> clusters);
  void                    CheckCoverage (void);

  virtual void DoDispose (void);
  virtual void DoInitialize (void);
  virtual Vector DoGetPosition (void) const;
  virtual void DoSetPosition (const Vector &position);
  virtual Vector DoGetVelocity (void) const;

  //CONSTANTS
  const double M_PERC_SHARED;
  const double M_SEC_RETAIN_EDS;
  const double M_MAX_PAUSE;
  const uint32_t M_KATG_PLUS;
  const double M_SEC_CHECK_CONN;

  ConstantVelocityHelper m_helper; //!< helper for this object
  EventId m_event; //!< stored event ID 
  Time m_modeTime; //!< Change current direction and speed after this delay

  Vector m_bsPos;  //!< Stores the position of the BS
  Ipv4Address m_bsAddr; //!< Stores the IP address of the BS
  uint32_t m_pause; //<! stores the number of pause intervals
  uint32_t m_hops;  //<! store the number of hops from the BS of the current or last iteration
  uint32_t m_persist; //<! store the number of intervals the node should go on even if no connectivity to BS
  double m_load;
  bool m_lastNode;

  double m_speed; //!< Speed of aerial nodes
  double m_kAta; //<! stifness of aerial springs
  double m_tol;  //<! minimum force module to cause a movement
  double m_lbReqAta; //<! required LB for AtA links
  double m_lbReqAtg; //<! required LB for AtG links
  double m_rangeApprox; //<! store the approximate range of transmission
  bool m_detach;

  bool m_kAtgPlusMode; //<! if true give more priority to nodes exclusively covered by the current UAV
  uint16_t m_kMode;
  uint16_t m_connMode;


  uint32_t m_id;  //<! id of the UAV
  
  std::vector<uint32_t> m_ataNodes; //<! List of IDs of all UAVs
  std::vector<uint32_t> m_atgNodes; //<! List of IDs of all EDs
  std::vector<uint32_t> m_neighbours; //<! Store current one hop neighbours of current node
  std::vector<uint32_t> m_allNeighbours; //<! Store one hop neighbours + UAVs covering at least one ED;
  std::map<uint32_t, EdsEntry> m_eds; //<! List of currently covered EDs;
  //std::vector<Ipv4Address> m_addresses;

  //LoraMonitor
  Ptr<LoraEdsMonitor> m_monitor;

  //LoadMonitor
  Ptr<LoadMonitor> m_loadMonitor;

  //Link Budget Estimator
  Ptr<LinkBudgetEstimator> m_estimator;

  //Seed Manager
  uint16_t m_predictionMode;
  Ptr<SeedsManager> m_manager;
  Token m_token;

  //Routing protocol
  Ptr<olsr::RoutingProtocol> m_routing;

  //Trace sources
  ns3::TracedCallback<const std::vector<int> &> m_nodesInRangeTrace;
  ns3::TracedCallback<uint32_t, Time> m_disconnectedTimeTrace;
};


} // namespace ns3

#endif /* VIRTUAL_SPRINGS_2D_MOBILITY_MODEL_H */
