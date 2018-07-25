/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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

#ifndef LORA_EDS_MONITOR_H
#define LORA_EDS_MONITOR_H

#include "ns3/object.h"
#include "ns3/gateway-lora-phy.h"
#include <queue>
#include "ns3/kmeans.h"

namespace ns3 {

struct EdsEntry 
{
	double x;
	double y;
	Time time;
};

struct ClusterInfo
{
  uint16_t id;
  Time lastTime;
  Vector velocity;
  Vector lastPosition;
  double weight;

  ClusterInfo ():weight(0)
  {
  }
};

/**
 * This class keeps track of the connected EDs to the gateway
 */
class LoraEdsMonitor : public SimpleRefCount<LoraEdsMonitor>
{
public:

  LoraEdsMonitor();
  virtual ~LoraEdsMonitor();

  LoraEdsMonitor(Ptr<GatewayLoraPhy> gatewayPhy);

  std::map<uint32_t, EdsEntry> GetEdsList (void);
  std::map<uint32_t, EdsEntry> GetEdsList (Time tolerance);
  void UpdateHistory (Time tolerance);
  std::queue<EdsEntry> GetEdsHistory (void);
  Vector GetSpeedVector (void);
  Time   GetLastTime (void);
  Vector GetLastPosition (void);
  void UpdateLostEds (Time tolerance);
  void FilterLostEds (Time interTol, Time lostTol);
  std::vector<ClusterInfo> GetClusterInfo (void);
  std::vector<ClusterInfo> ComputeClustersInfo (point pt, int len, uint16_t k);
  void CreateClusters (void);
  uint16_t CreateClusters (point pt, int len, uint16_t kMax);
  uint32_t GetEdsLostSize (void);
  //std::map<uint32_t, std::queue<EdsEntry>> FilterHistory (Time tolerance);
  //std::map<uint32_t, Vector> GetDirectionList (Time tolerance);
  //std::map<uint32_t, Vector> GetDirectionList (std::map<uint32_t, std::queue<EdsEntry>> filtered);
  //void UpdateClusters (Time tolerance);
  //uint16_t GetClustersNumber (Time tolerance);

private:

  double GetSpeed (void);
  Vector GetDirection (void);
  void UpdateHistory (uint32_t id, Vector pos, Time delta);
  double ComputeAverageSilhoutte (point pt, int len, uint16_t k);

  Vector ComputeClusterCenter (point pt, int len, uint16_t k);
  Vector ComputeCenterMassVel (point pt, int len, uint16_t k, Vector centerMassCur);
  Time   GetLastTime (point pt, int len, uint16_t k);

  void PrintHistory ();
  void PrintHistory (std::map<uint32_t, std::queue<EdsEntry>> map);
  void PrintPts (point pt, int len);

  void UpdateList (Ptr<Packet const> packet, uint32_t id);

  Ptr<GatewayLoraPhy> m_phy;   //!< Pointer to the phy

  std::map<uint32_t, EdsEntry> m_eds;
  std::queue<EdsEntry> m_eds_hist;
  std::map<uint32_t, std::queue<EdsEntry> > m_eds_single_hist;
  std::map<uint32_t, std::queue<EdsEntry> > m_eds_lost;

  std::map<uint32_t, std::queue<EdsEntry> > m_clusters [8];
  //ClusterInfo m_clusters [8];
  //ClusterInfo m_lost_clusters[8];



};

}

#endif /* DEVICE_STATUS_H */
