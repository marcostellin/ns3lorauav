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

#ifndef PSO_MANAGER_H
#define PSO_MANAGER_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"
#include "ns3/particle.h"

namespace ns3 {

/**
 * This class manage the PSO algorithm
 */

class PsoManager : public Object
{
public:

  static TypeId GetTypeId (void);
  PsoManager();
  virtual ~PsoManager();

  void                         DoInitialize           (void);

  void                         AddAtaNode             (uint32_t node);
  void                         AddAtgNode             (uint32_t node);

private:

  void DoInitializePrivate ();
  void RunPso ();
  void InitializePopulation ();

  std::vector<uint32_t> m_ataNodes;
  std::vector<uint32_t> m_atgNodes;
  std::vector<Ptr<Particle>> m_particles;

  uint32_t m_mu;
  double m_speed;

  Time m_t;
  uint16_t m_h;

};

}

#endif 