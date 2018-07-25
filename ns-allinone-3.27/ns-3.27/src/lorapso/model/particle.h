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

#ifndef PARTICLE_H
#define PARTICLE_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"
#include "ns3/random-variable-stream.h"
#include "ns3/double.h"
#include "ns3/mobility-model.h"

namespace ns3 {

/**
 * This class implement a particle of the 
 */

struct NodeState {
	uint32_t id;
	Vector pos;
	double r;
	double v;
	double deltaR;
	double deltaV;
};

class Particle : public Object
{
public:

  static TypeId GetTypeId (void);
  Particle();
  virtual ~Particle();

  void SetH (uint16_t h);
  void SetVmax (double vmax);

  void                         AddAtaNodes             (std::vector<uint32_t> node);
  void                         AddAtgNodes             (std::vector<uint32_t> node);
  void												 PrintParticleState      (void);
  void 												 InitializeParticle      (void);

private:

  std::vector<uint32_t> m_ataNodes;
  std::vector<uint32_t> m_atgNodes;
  std::vector<NodeState> m_nodeStates;

  double m_vmax;
  uint16_t m_h;
  double m_deltaRmax;
  double m_deltaVmax;

};

}

#endif 