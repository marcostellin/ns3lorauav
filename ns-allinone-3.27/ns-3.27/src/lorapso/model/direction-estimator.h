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

#ifndef DIRECTION_ESTIMATOR_H
#define DIRECTION_ESTIMATOR_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"
#include "ns3/mobility-model.h"
#include <queue>

namespace ns3 {

/**
 * This class implement a DirectionEstimator for the node
 */

class DirectionEstimator : public Object
{
public:

  static TypeId GetTypeId (void);
  DirectionEstimator();
  DirectionEstimator(Time t);
  virtual ~DirectionEstimator();

  Vector GetEstimatedPosition (void);
  Vector GetEstimatedVelocity (void);


private:

  void UpdatePositions (void);
  void InitializeDirectionEstimator ();

  std::queue<Vector> m_positions;

  const uint16_t MAX_POS;
  uint16_t m_index;
  Time m_t;

};

}

#endif 