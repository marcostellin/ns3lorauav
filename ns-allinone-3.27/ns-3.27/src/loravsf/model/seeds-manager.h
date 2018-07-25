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

#ifndef SEEDS_MANAGER_H
#define SEEDS_MANAGER_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"
#include <queue>


namespace ns3 {

struct Seed
{
  Vector origin;
  Vector pos;
  double weight;
};

struct Token
{
  Vector lastPos;
  Time lastTime;
  double weight;
  Vector speed;
  Time expires;
  uint32_t priority;
  uint32_t id;
};


/**
 * This class manage the seeds
 */

class SeedsManager : public Object
{
public:

  static TypeId GetTypeId (void);
  SeedsManager();
  virtual ~SeedsManager();

  void AddToken (Token token);
  std::vector<Seed> GetSeeds ();
  
private:

  std::vector<Seed> ComputeSeeds ();

  std::vector<Token> m_tokens;
};

}

#endif /* DEVICE_STATUS_H */
