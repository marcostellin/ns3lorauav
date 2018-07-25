/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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

#include "ns3/seeds-manager.h"
#include "ns3/simulator.h"
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SeedsManager");
NS_OBJECT_ENSURE_REGISTERED (SeedsManager);

TypeId
SeedsManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SeedsManager")
    .SetParent<Object> ()
    .AddConstructor<SeedsManager> ();

    return tid;
}

SeedsManager::SeedsManager ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

SeedsManager::~SeedsManager ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void 
SeedsManager::AddToken (Token token)
{
  for (uint16_t i = 0; i < m_tokens.size (); i++)
  {
    double minX = m_tokens[i].lastPos.x - 30;
    double maxX = m_tokens[i].lastPos.x + 30;
    double minY = m_tokens[i].lastPos.y - 30;
    double maxY = m_tokens[i].lastPos.y + 30;

    // if (m_tokens[i].id == token.id)
    // {
    //   m_tokens[i] = token;
    //   return;
    // }

    if (token.lastPos.x > minX && token.lastPos.x < maxX && token.lastPos.y > minY && token.lastPos.y < maxY)
    {
      m_tokens[i] = token;
      return;
    }
  }

  m_tokens.push_back (token);

  NS_LOG_INFO ("Added token." << " LastPos=" << token.lastPos << " LastTime=" << token.lastTime << " Weight=" << token.weight << " Speed=" << token.speed << " Expires=" << token.expires  );
}

std::vector<Seed>
SeedsManager::ComputeSeeds ()
{
  std::vector<Seed> seeds;

  for (uint16_t i = 0; i < m_tokens.size (); i++)
  {
    Token token = m_tokens[i];

    if (token.expires < Simulator::Now ())
      continue;

    Seed seed;
    Vector pos;

    double interval = (Simulator::Now () - token.lastTime).GetSeconds ();
    pos.x = token.lastPos.x + token.speed.x * interval;
    pos.y = token.lastPos.y + token.speed.y * interval;

    seed.pos = pos;
    seed.weight = token.weight;
    seed.origin = Vector (token.lastPos.x, token.lastPos.y, 0);

    seeds.push_back (seed);

    //NS_LOG_INFO ("Seed. " << "Position=" << seed.pos << " Weight=" << seed.weight);

  }

  return seeds;


}

std::vector<Seed> 
SeedsManager::GetSeeds ()
{
  return ComputeSeeds ();
}

}
