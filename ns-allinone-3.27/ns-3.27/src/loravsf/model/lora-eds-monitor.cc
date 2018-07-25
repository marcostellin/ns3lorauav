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

#include "ns3/lora-eds-monitor.h"
#include "ns3/log.h"
#include "ns3/lora-tag.h"
#include "ns3/node-list.h"
#include "ns3/random-variable-stream.h"
#include <algorithm>
//#include "kmeans.cc"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LoraEdsMonitor");

LoraEdsMonitor::LoraEdsMonitor ()
{
  NS_LOG_FUNCTION (this);
}

LoraEdsMonitor::~LoraEdsMonitor ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

LoraEdsMonitor::LoraEdsMonitor (Ptr<GatewayLoraPhy> gatewayPhy)
{
  NS_LOG_FUNCTION (this);
  m_phy = gatewayPhy;
  m_phy -> TraceConnectWithoutContext ("ReceivedPacket",
                                        MakeCallback (&LoraEdsMonitor::UpdateList, this));
}

void
LoraEdsMonitor::UpdateList (Ptr<Packet const> packet, uint32_t id)
{
  NS_LOG_FUNCTION (this);

  LoraTag tag;
  bool found = packet -> PeekPacketTag (tag);

  if (found)
  {    
    std::map<uint32_t, EdsEntry>::iterator it;
    uint32_t edId = tag.GetId ();
    Ptr<Node> node = NodeList::GetNode (edId);
    Ptr<MobilityModel> mob = node -> GetObject<MobilityModel> ();
    Vector pos = mob -> GetPosition ();

    it = m_eds.find (tag.GetId ());

    if (it != m_eds.end ())
    {
       it -> second.time = Simulator::Now ();
       it -> second.x = pos.x;
       it -> second.y = pos.y;
    }
    else
    {
       EdsEntry entry;
       entry.time = Simulator::Now ();
       entry.x = pos.x;
       entry.y = pos.y;

       m_eds[edId] = entry;
    }

    LoraEdsMonitor::UpdateHistory (edId, pos, Simulator::Now ());
  }

}

void
LoraEdsMonitor::UpdateHistory (Time tolerance)
{
  std::map<uint32_t, EdsEntry> eds = LoraEdsMonitor::GetEdsList (tolerance);

  if (eds.size () > 0)
  {
    EdsEntry e;
    //Compute center of mass of eds
    for (std::map<uint32_t, EdsEntry>::iterator it = eds.begin (); it != eds.end (); ++it )
    {
      //NS_LOG_INFO ("ED : " << (*it).second.x << ":" << (*it).second.y);
      e.x = e.x + (*it).second.x;
      e.y = e.y + (*it).second.y;
      e.time = (*it).second.time;
    }

    e.x = e.x / eds.size ();
    e.y = e.y / eds.size ();

    //Save entry in queue
    m_eds_hist.push (e);
    //NS_LOG_INFO ("Center mass: " << e.x << ":" << e.y);
    if (m_eds_hist.size () > 3)
      m_eds_hist.pop ();
  }
}

void
LoraEdsMonitor::UpdateHistory (uint32_t id, Vector pos, Time delta)
{
  std::map<uint32_t, std::queue<EdsEntry>>::iterator it;

  EdsEntry e;
  e.x = pos.x;
  e.y = pos.y;
  e.time = delta;

  it = m_eds_single_hist.find (id);

  if (it == m_eds_single_hist.end ())
  {
    std::queue<EdsEntry> q;
  
    q.push (e);
    m_eds_single_hist[id] = q;
  }
  else
  {
    std::queue<EdsEntry> q = it -> second;

    if (q.size () >= 2)
      q.pop();

    q.push (e);
    m_eds_single_hist[id] = q;
  }
}

void
LoraEdsMonitor::UpdateLostEds (Time tolerance)
{
  Time now = Simulator::Now ();

  for (auto it = m_eds_single_hist.cbegin(); it != m_eds_single_hist.cend();)
  {
    uint32_t id = it -> first;
    std::queue<EdsEntry> q = it -> second;

    EdsEntry e2 = q.back ();

    if (e2.time  < now - tolerance)
    {
      m_eds_lost[id] = q;
      m_eds_single_hist.erase (it++);
    } 
    else
    {
      ++it;
    }
  }

  //PrintHistory (m_eds_lost);
}

void
LoraEdsMonitor::FilterLostEds (Time interTol, Time lostTol)
{
  Time now = Simulator::Now ();

  for (auto it = m_eds_lost.cbegin(); it != m_eds_lost.cend();)
  {
    std::queue<EdsEntry> q = it -> second;

    if (q.size () < 2)
    {
      ++it;
    } 
    else
    {
      EdsEntry e1 = q.front ();
      EdsEntry e2 = q.back ();

      //NS_LOG_INFO ("Diff1: " << (now - e2.time).GetSeconds () );
      //NS_LOG_INFO ("Diff2 " << (e2.time - e1.time).GetSeconds ());
      if ( ( (now - e2.time) > lostTol ) || ( (e2.time - e1.time) > interTol) )
      {
        //NS_LOG_INFO ("Erased " << it -> first);  
        m_eds_lost.erase (it++);
      }
      else
        ++it;
    }
  }

  //NS_LOG_INFO ("SIZE " << m_eds_lost.size ());
  //PrintHistory (m_eds_lost);
}

uint16_t
LoraEdsMonitor::CreateClusters (point pt, int len, uint16_t kMax)
{
  if (m_eds_lost.size () == 0)
    return 0;

  //point pt = (point) malloc(sizeof(point_t)*m_eds_lost.size ());

  for (uint16_t j = 2; j <= kMax; j++ )
  {
    uint16_t cnt = 0;

    for (std::map<uint32_t, std::queue<EdsEntry>>::iterator it = m_eds_lost.begin (); it != m_eds_lost.end (); ++it)
    {
      uint32_t id = it -> first;
      std::queue<EdsEntry> q = it -> second;

      point_t p;
      p.id = id;
      p.x = q.back().x;
      p.y = q.back().y;

      pt[cnt] = p; 
      cnt++;
    }

    //print_points (pt, m_eds_lost.size ());
    if (m_eds_lost.size () >= j)
    {
      lloyd(pt, m_eds_lost.size (), j);
      double sil = ComputeAverageSilhoutte (pt, len, j);
      PrintPts (pt, m_eds_lost.size ());
      NS_LOG_INFO ("Silhoutte for k = " << j << " is " << sil);
      
      if (sil > 0.8)
        return j;      
    }
  }

  return 1;
}

double
LoraEdsMonitor::ComputeAverageSilhoutte (point pt, int len, uint16_t k)
{
  //Contains the number of nodes per cluster;
  uint16_t clusterPts[k];
  for (uint16_t i = 0; i < k; i++)
    clusterPts[i] = 0;

  //double *aI = (double*) malloc (sizeof (double) * len);
  double *aI = new double[len];

  //Compute the average distance 'a' from each node in a cluster to the other nodes in the same cluster
  for (uint16_t i = 0; i < len; i++)
  {
    int group = pt[i].group;
    clusterPts[group] ++;
    Vector pos = Vector (pt[i].x, pt[i].y, 0);
    uint16_t cnt = 0;
    aI[i] = 0;

    for (uint16_t j = 0; j < len; j++)
    {
      if (i != j && group == pt[j].group)
      {
        cnt++;
        Vector oPos = Vector (pt[j].x, pt[j].y, 0);
        double dist = CalculateDistance (oPos, pos);
        aI[i] += dist;
      }
    }

    if (cnt > 0)
      aI[i] = aI[i] / cnt;
  }

  //Compute the average distance 'b' from each node in a cluster to other nodes in different clusters
  //double *bI = (double*) malloc (sizeof (double)  * len);

  double** bI = new double*[len];
  for (uint16_t i = 0; i < len; ++i)
    bI[i] = new double[k];

  for (uint16_t i = 0; i < len; i++)
    for (uint16_t j = 0; j < k; j++)
      bI[i][j] = 0;

  for (uint16_t i = 0; i < len; i++)
  {
    int group = pt[i].group;
    Vector pos = Vector (pt[i].x, pt[i].y, 0);


    for (uint16_t j = 0; j < len; j++)
    {
      if (i != j && group != pt[j].group)
      {
        Vector oPos = Vector (pt[j].x, pt[j].y, 0);
        double dist = CalculateDistance (oPos, pos);
        bI[i][pt[j].group] += dist/clusterPts[pt[j].group];
      }
    }
  }

  //Select the closest cluster to each node and save its distance
  double *bIFinal = new double[len];
  for (uint16_t i = 0; i < len; i++)
  { 
    double min = 1e9;
    for (uint16_t j = 0; j < k; j++)
    {
      double cur = bI[i][j];
      if (cur != 0 && cur < min)
        min = cur;
    }

    bIFinal[i] = min;
  }

  //Compute silhoutte value for each node
  double *sI = new double[len];
  for (uint16_t i = 0; i < len; i++)
  {
    if (aI[i] < bIFinal[i])
      sI[i] = 1 - aI[i] / bIFinal[i];

    if (aI[i] > bIFinal[i])
      sI[i] = aI[i] / bIFinal[i] - 1;

    if (aI[i] == bIFinal[i])
      sI[i] = 0;

    if (aI[i] == 0)
      sI[i] = 0;
  }

  //Compute average silhoutte
  double avgS = 0;

  for (uint16_t i = 0; i < len; i++)
  {
    avgS = avgS + sI[i];
  }

  //Free memory
  delete [] aI;
  for (uint16_t i = 0; i < len; ++i)
    delete [] bI[i];

  delete [] bI;
  delete [] sI;
  delete [] bIFinal;

  return avgS / (double) len;

}

std::vector<ClusterInfo>
LoraEdsMonitor::ComputeClustersInfo (point pt, int len, uint16_t k)
{
  std::vector<ClusterInfo> clusters;

  if (k == 0)
    return clusters;

  Ptr<UniformRandomVariable> rd = CreateObject<UniformRandomVariable> ();
  rd->SetAttribute ("Min", DoubleValue (0));
  rd->SetAttribute ("Max", DoubleValue (100000));

  if (k == 1)
  {
    for (uint16_t i = 0; i < len; i++)
    {
      pt[i].group = 0;
    }
  }

  uint32_t clusterMembers[k];
  for (uint16_t i = 0; i < k; i++)
    clusterMembers[i] = 0;

  for (uint16_t i = 0; i < len; i++)
  {
    int group = pt[i].group;
    clusterMembers[group] ++;
  }

  if (k > 0)
  {
    for (uint16_t i = 0; i < k; i++)
    {
      Vector centerMass = ComputeClusterCenter (pt, len, i);
      NS_LOG_INFO ("Center Mass of cluster " << i << " is " << centerMass);
      Vector centerMassVel = ComputeCenterMassVel (pt, len, i, centerMass);
      NS_LOG_INFO ("Center Mass Velocity of cluster " << i << " is " << centerMassVel);
      Time lastTime = GetLastTime (pt, len, i);
      NS_LOG_INFO ("Last time of cluster " << i << " is " << lastTime.GetSeconds () << " s");

      if (centerMassVel.GetLength () > 0)
      {
        ClusterInfo cluster;
        cluster.id = rd -> GetInteger();
        cluster.lastTime = lastTime;
        cluster.velocity = centerMassVel;
        cluster.lastPosition = centerMass;
        cluster.weight = clusterMembers[i];

        clusters.push_back (cluster);
      }

    }
  }

  // if (k == 1)
  // {
  //   for (uint16_t i = 0; i < len; i++)
  //   {
  //     pt[i].group = 0;
  //   }

  //   Vector centerMass = ComputeClusterCenter (pt, len, 0);
  //   NS_LOG_INFO ("Center Mass of cluster " << 0 << " is " << centerMass);
  //   Vector centerMassVel = ComputeCenterMassVel (pt, len, 0, centerMass);
  //   NS_LOG_INFO ("Center Mass Velocity of cluster " << 0 << " is " << centerMassVel);
  //   Time lastTime = GetLastTime (pt, len, 0);
  //   NS_LOG_INFO ("Last time of cluster " << 0 << " is " << lastTime.GetSeconds () << " s");

  //   if (centerMassVel.GetLength () > 0)
  //   {
  //     ClusterInfo cluster;
  //     cluster.id = rd -> GetInteger();
  //     cluster.lastTime = lastTime;
  //     cluster.velocity = centerMassVel;
  //     cluster.lastPosition = centerMass;
  //     cluster.weight = clusterMembers[0];

  //     clusters.push_back (cluster);
  //   }
  // }

  return clusters;
}

Vector
LoraEdsMonitor::ComputeClusterCenter (point pt, int len, uint16_t k)
{
  Vector centerMass;
  uint16_t cnt = 0;

  for (uint16_t i = 0; i < len; i++)
  {
    if (pt[i].group == k)
    {
      cnt ++;
      centerMass.x += pt[i].x;
      centerMass.y += pt[i].y;
    }
  }

  centerMass.x /= (double)cnt;
  centerMass.y /= (double)cnt;

  return centerMass;
}

Vector
LoraEdsMonitor::ComputeCenterMassVel (point pt, int len, uint16_t k, Vector centerMassCur)
{
  Vector centerMassVel;
  Vector centerMassPrev;
  uint16_t cnt = 0;
  
  for (uint16_t i = 0; i < len; i++)
  {
    if (pt[i].group == k)
    {
      uint32_t id = pt[i].id;

      std::map<uint32_t, std::queue<EdsEntry> >::iterator it = m_eds_lost.find (id);

      if (it != m_eds_lost.end ())
      {
        std::queue<EdsEntry> q = it -> second;

        if (q.size () < 2)
          continue;
        else
        {
          cnt++;
          centerMassPrev = centerMassPrev + Vector ( q.front().x, q.front().y, 0 );
        }
      }
    }
  }


  if ( centerMassPrev.GetLength () == 0 )
  {
    NS_LOG_INFO ("The center of mass of previous position of cluster " << k << " is " << Vector(0,0,0));
    return Vector (0,0,0);
  }

  centerMassPrev.x /= (double)cnt;
  centerMassPrev.y /= (double)cnt;

  NS_LOG_INFO ("The center of mass of previous position of cluster " << k << " is " << centerMassPrev);

  return centerMassCur - centerMassPrev;
}

Time
LoraEdsMonitor::GetLastTime (point pt, int len, uint16_t k)
{
  Time lastTime = Seconds (0);

  for (uint16_t i = 0; i < len; i++)
  {
    if (pt[i].group == k)
    {
      uint32_t id = pt[i].id;

      std::map<uint32_t, std::queue<EdsEntry> >::iterator it = m_eds_lost.find (id);

      if (it != m_eds_lost.end ())
      {
        std::queue<EdsEntry> q = it -> second;
        if (q.back().time > lastTime)
          lastTime = q.back ().time;
      }
    }
  }

  return lastTime;
}

void
LoraEdsMonitor::CreateClusters ()
{
  for  (uint16_t i = 0; i < 8; i++)
  {
    m_clusters[i].clear ();
  }

  for (std::map<uint32_t, std::queue<EdsEntry>>::iterator it = m_eds_lost.begin (); it != m_eds_lost.end (); ++it)
  {
    uint32_t id = it -> first;
    std::queue<EdsEntry> q = it -> second;

    if (q.size () < 2)
      continue;

    Vector dir  = Vector (q.back().x - q.front().x , q.back().y - q.front ().y, 0);

    double theta = std::atan2 (dir.y,dir.x) * 180 / M_PI;

    if (theta > 0 && theta < 45)
      m_clusters[0][id] = q;
    else if (theta > 45 && theta <= 90)
      m_clusters[1][id] = q;
    else if (theta > 90 && theta <= 135)
      m_clusters[2][id] = q;
    else if (theta > 135 && theta <= 180)
      m_clusters[3][id] = q;
    else if (theta > 180 && theta <= 225 )
      m_clusters[4][id] = q;
    else if (theta > 225 && theta <= 270)
      m_clusters[5][id] = q;
    else if (theta > 270 && theta <= 315)
      m_clusters[6][id] = q;
    else
      m_clusters[7][id] = q;
  }

  // for (uint16_t i = 0; i < 8; i++)
  // {
  //   if (m_clusters[i].size () >= 5)
  //   {
  //     NS_LOG_INFO ("CLUSTER " << i);
  //     PrintHistory(m_clusters[i]);
  //   }
  // }
}

std::vector<ClusterInfo>
LoraEdsMonitor::GetClusterInfo ()
{
  LoraEdsMonitor::CreateClusters ();

  std::vector<ClusterInfo> dirs;

  for (uint16_t i = 0; i < 8; i++)
  {
    if (m_clusters[i].size () < 4)
      continue;

    std::map<uint32_t, std::queue<EdsEntry>>::iterator it = m_clusters[i].begin ();

    EdsEntry e1 = it -> second.front ();
    EdsEntry e2 = it -> second.back ();

    ClusterInfo cInfo;
    cInfo.id = i;
    cInfo.lastTime = e2.time;
    cInfo.lastPosition = Vector (e2.x, e2.y, 0);
    cInfo.velocity = Vector (e2.x - e1.x , e2.y - e1.y, 0);

    dirs.push_back (cInfo);
  }

  for (std::vector<ClusterInfo>::iterator it = dirs.begin (); it != dirs.end (); ++it)
  {
    ClusterInfo c = *it;
    NS_LOG_INFO (c.id << " : " << "dir: " << c.velocity);
  }

  return dirs;
}

uint32_t
LoraEdsMonitor::GetEdsLostSize ()
{
  return m_eds_lost.size ();
}


Vector
LoraEdsMonitor::GetDirection ()
{
  std::queue<EdsEntry> eds = LoraEdsMonitor::GetEdsHistory ();

  double size = eds.size ();
  Vector res;

  while (!eds.empty ())
  {
    Vector v1 ( eds.front ().x, eds.front ().y, 0.0 );
    eds.pop ();

    if (!eds.empty ())
    {
      Vector v2 ( eds.front ().x, eds.front ().y, 0.0 );
      res.x += (v2 - v1).x;
      res.y += (v2 - v1).y;
    }
  }

  if (size > 1)
  {
    res.x = res.x / (size - 1);
    res.y = res.y / (size - 1);
  }

  return res;

}

double
LoraEdsMonitor::GetSpeed ()
{
  std::queue<EdsEntry> eds = LoraEdsMonitor::GetEdsHistory ();

  double interval =  10.0;
  double size = eds.size ();
  double speed = 0;

  while (!eds.empty ())
  {
    Vector v1 ( eds.front ().x, eds.front ().y, 0.0 );
    eds.pop ();

    if (!eds.empty ())
    {
      Vector v2 ( eds.front ().x, eds.front ().y, 0.0 );
      speed += CalculateDistance (v1, v2) / interval;
    }
  }

  if (size > 1)
  {
    speed = speed / (size - 1);
  }

  return speed;

}

Vector
LoraEdsMonitor::GetSpeedVector ()
{
  Vector dir = LoraEdsMonitor::GetDirection ();
  double speed = LoraEdsMonitor::GetSpeed ();

  Vector vector;

  if (dir.GetLength () > 0)
  {
    double k = speed/std::sqrt(dir.x*dir.x + dir.y*dir.y);
    vector.x = k*dir.x;
    vector.y = k*dir.y;
  }

  //NS_LOG_INFO ("Direction: " << dir << " Speed: " << speed << " Speed Vector: " << vector);

  return vector;

}

Time
LoraEdsMonitor::GetLastTime ()
{
  return m_eds_hist.back ().time;
}

Vector
LoraEdsMonitor::GetLastPosition ()
{
  return Vector ( m_eds_hist.back().x, m_eds_hist.back ().y, 0.0 );
}

std::map<uint32_t, EdsEntry>
LoraEdsMonitor::GetEdsList (void)
{
  return m_eds;
}

std::queue<EdsEntry>
LoraEdsMonitor::GetEdsHistory (void)
{
  return m_eds_hist;
}

void
LoraEdsMonitor::PrintHistory ()
{
  for (std::map<uint32_t, std::queue<EdsEntry>>::iterator it = m_eds_single_hist.begin (); it != m_eds_single_hist.end (); ++it)
  {
    uint32_t id = it -> first;
    std::queue<EdsEntry> q = it -> second;

    EdsEntry e1;
    EdsEntry e2;

    if (q.size () == 1)
    {
      e1 = q.front();
      NS_LOG_INFO (id << ": " << "Pos1=" << Vector(e1.x, e1.y, 0) << " Time1=" << e1.time << " Pos2=null");

    }
    else if (q.size () == 2)
    {
      e1 = q.front ();
      e2 = q.back ();
      NS_LOG_INFO (id << ": " << "Pos1=" << Vector(e1.x, e1.y, 0) << " Time1=" << e1.time << " Pos2=" << Vector (e2.x, e2.y, 0) << " Time2= " << e2.time);
    }
    else {
      NS_LOG_INFO ("Problem here!");
    }

  }
}

void
LoraEdsMonitor::PrintHistory (std::map<uint32_t, std::queue<EdsEntry>> map)
{
  for (std::map<uint32_t, std::queue<EdsEntry>>::iterator it = map.begin (); it != map.end (); ++it)
  {
    uint32_t id = it -> first;
    std::queue<EdsEntry> q = it -> second;

    EdsEntry e1;
    EdsEntry e2;

    if (q.size () == 1)
    {
      e1 = q.front();
      NS_LOG_INFO (id << ": " << "Pos1=" << Vector(e1.x, e1.y, 0) << " Time1=" << e1.time.GetSeconds () << " Pos2=null");

    }
    else if (q.size () == 2)
    {
      e1 = q.front ();
      e2 = q.back ();
      NS_LOG_INFO (id << ": " << "Pos1=" << Vector(e1.x, e1.y, 0) << " Time1=" << e1.time.GetSeconds() << " Pos2=" << Vector (e2.x, e2.y, 0) << " Time2= " << e2.time.GetSeconds ());
    }
    else {
      NS_LOG_INFO ("Problem here!");
    }

  }
}

void
LoraEdsMonitor::PrintPts (point pts, int len)
{
  NS_LOG_INFO ("------------------------------------");
  for (int i = 0; i < len; i++)
  {
    point_t pt = pts[i];
    NS_LOG_INFO ("id=" << pt.id << " x=" << pt.x << ", y=" << pt.y << ", group=" << pt.group);
    //printf("x=%f, y=%f, group=%d \n", pt.x, pt.y, pt.group);
  }
}

std::map<uint32_t, EdsEntry>
LoraEdsMonitor::GetEdsList (Time tolerance)
{
  Time cur = Simulator::Now ();

  std::map<uint32_t, EdsEntry> filtered;

  for (std::map<uint32_t, EdsEntry>::iterator it = m_eds.begin (); it != m_eds.end (); ++it)
  {
    Time last = (*it).second.time;

    if (cur < last + tolerance)
      filtered.insert (std::pair<uint32_t, EdsEntry> ( (*it).first, (*it).second ));
  }

  return filtered;
}

}

// std::map<uint32_t, std::queue<EdsEntry>>
// LoraEdsMonitor::FilterHistory (Time tolerance)
// {
//   Time now = Simulator::Now ();
//   std::map<uint32_t, std::queue<EdsEntry>> filtered;

//   for (std::map<uint32_t, std::queue<EdsEntry>>::iterator it = m_eds_single_hist.begin (); it != m_eds_single_hist.end (); ++it)
//   {
//     uint32_t id = it -> first;
//     std::queue<EdsEntry> q = it -> second;

//     if (q.size () == 2)
//     {
//       EdsEntry e2 = q.back ();
//       EdsEntry e1 = q.front ();

//       if (e2.time > now - tolerance && e2.time - e1.time < tolerance)
//         filtered[id] = q;
//     }
//   }

//   return filtered;

// }

// std::map<uint32_t, Vector>
// LoraEdsMonitor::GetDirectionList (std::map<uint32_t, std::queue<EdsEntry>> filtered)
// {
//   std::map<uint32_t, Vector> dirs;

//   for (std::map<uint32_t, std::queue<EdsEntry>>::iterator it = filtered.begin (); it != filtered.end (); ++it)
//   {
//     uint32_t id = it -> first;
//     std::queue<EdsEntry> q = it -> second;

//     dirs[id] = Vector (q.back().x - q.front().x , q.back().y - q.front ().y, 0);
//   }

//   return dirs;
// }

// void
// LoraEdsMonitor::UpdateClusters (Time tolerance)
// {
//   std::map<uint32_t, std::queue<EdsEntry>> filtered = LoraEdsMonitor::FilterHistory (tolerance);
//   std::map<uint32_t, Vector> dirs = LoraEdsMonitor::GetDirectionList (filtered);

//   std::vector<uint32_t> membership[8];

//   for (std::map<uint32_t, Vector>::iterator it = dirs.begin (); it != dirs.end (); ++it) 
//   {
//     Vector curDir =  it -> second;
//     uint32_t id = it -> first;
//     double theta = std::atan2 (curDir.y,curDir.x) * 180 / M_PI;

//     if (theta > 0 && theta < 45)
//       membership[0].push_back (id);
//     else if (theta > 45 && theta <= 90)
//       membership[1].push_back (id);
//     else if (theta > 90 && theta <= 135)
//       membership[2].push_back (id);
//     else if (theta > 135 && theta <= 180)
//       membership[3].push_back (id);
//     else if (theta > 180 && theta <= 225 )
//       membership[4].push_back (id);
//     else if (theta > 225 && theta <= 270)
//       membership[5].push_back (id);
//     else if (theta > 270 && theta <= 315)
//       membership[6].push_back (id);
//     else
//       membership[7].push_back (id);
//   }

//   for (uint16_t i = 0; i < 8; i++) 
//   {
//     if (membership[i].size () < 5)
//     {
//       if (m_clusters[i].velocity.GetLength () > 0)
//         m_lost_clusters[i] = m_clusters[i];

//       continue;
//     }

//   } 


// }

// uint16_t
// LoraEdsMonitor::GetClustersNumber (Time tolerance)
// {
//   std::map<uint32_t, Vector> dirs = LoraEdsMonitor::GetDirectionList (tolerance);

//   uint16_t buckets[8] = {0,0,0,0,0,0,0,0};

//   for (std::map<uint32_t, Vector>::iterator it = dirs.begin (); it != dirs.end (); ++it) 
//   {
//     Vector curDir =  it -> second;
//     double theta = std::atan2 (curDir.y,curDir.x) * 180 / M_PI;

//     if (theta > 0 && theta < 45)
//       buckets[0]++;
//     else if (theta > 45 && theta <= 90)
//       buckets[1]++;
//     else if (theta > 90 && theta <= 135)
//       buckets[2]++;
//     else if (theta > 135 && theta <= 180)
//       buckets[3]++;
//     else if (theta > 180 && theta <= 225 )
//       buckets[4]++;
//     else if (theta > 225 && theta <= 270)
//       buckets[5]++;
//     else if (theta > 270 && theta <= 315)
//       buckets[6]++;
//     else
//       buckets[7]++;
//   }

//   uint16_t numClusters = 0;
//   for (uint16_t i = 0; i < 8; i++)
//   {
//     NS_LOG_INFO (i << ":" << buckets[i]);
//     if (buckets[i] > 0)
//       numClusters++;
//   }

//   NS_LOG_INFO ("Clusters=" << numClusters);
//   return numClusters;

// }
