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
#include "firemen-mobility-model.h"
#include "ns3/enum.h"
#include "ns3/double.h"
#include "ns3/integer.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/boolean.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include <cmath>
#include <algorithm>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("FiremenMobilityModel");

NS_OBJECT_ENSURE_REGISTERED (FiremenMobilityModel);

TypeId
FiremenMobilityModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::FiremenMobilityModel")
    .SetParent<MobilityModel> ()
    .SetGroupName ("Mobility")
    .AddConstructor<FiremenMobilityModel> ()

    .AddAttribute ("PropagationArea",
                   "Area of spreading of fire",
                   RectangleValue(Rectangle(0,1000,0,1000)),
                   MakeRectangleAccessor (&FiremenMobilityModel::m_area),
                   MakeRectangleChecker ())

    .AddAttribute ("NumTeams",
                   "Total number of teams that are taking part in the operation",
                   IntegerValue (1),
                   MakeIntegerAccessor (&FiremenMobilityModel::m_numTeams),
                   MakeIntegerChecker<int> ())

    .AddAttribute ("TeamId",
                   "ID of the team. ID is progressive from 1 to NumTeams (which defaults to 1)",
                   IntegerValue (0),
                   MakeIntegerAccessor (&FiremenMobilityModel::m_teamId),
                   MakeIntegerChecker<int> ())

    .AddAttribute ("HeightWpArea",
                   "Height of subarea in which waypoints are chosen",
                   IntegerValue (100),
                   MakeIntegerAccessor (&FiremenMobilityModel::m_heightWpArea),
                   MakeIntegerChecker<int> ())

    .AddAttribute ("WaypointRv",
                   "Random Variable used to select waypoints",
                   StringValue ("ns3::UniformRandomVariable"),
                   MakePointerAccessor (&FiremenMobilityModel::m_wp),
                   MakePointerChecker<RandomVariableStream> ())

    .AddAttribute ("WalkTime",
                   "Time before changing direction when random walk",
                   TimeValue (Seconds(30)),
                   MakeTimeAccessor (&FiremenMobilityModel::m_walkTime),
                   MakeTimeChecker ())

    .AddAttribute ("FallbackTime",
                   "Time before falling back to another point of intervention",
                   TimeValue (Minutes(5)),
                   MakeTimeAccessor (&FiremenMobilityModel::m_fallTime),
                   MakeTimeChecker ())

    .AddAttribute ("SpreadY",
                   "Maximum spread of team in y direction (meters)",
                   DoubleValue (300),
                   MakeDoubleAccessor (&FiremenMobilityModel::m_spreadY),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("SpreadX",
                   "Maximum spread of team in x direction (meters)",
                   DoubleValue (300),
                   MakeDoubleAccessor (&FiremenMobilityModel::m_spreadX),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("Split",
                   "Split teams",
                   BooleanValue (false),
                   MakeBooleanAccessor (&FiremenMobilityModel::m_split),
                   MakeBooleanChecker ())

    .AddAttribute ("Speed",
                   "The speed of the firemen",
                   DoubleValue (1.39),
                   MakeDoubleAccessor (&FiremenMobilityModel::m_speed),
                   MakeDoubleChecker<double> ());     
                   
  return tid;
}

void
FiremenMobilityModel::DoInitialize (void)
{
  Ptr<FiremenMobilityModel> mob = Ptr<FiremenMobilityModel> (this);
  Ptr<Node> node = mob -> GetObject<Node> ();
  m_id = node -> GetId ();

  //m_split = true;

  m_minY = m_area.yMax;
  m_wp -> SetStream (m_teamId);
  m_cur = Seconds (0);

  m_direction = CreateObject<UniformRandomVariable> ();
  m_direction -> SetAttribute ("Min", DoubleValue(0));
  m_direction -> SetAttribute ("Max", DoubleValue(6.283184));

  Ptr<UniformRandomVariable> time = CreateObject<UniformRandomVariable> ();
  time -> SetStream (m_teamId + 10);
  time -> SetAttribute ("Min", DoubleValue(300));
  time -> SetAttribute ("Max", DoubleValue(600));

  m_fallTime = Seconds (time -> GetValue ());

  DoInitializePrivate ();
  MobilityModel::DoInitialize ();
}

void
FiremenMobilityModel::DoInitializePrivate (void)
{
  m_helper.Update();
  double speed = m_speed;

  //Select Waypoint
  double lengthSubArea = m_area.xMax / m_numTeams;
  double xLowLeft = m_teamId * lengthSubArea;
  double yLowLeft;

  if (m_minY - m_heightWpArea)
    yLowLeft = m_minY - m_heightWpArea;
  else
    yLowLeft = 0;

  m_minY = yLowLeft;

  double xUpRight = xLowLeft + lengthSubArea;
  double yUpRight = yLowLeft + m_heightWpArea;

  NS_LOG_DEBUG ("Assigned Area team " << m_teamId << " " << Rectangle(xLowLeft, xUpRight, yLowLeft, yUpRight));

  Vector waypoint;

  m_wp -> SetAttribute ("Min", DoubleValue(xLowLeft));
  m_wp -> SetAttribute ("Max", DoubleValue(xUpRight));

  waypoint.x = m_wp -> GetValue ();

  m_wp -> SetAttribute ("Min", DoubleValue(yLowLeft));
  m_wp -> SetAttribute ("Max", DoubleValue(yUpRight));

  waypoint.y = m_wp -> GetValue ();

  NS_LOG_DEBUG ("Current waypoint = " << waypoint);

  //Define bounds for random walk area
  m_bounds = Rectangle (waypoint.x - m_spreadX/2, 
                        waypoint.x + m_spreadX/2, 
                        waypoint.y - m_spreadY/2, 
                        waypoint.y + m_spreadY/2);

  Vector pos = m_helper.GetCurrentPosition ();
  double dist = CalculateDistance (waypoint, pos);
  NS_ASSERT (pos.z == 0);
  Vector vel = Vector ( speed * (waypoint.x - pos.x) / dist,
                        speed * (waypoint.y - pos.y) / dist,
                        0.0);
  
  //NS_LOG_DEBUG ("Velocity = " << vel);

  m_helper.SetVelocity (vel);
  m_helper.Unpause ();

  Time delayLeft = Seconds(dist / speed);

  DoWalk (delayLeft);
  
}

void
FiremenMobilityModel::ChangeDirection (Time delayLeft)
{
  m_helper.Update ();
  m_split = false;
  double speed = m_speed;
  m_direction -> SetStream (m_teamId);

  double direction = m_direction -> GetValue();

  m_direction -> SetStream (-1);

  Vector vel (std::cos (direction) * speed,
                 std::sin (direction) * speed,
                 0.0);
  m_helper.SetVelocity (vel);
  m_helper.Unpause ();

  Time left = delayLeft / 4;

  Vector pos = m_helper.GetCurrentPosition ();
  Vector nextPos = Vector (pos.x + vel.x * left.GetSeconds (), pos.y + vel.y * left.GetSeconds (), 0 );
  
  m_bounds = Rectangle (nextPos.x - m_spreadX/2, 
                        nextPos.x + m_spreadX/2, 
                        nextPos.y - m_spreadY/2, 
                        nextPos.y + m_spreadY/2);

  m_event.Cancel ();
  Simulator::Schedule (left, &FiremenMobilityModel::DoInitializePrivateRandomWalk, this);
}


void
FiremenMobilityModel::DoWalk (Time delayLeft)
{
  m_event.Cancel ();

  m_event = Simulator::Schedule (delayLeft, &FiremenMobilityModel::DoInitializePrivateRandomWalk, this);

  if (m_split && m_id % 2 == 0)
  {
    m_event.Cancel ();
    m_event = Simulator::Schedule (delayLeft / 2, &FiremenMobilityModel::ChangeDirection, this, delayLeft);
  }

  NotifyCourseChange ();

  /*
  Vector position = m_helper.GetCurrentPosition ();
  Vector speed = m_helper.GetVelocity ();
  Vector nextPosition = position;
  nextPosition.x += speed.x * delayLeft.GetSeconds ();
  nextPosition.y += speed.y * delayLeft.GetSeconds ();
  m_event.Cancel ();
  if (m_bounds.IsInside (nextPosition))
    {
      m_event = Simulator::Schedule (delayLeft, &FiremenMobilityModel::DoInitializePrivate, this);
    }
  else
    {
      nextPosition = m_bounds.CalculateIntersection (position, speed);
      Time delay = Seconds ((nextPosition.x - position.x) / speed.x);
      m_event = Simulator::Schedule (delay, &FiremenMobilityModel::Rebound, this,
                                     delayLeft - delay);
    }
  NotifyCourseChange ();
  */
}

/*
void
FiremenMobilityModel::Rebound (Time delayLeft)
{
  m_helper.UpdateWithBounds (m_bounds);
  Vector position = m_helper.GetCurrentPosition ();
  Vector speed = m_helper.GetVelocity ();
  switch (m_bounds.GetClosestSide (position))
    {
    case Rectangle::RIGHT:
    case Rectangle::LEFT:
      speed.x = -speed.x;
      break;
    case Rectangle::TOP:
    case Rectangle::BOTTOM:
      speed.y = -speed.y;
      break;
    }
  m_helper.SetVelocity (speed);
  m_helper.Unpause ();
  DoWalk (delayLeft);
  
}

*/

void 
FiremenMobilityModel::DoInitializePrivateRandomWalk (void) 
{
  m_helper.Update ();
  double speed = m_speed;
  double direction = m_direction -> GetValue();

  Vector vector (std::cos (direction) * speed,
                 std::sin (direction) * speed,
                 0.0);

  m_helper.SetVelocity (vector);
  m_helper.Unpause ();

  //NS_LOG_DEBUG ("Time passed " << m_cur.GetSeconds());

  if (m_cur > m_fallTime)
  {
    m_event.Cancel ();
    m_cur = Seconds (0);
    m_event = Simulator::ScheduleNow (&FiremenMobilityModel::DoInitializePrivate, this);
  }
  else
  {
    m_cur += m_walkTime;

    Time delayLeft = m_walkTime;

    DoWalkRandom (delayLeft);
  }
}

void
FiremenMobilityModel::DoWalkRandom (Time delayLeft)
{
  Vector position = m_helper.GetCurrentPosition ();
  Vector speed = m_helper.GetVelocity ();
  Vector nextPosition = position;
  nextPosition.x += speed.x * delayLeft.GetSeconds ();
  nextPosition.y += speed.y * delayLeft.GetSeconds ();
  
  m_event.Cancel ();

  //NS_LOG_DEBUG ("Bounds " << m_bounds);
  //NS_LOG_DEBUG ("Next Position " << nextPosition);

  if (m_bounds.IsInside (nextPosition))
  {
    m_event = Simulator::Schedule (delayLeft, &FiremenMobilityModel::DoInitializePrivateRandomWalk, this);
  }
  else
  {
    nextPosition = m_bounds.CalculateIntersection (position, speed);
    Time delay = Seconds ((nextPosition.x - position.x) / speed.x);
    m_event = Simulator::Schedule (delay, &FiremenMobilityModel::Rebound, this, delayLeft - delay);
  }
  
  NotifyCourseChange ();
}

void
FiremenMobilityModel::Rebound (Time delayLeft)
{
  m_helper.UpdateWithBounds (m_bounds);
  Vector position = m_helper.GetCurrentPosition ();
  Vector speed = m_helper.GetVelocity ();

  switch (m_bounds.GetClosestSide (position))
  {
    case Rectangle::RIGHT:
    case Rectangle::LEFT:
      speed.x = -speed.x;
      break;

    case Rectangle::TOP:
    case Rectangle::BOTTOM:
      speed.y = -speed.y;
      break;
  }
  
  m_helper.SetVelocity (speed);
  m_helper.Unpause ();
  DoWalkRandom (delayLeft);
}

void
FiremenMobilityModel::DoDispose (void)
{
  // chain up
  MobilityModel::DoDispose ();
}
Vector
FiremenMobilityModel::DoGetPosition (void) const
{
  m_helper.Update();
  return m_helper.GetCurrentPosition ();	
  /*
  m_helper.UpdateWithBounds (m_bounds);
  return m_helper.GetCurrentPosition ();
  */
}
void
FiremenMobilityModel::DoSetPosition (const Vector &position)
{ 
  m_helper.SetPosition (position);
  Simulator::Remove (m_event);
  m_event = Simulator::ScheduleNow (&FiremenMobilityModel::DoInitializePrivate, this);

  /*
  NS_ASSERT (m_bounds.IsInside (position));
  m_helper.SetPosition (position);
  Simulator::Remove (m_event);
  m_event = Simulator::ScheduleNow (&FiremenMobilityModel::DoInitializePrivate, this);
  */
}
Vector
FiremenMobilityModel::DoGetVelocity (void) const
{
  
  return m_helper.GetVelocity ();
  
}



} // namespace ns3
