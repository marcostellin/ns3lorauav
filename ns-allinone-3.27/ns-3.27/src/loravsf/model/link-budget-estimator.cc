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

#include "ns3/link-budget-estimator.h"
#include "ns3/log.h"
#include "ns3/node-list.h"
#include "ns3/double.h"
#include <algorithm>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LinkBudgetEstimator");
NS_OBJECT_ENSURE_REGISTERED (LinkBudgetEstimator);

TypeId
LinkBudgetEstimator::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LinkBudgetEstimator")
    .SetParent<Object> ()
    .AddConstructor<LinkBudgetEstimator> ()

    .AddAttribute ("TxPowerAta",
                   "Transmission power of Ata nodes (dBm)",
                   DoubleValue (16.0),
                   MakeDoubleAccessor (&LinkBudgetEstimator::m_txAtaPower),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("TxPowerAtg",
                   "Transmission power of Atg nodes (dBm)",
                   DoubleValue (14.0),
                   MakeDoubleAccessor (&LinkBudgetEstimator::m_txAtgPower),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("SensitivityAtg",
                   "Sensitivity of AtG receiver",
                   DoubleValue (-137.0),
                   MakeDoubleAccessor (&LinkBudgetEstimator::m_sensAtg),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("SensitivityAta",
                   "Sensitivity of Ata receiver",
                   DoubleValue (-91.0),
                   MakeDoubleAccessor (&LinkBudgetEstimator::m_sensAta),
                   MakeDoubleChecker<double> ());

    return tid;
}


LinkBudgetEstimator::LinkBudgetEstimator ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

LinkBudgetEstimator::~LinkBudgetEstimator ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void
LinkBudgetEstimator::SetAtaModel (Ptr<PropagationLossModel> model)
{
  m_ataModel = model;
}

void
LinkBudgetEstimator::SetAtgModel (Ptr<PropagationLossModel> model)
{
  m_atgModel = model;
}

double
LinkBudgetEstimator::GetAtaLinkBudget (Ptr<MobilityModel> mob1, Ptr<MobilityModel> mob2)
{
  double rxPower = m_ataModel -> CalcRxPower (m_txAtaPower, mob1, mob2);

  return rxPower - m_sensAta;
}

double
LinkBudgetEstimator::GetAtgLinkBudget (Ptr<MobilityModel> mob1, Ptr<MobilityModel> mob2)
{
  double rxPower = m_atgModel -> CalcRxPower (m_txAtgPower, mob1, mob2);

  NS_LOG_DEBUG ("RxPower = " << rxPower << " TxPower = " << m_txAtgPower << " Sensitivity = " << m_sensAtg );

  return rxPower - m_sensAtg;
}

}
