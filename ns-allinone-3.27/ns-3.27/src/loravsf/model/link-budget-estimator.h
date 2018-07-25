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

#ifndef LINK_BUDGET_ESTIMATOR_H
#define LINK_BUDGET_ESTIMATOR_H

#include "ns3/object.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/mobility-model.h"

namespace ns3 {

/**
 * This class estimate the AtA and AtG link budget
 */

class LinkBudgetEstimator : public Object
{
public:

  static TypeId GetTypeId (void);
  LinkBudgetEstimator();
  virtual ~LinkBudgetEstimator();

  void SetAtaModel (Ptr<PropagationLossModel> model);
  void SetAtgModel (Ptr<PropagationLossModel> model);
  double GetAtaLinkBudget (Ptr<MobilityModel> mob1, Ptr<MobilityModel> mob2);
  double GetAtgLinkBudget (Ptr<MobilityModel> mob1, Ptr<MobilityModel> mob2);

private:

  Ptr<PropagationLossModel> m_ataModel;
  Ptr<PropagationLossModel> m_atgModel;
  double m_txAtaPower;
  double m_txAtgPower;
  double m_sensAta;
  double m_sensAtg;

};

}

#endif /* DEVICE_STATUS_H */
