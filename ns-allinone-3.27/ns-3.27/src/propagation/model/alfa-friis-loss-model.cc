#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/mobility-model.h"
#include <cmath>

#include "alfa-friis-loss-model.h"

namespace ns3 {

	NS_LOG_COMPONENT_DEFINE ("AlfaFriisPropagationLossModel");

	NS_OBJECT_ENSURE_REGISTERED (AlfaFriisPropagationLossModel);

	TypeId 
	AlfaFriisPropagationLossModel::GetTypeId (void)
	{
	  static TypeId tid = TypeId ("ns3::AlfaFriisPropagationLossModel")
	    .SetParent<PropagationLossModel> ()
	    .SetGroupName ("Propagation")
	    .AddConstructor<AlfaFriisPropagationLossModel> ()
        .AddAttribute ("Frequency", 
                   "The carrier frequency (in MHz) at which propagation occurs  (default is 1 GHz).",
                   DoubleValue (867),
                   MakeDoubleAccessor (&AlfaFriisPropagationLossModel::SetFrequency,
                                       &AlfaFriisPropagationLossModel::GetFrequency),
                   MakeDoubleChecker<double> ())


        .AddAttribute ("Alfa", 
                   "The alfa parameter. Default is 2 (Free Space)",
                   DoubleValue (0.75),
                   MakeDoubleAccessor (&AlfaFriisPropagationLossModel::SetAlfa,
                                       &AlfaFriisPropagationLossModel::GetAlfa),
                   MakeDoubleChecker<double> ())
	  ;
	  return tid;
	}
	AlfaFriisPropagationLossModel::AlfaFriisPropagationLossModel ()
	{
	}

	void
	AlfaFriisPropagationLossModel::SetFrequency (double frequency)
	{
		m_frequency = frequency;
		static const double C = 299792458.0; // speed of light in vacuum
        m_lambda = C / frequency;
	}

	void
	AlfaFriisPropagationLossModel::SetAlfa (double alfa)
	{
		m_alfa = alfa;
	}

	double
	AlfaFriisPropagationLossModel::GetAlfa (void) const
	{
		return m_alfa;
	}

	double
	AlfaFriisPropagationLossModel::GetFrequency (void) const
	{
		return m_frequency;
	}

	double
	AlfaFriisPropagationLossModel::DbmToW (double dbm) const
	{
	  double mw = std::pow (10.0,dbm/10.0);
	  return mw / 1000.0;
	}

	double
	AlfaFriisPropagationLossModel::DbmFromW (double w) const
	{
	  double dbm = std::log10 (w * 1000.0) * 10.0;
	  return dbm;
	}
    
	double 
    AlfaFriisPropagationLossModel::DoCalcRxPower (double txPowerDbm,
                                          Ptr<MobilityModel> a,
                                          Ptr<MobilityModel> b) const
    {
  
  	   double distance = a->GetDistanceFrom (b);

       double numerator = m_lambda * m_lambda;
       double denominator = 16 * M_PI * M_PI * std::pow (distance, m_alfa);
       double lossDb = -10 * log10 (numerator / denominator);
       NS_LOG_DEBUG ("distance=" << distance<< "m, loss=" << lossDb <<"dB");

       return txPowerDbm - lossDb;
    }

	int64_t
	AlfaFriisPropagationLossModel::DoAssignStreams (int64_t stream)
	{
	  return 0;
	}



}