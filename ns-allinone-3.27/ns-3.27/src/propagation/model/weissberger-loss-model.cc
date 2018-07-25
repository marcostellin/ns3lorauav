#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/mobility-model.h"
#include <cmath>

#include "weissberger-loss-model.h"

namespace ns3 {

	NS_LOG_COMPONENT_DEFINE ("WeissbergerPropagationLossModel");

	NS_OBJECT_ENSURE_REGISTERED (WeissbergerPropagationLossModel);

	TypeId 
	WeissbergerPropagationLossModel::GetTypeId (void)
	{
	  static TypeId tid = TypeId ("ns3::WeissbergerPropagationLossModel")
	    .SetParent<PropagationLossModel> ()
	    .SetGroupName ("Propagation")
	    .AddConstructor<WeissbergerPropagationLossModel> ()
        .AddAttribute ("Frequency", 
                   "The carrier frequency (in MHz) at which propagation occurs  (default is 1 GHz).",
                   DoubleValue (867),
                   MakeDoubleAccessor (&WeissbergerPropagationLossModel::SetFrequency,
                                       &WeissbergerPropagationLossModel::GetFrequency),
                   MakeDoubleChecker<double> ())

        .AddAttribute ("Depth", 
                   "The depth of the foliage in meters. Default is 15m. Maximum value is 400m",
                   DoubleValue (15),
                   MakeDoubleAccessor (&WeissbergerPropagationLossModel::SetDepth,
                                       &WeissbergerPropagationLossModel::GetDepth),
                   MakeDoubleChecker<double> ())

        .AddAttribute ("X", 
                   "The X parameter. Default is 0.75",
                   DoubleValue (0.75),
                   MakeDoubleAccessor (&WeissbergerPropagationLossModel::SetX,
                                       &WeissbergerPropagationLossModel::GetX),
                   MakeDoubleChecker<double> ())

        .AddAttribute ("Y", 
                   "The Y parameter. Default is 0.25",
                   DoubleValue (0.25),
                   MakeDoubleAccessor (&WeissbergerPropagationLossModel::SetY,
                                       &WeissbergerPropagationLossModel::GetY),
                   MakeDoubleChecker<double> ())

        .AddAttribute ("Z", 
                   "The Z parameter. Default is 0.35",
                   DoubleValue (0.35),
                   MakeDoubleAccessor (&WeissbergerPropagationLossModel::SetZ,
                                       &WeissbergerPropagationLossModel::GetZ),
                   MakeDoubleChecker<double> ())
	  ;
	  return tid;
	}
	WeissbergerPropagationLossModel::WeissbergerPropagationLossModel ()
	{
	}

	void
	WeissbergerPropagationLossModel::SetDepth (double depth)
	{
		m_depth = depth;
	}

	double
	WeissbergerPropagationLossModel::GetDepth (void) const
	{
		return m_depth;
	}

	void
	WeissbergerPropagationLossModel::SetFrequency (double frequency)
	{
		m_frequency = frequency;
	}

	void
	WeissbergerPropagationLossModel::SetX (double x)
	{
		m_x = x;
	}

	void
	WeissbergerPropagationLossModel::SetY (double y)
	{
		m_y = y;
	}

	void
	WeissbergerPropagationLossModel::SetZ (double z)
	{
		m_z = z;
	}

	double
	WeissbergerPropagationLossModel::GetX (void) const
	{
		return m_x;
	}

	double
	WeissbergerPropagationLossModel::GetY (void) const
	{
		return m_y;
	}

	double
	WeissbergerPropagationLossModel::GetZ (void) const
	{
		return m_z;
	}

	double
	WeissbergerPropagationLossModel::GetFrequency (void) const
	{
		return m_frequency;
	}

	double
	WeissbergerPropagationLossModel::DbmToW (double dbm) const
	{
	  double mw = std::pow (10.0,dbm/10.0);
	  return mw / 1000.0;
	}

	double
	WeissbergerPropagationLossModel::DbmFromW (double w) const
	{
	  double dbm = std::log10 (w * 1000.0) * 10.0;
	  return dbm;
	}

	double
	WeissbergerPropagationLossModel::DoCalcRxPower (double txPowerDbm,
													Ptr<MobilityModel> a,
													Ptr<MobilityModel> b) const
	{
		if (m_depth >= 400){
			NS_LOG_WARN ("The depth of the foliage is too high. Inaccurate propagation loss");
		}

		double lossDb;

		lossDb = m_x * std::pow (m_frequency, m_y) * std::pow (m_depth, m_z);		

		NS_LOG_DEBUG ("lossDB=" << lossDb << "Foliage depth=" << m_depth << "Frequency=" << m_frequency);

		return txPowerDbm - lossDb;

	}

	int64_t
	WeissbergerPropagationLossModel::DoAssignStreams (int64_t stream)
	{
	  return 0;
	}



}