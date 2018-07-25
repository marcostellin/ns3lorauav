#ifndef WEISSBERGER_PROPAGATION_LOSS_MODEL_H
#define WEISSBERGER_PROPAGATION_LOSS_MODEL_H

#include <ns3/propagation-loss-model.h>
#include "ns3/pointer.h"

namespace ns3 {

class WeissbergerPropagationLossModel : public PropagationLossModel
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  WeissbergerPropagationLossModel ();
  /**
   * \param frequency (Hz)
   *
   * Set the carrier frequency used in the model 
   * calculation.
   */
  void SetFrequency (double frequency);

  /* Set the depth of the foliage */

  void SetDepth (double depth);

  /* Set the three parameters of the Weissberger formula */

  void SetX (double x);
  void SetY (double y);
  void SetZ (double z);

  /*Get the three parameters of the Weissberger formula */

  double GetX (void) const;
  double GetY (void) const;
  double GetZ (void) const;

  /**
   * \returns the current frequency (Hz)
   */
  double GetFrequency (void) const;

  /* returns the current depth (m) */

  double GetDepth (void) const;

private:
  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   */
  WeissbergerPropagationLossModel (const WeissbergerPropagationLossModel &);
  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   * \returns
   */
  WeissbergerPropagationLossModel & operator = (const WeissbergerPropagationLossModel &);

  virtual double DoCalcRxPower (double txPowerDbm,
                                Ptr<MobilityModel> a,
                                Ptr<MobilityModel> b) const;
  virtual int64_t DoAssignStreams (int64_t stream);

  /**
   * Transforms a Dbm value to Watt
   * \param dbm the Dbm value
   * \return the Watts
   */
  double DbmToW (double dbm) const;

  /**
   * Transforms a Watt value to Dbm
   * \param w the Watt value
   * \return the Dbm
   */
  double DbmFromW (double w) const;

  double m_depth;        //!< the foliage depth
  double m_frequency;     //!< the carrier frequency
  double m_x;           //!< the x parameter
  double m_y;           //!< the x parameter
  double m_z;           //!< the x parameter
};

}

#endif 