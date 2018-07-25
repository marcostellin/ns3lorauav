#ifndef AlfaFriis_PROPAGATION_LOSS_MODEL_H
#define AlfaFriis_PROPAGATION_LOSS_MODEL_H

#include <ns3/propagation-loss-model.h>
#include "ns3/pointer.h"

namespace ns3 {

class AlfaFriisPropagationLossModel : public PropagationLossModel
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  AlfaFriisPropagationLossModel ();
  /**
   * \param frequency (Hz)
   *
   * Set the carrier frequency used in the model 
   * calculation.
   */
  void SetFrequency (double frequency);

  /* Set the depth of the foliage */

  void SetDepth (double depth);

  /* Set the alfa parameter */

  void SetAlfa (double alfa);

  /*Get the alfa parameter */

  double GetAlfa (void) const;

  /**
   * \returns the current frequency (Hz)
   */
  double GetFrequency (void) const;


private:
  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   */
  AlfaFriisPropagationLossModel (const AlfaFriisPropagationLossModel &);
  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   * \returns
   */
  AlfaFriisPropagationLossModel & operator = (const AlfaFriisPropagationLossModel &);

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

  double m_frequency;     //!< the carrier frequency
  double m_lambda;
  double m_alfa;           //!< the alfa parameter

};

}

#endif 