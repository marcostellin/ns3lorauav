/*
 * This script simulates a complex scenario with multiple gateways and end
 * devices. The metric of interest for this script is the throughput of the
 * network.
 */

#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/end-device-lora-mac.h"
#include "ns3/gateway-lora-mac.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/node-container.h"
#include "ns3/position-allocator.h"
#include "ns3/one-shot-sender-helper.h"
#include "ns3/command-line.h"
#include <algorithm>
#include <ctime>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("LoraDronesScenario");

int
main(int argc, char *argv[]){
   
  LogComponentEnable ("LoraDronesScenario", LOG_LEVEL_INFO);
  //LogComponentEnable ("LoraChannel", LOG_LEVEL_INFO);
  //LogComponentEnable ("LoraPhy", LOG_LEVEL_INFO);
  LogComponentEnable ("EndDeviceLoraPhy", LOG_LEVEL_INFO);
  LogComponentEnable ("GatewayLoraPhy", LOG_LEVEL_INFO);
  //LogComponentEnable ("LoraInterferenceHelper", LOG_LEVEL_INFO);
  //LogComponentEnable ("LoraMac", LOG_LEVEL_INFO);
  //LogComponentEnable ("EndDeviceLoraMac", LOG_LEVEL_INFO);
  LogComponentEnable ("GatewayLoraMac", LOG_LEVEL_INFO);
  //LogComponentEnable ("LogicalLoraChannelHelper", LOG_LEVEL_INFO);
  //LogComponentEnable ("LogicalLoraChannel", LOG_LEVEL_INFO);
  //LogComponentEnable ("LoraHelper", LOG_LEVEL_INFO);
  //LogComponentEnable ("LoraPhyHelper", LOG_LEVEL_INFO);
  //LogComponentEnable ("LoraMacHelper", LOG_LEVEL_INFO);
  //LogComponentEnable ("OneShotSenderHelper", LOG_LEVEL_INFO);
  LogComponentEnable ("OneShotSender", LOG_LEVEL_INFO);
  //LogComponentEnable ("LoraMacHeader", LOG_LEVEL_INFO);
  //LogComponentEnable ("LoraFrameHeader", LOG_LEVEL_INFO);
  LogComponentEnableAll (LOG_PREFIX_FUNC);
  LogComponentEnableAll (LOG_PREFIX_NODE);
  LogComponentEnableAll (LOG_PREFIX_TIME);
  
  
  /************************
  *  Create the channel  *
  ************************/

  NS_LOG_INFO ("Creating the channel...");

  // Create the lora channel object. The Lora channel has a LogDistance propagation loss and a constant delay (default to speed of light).
  
  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  loss->SetPathLossExponent (3.76);
  loss->SetReference (1, 8.1);

  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);
  
  /************************
  *  Create the helpers  *
  ************************/

  NS_LOG_INFO ("Setting up helpers...");

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> nodesAllocator = CreateObject<ListPositionAllocator> ();
  nodesAllocator->Add (Vector (0,50,0));
  nodesAllocator->Add (Vector (50,0,0));
  nodesAllocator->Add (Vector (-50,0,0));
  nodesAllocator->Add (Vector (0,-50,0));
  mobility.SetPositionAllocator (nodesAllocator);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

  // Create the LoraMacHelper
  LoraMacHelper macHelper = LoraMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();
  
  
  /************************
  *  Create End Devices  *
  ************************/

  NS_LOG_INFO ("Creating end devices...");

  // Create a set of nodes
  NodeContainer endDevices;
  endDevices.Create (1);

  // Assign a mobility model to the node
  mobility.Install (endDevices);

  // Create the LoraNetDevices of the end devices
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LoraMacHelper::ED);
  helper.Install (phyHelper, macHelper, endDevices);
  
  /*********************
  *  Create Gateways  *
  *********************/

  NS_LOG_INFO ("Creating the gateway...");
  NodeContainer gateways;
  gateways.Create (2);
  
  Ptr<ListPositionAllocator> gwAllocator = CreateObject<ListPositionAllocator> ();
  gwAllocator->Add(Vector(0,0,0));
  
  mobility.SetPositionAllocator (gwAllocator);
  mobility.Install (gateways);

  // Create a netdevice for each gateway
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LoraMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);
  
  /*********************************************
  *  Install applications on the end devices  *
  *********************************************/

  OneShotSenderHelper oneShotSenderHelper;
  oneShotSenderHelper.SetSendTime (Seconds (10));

  oneShotSenderHelper.Install (endDevices);
  
  /****************
  *  Simulation  *
  ****************/

  Simulator::Stop (Hours (2));

  Simulator::Run ();

  Simulator::Destroy ();

  return 0;
  
  
}
