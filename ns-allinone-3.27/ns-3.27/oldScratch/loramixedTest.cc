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
#include "ns3/v4ping.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/weissberger-loss-model.h"
#include <algorithm>
#include <ctime>


#include "ns3/internet-module.h"
#include "ns3/aodv-module.h"
#include "ns3/v4ping-helper.h"
#include "ns3/wifi-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("LoraDronesScenario");

std::string phyMode ("DsssRate11Mbps");
Ptr<Socket> source;


void ReceivePacket (Ptr<Socket> socket)
{
  while (socket->Recv ())
  {
      NS_LOG_UNCOND ("Received one packet!");
      NS_LOG_UNCOND (Simulator::Now().GetSeconds());
  }
}

static void SendUdpPacket (Ptr<Socket> socket, uint32_t pktSize)
{

      socket->Send (Create<Packet> (pktSize));
}

bool
ReceiveFromLora (Ptr<NetDevice> loraNetDevice, Ptr<const Packet>
                            packet, uint16_t protocol, const Address& sender)
{
  NS_LOG_UNCOND("ReceiveFromLora Callback!");
  NS_LOG_UNCOND(Simulator::Now().GetSeconds());

  Simulator::ScheduleNow (&SendUdpPacket, source, 20);

  return true;
}


int
main(int argc, char *argv[]){
   
  LogComponentEnable ("LoraDronesScenario", LOG_LEVEL_INFO);
  LogComponentEnable ("LoraChannel", LOG_LEVEL_DEBUG);
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
  //LogComponentEnable ("OneShotSender", LOG_LEVEL_INFO);
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
  
  Ptr<LogDistancePropagationLossModel> logloss = CreateObject<LogDistancePropagationLossModel> ();
  logloss->SetPathLossExponent (3.76);
  logloss->SetReference (1, 8.1);

  Ptr<WeissbergerPropagationLossModel> loss = CreateObject<WeissbergerPropagationLossModel> ();

  loss->SetDepth (14);
  loss->SetNext(logloss);

  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);
  
  /************************
  *  Create the helpers  *
  ************************/

  NS_LOG_INFO ("Setting up helpers...");

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> nodesAllocator = CreateObject<ListPositionAllocator> ();
  nodesAllocator->Add (Vector (0,50,0));
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
  gateways.Create (1);
  
  Ptr<ListPositionAllocator> gwAllocator = CreateObject<ListPositionAllocator> ();
  gwAllocator->Add(Vector(0,0,0));
  
  mobility.SetPositionAllocator (gwAllocator);
  mobility.Install (gateways);

  // Create a netdevice for each gateway
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LoraMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);

  macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);
  
  /*********************************************
  *  Install applications on the end devices  *
  *********************************************/

  //OneShotSenderHelper oneShotSenderHelper;
  //oneShotSenderHelper.SetSendTime (Seconds (15));

  //oneShotSenderHelper.Install (endDevices);
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  appHelper.SetPeriod (Seconds (1));
  ApplicationContainer appContainer = appHelper.Install (endDevices);

  appContainer.Start (Seconds (10));
  appContainer.Stop (Seconds(1000));

  /********************************************
  *  Configure the WiFi network               *
  ********************************************/

  NodeContainer wifiNodes;
  wifiNodes.Add (gateways.Get(0));
  wifiNodes.Create(2);

  NodeContainer groundStation;
  groundStation.Add(wifiNodes.Get(2)); 


  //Setup helpers
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add an upper mac and disable rate control
  WifiMacHelper wifiMac;
  WifiHelper wifi;

  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));

  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, wifiNodes);

  // Install mobility model

  Ptr<ListPositionAllocator> wifiAllocator = CreateObject<ListPositionAllocator> ();
  wifiAllocator->Add(Vector(0,0,0));
  wifiAllocator->Add(Vector(0,-100,0));
  wifiAllocator->Add(Vector(0,-20,0));
  mobility.SetPositionAllocator (wifiAllocator);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiNodes);

  AodvHelper aodv;
  //OlsrHelper aodv;
  InternetStackHelper stack;
  stack.SetRoutingHelper (aodv);
  stack.Install (wifiNodes);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (groundStation.Get (0), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

  source = Socket::CreateSocket (gateways.Get (0), tid);
  InetSocketAddress remote = InetSocketAddress (i.GetAddress (2, 0), 80);
  source->Connect (remote);

  //Set LoRa CallBack
  Ptr<Node> loraGateway = gateways.Get(0);
  Ptr<NetDevice> gatewayNetDevice = loraGateway->GetDevice (0);
  Ptr<LoraNetDevice> gatewayLoraNetDevice;
  if (gatewayNetDevice->GetObject<LoraNetDevice> () != 0)
  {
     gatewayLoraNetDevice = gatewayNetDevice->GetObject<LoraNetDevice> ();
     gatewayLoraNetDevice->SetReceiveCallback (MakeCallback(&ReceiveFromLora));
  } 


  /*
  V4PingHelper ping (interfaces.GetAddress (1));
  ping.SetAttribute ("Verbose", BooleanValue (true));
  ping.SetAttribute("Interval", TimeValue(Seconds(10.0)));
  ApplicationContainer p = ping.Install (wifiNodes.Get (0));
  p.Start(Seconds(10.0));
  p.Stop(Seconds(20.0));
  */

  
  /****************
  *  Simulation  *
  ****************/

  Ptr<FlowMonitor> flowMonitor;
  FlowMonitorHelper flowHelper;
  flowMonitor = flowHelper.Install(wifiNodes);


  Simulator::Stop (Hours (2));

  Simulator::Run ();

  flowMonitor->SerializeToXmlFile("Test.xml", false, false);

  Simulator::Destroy ();

  return 0;
  
  
}
