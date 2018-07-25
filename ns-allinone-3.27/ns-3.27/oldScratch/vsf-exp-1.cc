#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/core-module.h"
#include "ns3/mobility-helper.h"
#include "ns3/node-container.h"
#include "ns3/position-allocator.h"
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/virtual-springs-2d-mobility-model.h"
#include "ns3/alfa-friis-loss-model.h"

#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/end-device-lora-mac.h"
#include "ns3/gateway-lora-mac.h"
#include "ns3/lora-helper.h"
#include "ns3/periodic-sender-helper.h"

#include "ns3/internet-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/wifi-module.h"
#include "ns3/packet-sink-helper.h"

#include "ns3/udp-relay-server-helper.h"

#include <algorithm>
#include <ctime>
#include <iostream>
#include <fstream>
#include <string>
#include <fstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("VsfExp1");

std::string filename;

std::string phyMode ("VhtMcs0");
uint32_t numEds = 10;
uint32_t numUavs = 4;


/************************************
* Callbacks                         *
************************************/

// enum PacketOutcome {
//   RECEIVED,
//   INTERFERED,
//   NO_MORE_RECEIVERS,
//   UNDER_SENSITIVITY,
//   UNSET
// };

struct PacketStatus {
  Ptr<Packet const> packet;
  uint32_t senderId;
  //int outcomeNumber;
  //std::vector<enum PacketOutcome> outcomes;
};

struct EdStatus
{
	uint32_t nSent = 0;
	uint32_t nRecv = 0;
	uint32_t nInterf = 0;
	uint32_t nNoRecvs = 0;
	uint32_t nUnderSens = 0;
};

std::map<Ptr<Packet const>, PacketStatus> packetTracker;
std::map<Ptr<Node>, EdStatus> edTracker;

void
TransmissionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // NS_LOG_INFO ("Transmitted a packet from device " << systemId);
  // Create a packetStatus
  // PacketStatus status;
  // status.packet = packet;
  // status.senderId = systemId;
  // status.outcomeNumber = 0;
  // status.outcomes = std::vector<enum PacketOutcome> (numUavs, UNSET);

  // packetTracker.insert (std::pair<Ptr<Packet const>, PacketStatus> (packet, status));
	PacketStatus pStatus;
	pStatus.packet = packet;
	pStatus.senderId = systemId;

	packetTracker.insert (std::pair<Ptr<Packet const>, PacketStatus> (packet, pStatus));

	std::map<Ptr<Node>, EdStatus>::iterator it = edTracker.find(NodeList::GetNode (systemId));

	if ( it == edTracker.end () )
	{
		EdStatus status;
		status.nSent += 1;

  	edTracker.insert (std::pair<Ptr<Node>, EdStatus> (NodeList::GetNode (systemId), status ));

  	return;
	}

  it -> second.nSent += 1;	

}

void
PacketReceptionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // Remove the successfully received packet from the list of sent ones
  // NS_LOG_INFO ("A packet was successfully received at gateway " << systemId);

  // std::map<Ptr<Packet const>, PacketStatus>::iterator it = packetTracker.find (packet);
  // (*it).second.outcomes.at (systemId - numEds) = RECEIVED;
  // (*it).second.outcomeNumber += 1;
  std::map<Ptr<Packet const>, PacketStatus>::iterator itPacket = packetTracker.find (packet);

  std::map<Ptr<Node>, EdStatus>::iterator it = edTracker.find(NodeList::GetNode ( (*itPacket).second.senderId) );
  (*it).second.nRecv += 1;
}

void
InterferenceCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // NS_LOG_INFO ("A packet was lost because of interference at gateway " << systemId);

  // std::map<Ptr<Packet const>, PacketStatus>::iterator it = packetTracker.find (packet);
  // (*it).second.outcomes.at (systemId - numEds) = INTERFERED;
  // (*it).second.outcomeNumber += 1;
  
  std::map<Ptr<Packet const>, PacketStatus>::iterator itPacket = packetTracker.find (packet);

  std::map<Ptr<Node>, EdStatus>::iterator it = edTracker.find(NodeList::GetNode ( (*itPacket).second.senderId) );
  (*it).second.nInterf += 1;

}

void
NoMoreReceiversCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // NS_LOG_INFO ("A packet was lost because there were no more receivers at gateway " << systemId);

  // std::map<Ptr<Packet const>, PacketStatus>::iterator it = packetTracker.find (packet);
  // (*it).second.outcomes.at (systemId - numEds) = NO_MORE_RECEIVERS;
  // (*it).second.outcomeNumber += 1;

  std::map<Ptr<Packet const>, PacketStatus>::iterator itPacket = packetTracker.find (packet);

  std::map<Ptr<Node>, EdStatus>::iterator it = edTracker.find(NodeList::GetNode ( (*itPacket).second.senderId) );
  (*it).second.nNoRecvs += 1;

}

void
UnderSensitivityCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // NS_LOG_INFO ("A packet arrived at the gateway under sensitivity at gateway " << systemId);

  // std::map<Ptr<Packet const>, PacketStatus>::iterator it = packetTracker.find (packet);
  // (*it).second.outcomes.at (systemId - numEds) = UNDER_SENSITIVITY;
  // (*it).second.outcomeNumber += 1;

  std::map<Ptr<Packet const>, PacketStatus>::iterator itPacket = packetTracker.find (packet);

  std::map<Ptr<Node>, EdStatus>::iterator it = edTracker.find(NodeList::GetNode ( (*itPacket).second.senderId) );
  (*it).second.nUnderSens += 1;

}

bool
ReceiveFromLora (Ptr<NetDevice> loraNetDevice, Ptr<const Packet> packet, uint16_t protocol, const Address& sender)
{
  NS_LOG_DEBUG("ReceiveFromLora Callback!");
  NS_LOG_DEBUG(Simulator::Now().GetSeconds());
  
  Ptr<Node> node = loraNetDevice ->GetNode();
  Ptr<UdpRelay> relay = DynamicCast<UdpRelay>(node -> GetApplication(0));


  relay -> Send();

  return true;
}



int
main (int argc, char* argv[]){

  /*****************************
  * LOGGING                    *
  *****************************/
	//LogComponentEnable("UdpRelay", LOG_LEVEL_INFO);
	//LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
  //LogComponentEnable ("MobilityTest", LOG_LEVEL_INFO);
  //LogComponentEnable ("MobilityHelper", LOG_LEVEL_DEBUG);
  //LogComponentEnable ("EndDeviceLoraPhy", LOG_LEVEL_INFO);
  //LogComponentEnable ("GatewayLoraPhy", LOG_LEVEL_INFO);
  //LogComponentEnable ("GatewayLoraMac", LOG_LEVEL_INFO);
  //LogComponentEnable ("ConstantVelocityHelper", LOG_LEVEL_DEBUG);
  //LogComponentEnable ("VirtualSprings2d", LOG_LEVEL_DEBUG);
  //LogComponentEnable ("PropagationLossModel", LOG_LEVEL_DEBUG);

	LogComponentEnableAll (LOG_PREFIX_FUNC);
  LogComponentEnableAll (LOG_PREFIX_NODE);
  LogComponentEnableAll (LOG_PREFIX_TIME);

  filename = "nodes-stats.out.txt";
  std::ofstream out(filename);
  std::streambuf *coutbuf = std::cout.rdbuf();
  std::cout.rdbuf(out.rdbuf());
  
  /***********************
  * Set params           *
  ***********************/

  CommandLine cmd;
  cmd.AddValue ("NumED", "Number of end devices", numEds);
  cmd.AddValue("NumUAVs", "Number of UAVs", numUavs);
  cmd.Parse (argc, argv);

  /************************
  *  Create the channels  *
  ************************/

  NS_LOG_INFO ("Creating the AtG channel...");

  Ptr<LogDistancePropagationLossModel> medLoss = CreateObject<LogDistancePropagationLossModel> ();
  medLoss -> SetReference(1, 31.23);
  medLoss -> SetPathLossExponent(5.2);

  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (medLoss, delay);

  /************************
  *  Create the helpers  *
  ************************/

  NS_LOG_INFO ("Setting up helpers...");

  MobilityHelper mobility;

  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

  // Create the LoraMacHelper
  LoraMacHelper macHelper = LoraMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue(5.0e+09));
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add an upper mac and disable rate control
  WifiMacHelper wifiMac;
  WifiHelper wifi;

  wifi.SetStandard (WIFI_PHY_STANDARD_80211ac);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));

  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac", "VhtSupported", BooleanValue(true));


  /*******************
  * Create EDs       *
  *******************/

  NS_LOG_INFO ("Creating end devices...");

  // Create a set of nodes
  NodeContainer endDevices;
  endDevices.Create (numEds);

  // Assign a mobility model to the node
  Ptr<UniformDiscPositionAllocator> edsAllocator = CreateObject<UniformDiscPositionAllocator> ();
  edsAllocator->SetRho(200);
  edsAllocator->SetX(200.0);
  edsAllocator->SetY(200.0);

  mobility.SetPositionAllocator (edsAllocator);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (endDevices);

  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LoraMacHelper::ED);
  helper.Install (phyHelper, macHelper, endDevices);

  /************************
  *  Create UAVs          *
  ************************/

  NS_LOG_INFO ("Creating UAV devices...");

  // Create a set of nodes
  NodeContainer ataNodes;
  ataNodes.Create (4);

  //Assign mobility model to nodes

  Ptr<ListPositionAllocator> nodesAllocator = CreateObject<ListPositionAllocator> ();
  nodesAllocator->Add(Vector(200,300,100));
  nodesAllocator->Add(Vector(100,100,100));
  nodesAllocator->Add(Vector(200,100,100));
  nodesAllocator->Add(Vector(100,200,100));

  mobility.SetPositionAllocator (nodesAllocator);
  mobility.SetMobilityModel ("ns3::VirtualSprings2dMobilityModel", 
  																	"Time", TimeValue(Seconds(0.5)),
  																	"Tolerance", DoubleValue(20.0));
  mobility.Install (ataNodes);
  
  // Setup Nodes in VSF mobility model
  for (NodeContainer::Iterator j = ataNodes.Begin (); j != ataNodes.End (); ++j)
  {
    Ptr<Node> node = *j;
    Ptr<VirtualSprings2dMobilityModel> model = node -> GetObject<VirtualSprings2dMobilityModel>();

    for (NodeContainer::Iterator i = ataNodes.Begin (); i != ataNodes.End (); ++i)
    {
      Ptr<Node> nodeToAdd = *i;
      if (nodeToAdd -> GetId() != node->GetId()){
        model -> AddAtaNode(nodeToAdd->GetId());
      }
    }

    for (NodeContainer::Iterator i = endDevices.Begin (); i != endDevices.End (); ++i)
    {
      Ptr<Node> nodeToAdd = *i;
      model -> AddAtgNode(nodeToAdd -> GetId());
    }
  }

  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LoraMacHelper::GW);
  helper.Install (phyHelper, macHelper, ataNodes);

  /*******************
  * Create BS        *
  *******************/

  NodeContainer bsNodes;
  bsNodes.Create(1);

  Ptr<ListPositionAllocator> bsAllocator = CreateObject<ListPositionAllocator>();
  bsAllocator -> Add(Vector(200,200,0));

  mobility.SetPositionAllocator(bsAllocator);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(bsNodes);


  /*******************
  * Install devices **
  *******************/
  NodeContainer wifiNodes;
  wifiNodes.Add(bsNodes);
  wifiNodes.Add(ataNodes);

  NetDeviceContainer wifiDevices = wifi.Install (wifiPhy, wifiMac, wifiNodes);


  /************************
  * Set Routing           *
  ************************/
  
  DsdvHelper dsdv;
  //OlsrHelper aodv;
  InternetStackHelper stack;
  stack.SetRoutingHelper (dsdv);
  stack.Install (wifiNodes);
  

  /*************************
  * Install Internet Stack *
  *************************/
  
  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses to WiFi nodes...");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (wifiDevices);

  /************************
  * Install applications  *
  ************************/

  Address serverAddress = Address(i.GetAddress(0));
  NS_LOG_INFO("Server Address:");
  NS_LOG_INFO(serverAddress);
  

  uint16_t port = 4000;
  PacketSinkHelper server ("ns3::UdpSocketFactory", InetSocketAddress(i.GetAddress(0), port));
  ApplicationContainer apps = server.Install (bsNodes.Get (0));
  apps.Start (Seconds (1.0));
  apps.Stop (Minutes (10.0));


  uint32_t MaxPacketSize = 1024;
  uint32_t maxPacketCount = 320;
  UdpRelayHelper client (serverAddress, port);
  client.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
  client.SetAttribute ("PacketSize", UintegerValue (MaxPacketSize));
  apps = client.Install (ataNodes);
  apps.Start (Seconds (2.0));
  apps.Stop (Minutes (10.0));

  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  appHelper.SetPeriod (Seconds (20.0));
  apps = appHelper.Install (endDevices);

  apps.Start (Seconds (2));
  apps.Stop (Minutes (10)); 


  /*******************************
  * Set relay callback           *
  *******************************/
  for (NodeContainer::Iterator j = ataNodes.Begin (); j != ataNodes.End (); ++j)
  {
    Ptr<Node> source = *j;
    Ptr<NetDevice> sourceNetDevice = source->GetDevice (0);
    Ptr<LoraNetDevice> sourceLoraNetDevice;
    if (sourceNetDevice->GetObject<LoraNetDevice> () != 0)
    {
      sourceLoraNetDevice = sourceNetDevice->GetObject<LoraNetDevice> ();
      sourceLoraNetDevice->SetReceiveCallback (MakeCallback(&ReceiveFromLora));
    } 
  }

  /******************************
  * Set LoRa callbacks          *
  ******************************/
  
  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
  {
    Ptr<Node> node = *j;
    Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
    Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
    phy->TraceConnectWithoutContext ("StartSending",
                                       MakeCallback (&TransmissionCallback));
  }

  for (NodeContainer::Iterator j = ataNodes.Begin (); j != ataNodes.End (); j++)
  {

    Ptr<Node> object = *j;

    Ptr<NetDevice> netDevice = object->GetDevice (0);
    Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
    NS_ASSERT (loraNetDevice != 0);
    Ptr<GatewayLoraPhy> gwPhy = loraNetDevice->GetPhy ()->GetObject<GatewayLoraPhy> ();

    // Global callbacks (every gateway)
    gwPhy->TraceConnectWithoutContext ("ReceivedPacket",
                                         MakeCallback (&PacketReceptionCallback));
    gwPhy->TraceConnectWithoutContext ("LostPacketBecauseInterference",
                                         MakeCallback (&InterferenceCallback));
    gwPhy->TraceConnectWithoutContext ("LostPacketBecauseNoMoreReceivers",
                                         MakeCallback (&NoMoreReceiversCallback));
    gwPhy->TraceConnectWithoutContext ("LostPacketBecauseUnderSensitivity",
                                         MakeCallback (&UnderSensitivityCallback));
  }

  macHelper.SetSpreadingFactorsUp (endDevices, ataNodes, channel);
  
   
  /********************
  * RUN SIMULATION    *
  ********************/ 

  Simulator::Stop (Seconds (1000));
  
  /*******************
  * Netanim config   *
  *******************/

  AnimationInterface anim ("uav-scenario.xml");

  uint32_t edIcon = anim.AddResource("/home/letal32/workspace/ns-allinone-3.27/ns-3.27/images/fireman.png");
  uint32_t uavIcon = anim.AddResource("/home/letal32/workspace/ns-allinone-3.27/ns-3.27/images/drone.png");
  uint32_t bsIcon = anim.AddResource("/home/letal32/workspace/ns-allinone-3.27/ns-3.27/images/bs.png");

  for (uint32_t i = 0; i < endDevices.GetN (); ++i)
  {
      anim.UpdateNodeDescription (endDevices.Get (i), "ED"); // Optional
      anim.UpdateNodeColor (endDevices.Get (i), 255, 0, 0); // Optional
      anim.UpdateNodeSize (endDevices.Get (i)-> GetId(), 50, 50);
      anim.UpdateNodeImage (endDevices.Get (i)-> GetId(), edIcon);
  }
  for (uint32_t i = 0; i < bsNodes.GetN (); ++i)
  {
      anim.UpdateNodeDescription (bsNodes.Get (i), "BS"); // Optional
      anim.UpdateNodeColor (bsNodes.Get (i), 0, 255, 0); // Optional
      anim.UpdateNodeSize (bsNodes.Get (i)-> GetId(), 50, 50);
      anim.UpdateNodeImage (bsNodes.Get (i)-> GetId(), bsIcon);
  }
  for (uint32_t i = 0; i < ataNodes.GetN (); ++i)
  {
      anim.UpdateNodeDescription (ataNodes.Get (i), "UAV"); // Optional
      anim.UpdateNodeColor (ataNodes.Get (i), 0, 0, 255); // Optional
      anim.UpdateNodeSize (ataNodes.Get (i) -> GetId(), 50, 50);
      anim.UpdateNodeImage (ataNodes.Get (i)-> GetId(), uavIcon); 
  }

  anim.SetBackgroundImage("background.jpg", 0, 0, 0.5, 0.5, 0.8);

  Simulator::Run ();

  Simulator::Destroy ();


  /************************
  * Save/Print Stats      *
  ************************/
  std::cout << "id,nSent,nRecv,nInterf,nNoRecvs,nUnderSens" << std::endl;

  for (std::map<Ptr<Node>, EdStatus>::iterator it=edTracker.begin(); it!=edTracker.end(); ++it)
  {
    std::cout << it -> first -> GetId () << "," << 
    						 it -> second.nSent  << "," <<  
    						 it -> second.nRecv << "," <<
    						 it -> second.nInterf << "," <<
    						 it -> second.nNoRecvs << "," <<
    						 it -> second.nUnderSens << std::endl;
  }

  std::cout.rdbuf(coutbuf);

  

  return 0;
}
