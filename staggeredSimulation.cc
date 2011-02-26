/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include <iostream>
#include <fstream>

#include "ns3/simulator-module.h"
#include "ns3/node-module.h"
#include "ns3/core-module.h"
#include "ns3/wifi-module.h"
#include "ns3/helper-module.h"
#include "ns3/rectangle.h"
#include "ns3/mobility-module.h"
#include "ns3/ns2-mobility-helper.h"
#include <stdlib.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TapWifiVirtualMachineExample");

//static void
//CourseChange (std::ostream *os, std::string foo, Ptr<const MobilityModel> mobility)
//{
//  Vector pos = mobility->GetPosition (); // Get position
//  Vector vel = mobility->GetVelocity (); // Get velocity
//
//  // Prints position and velocities
//  *os << Simulator::Now () << " POS: x=" << pos.x << ", y=" << pos.y
//      << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
//      << ", z=" << vel.z << std::endl;
//}

// This is for ns-2 trace integration.  


// global variables relevant to the simulation to set up the nodes

#define MAX_NODES 50 
WifiHelper wifi;
NqosWifiMacHelper wifiMac;
YansWifiChannelHelper wifiChannel;
YansWifiPhyHelper wifiPhy;

int mobilityMode =-1; 
int terrainDimension=0;
int numberOfNodes=0;

Node node[MAX_NODES];					// array representing all the nodes in the system
MobilityHelper mobilityArray[MAX_NODES];		// mobility array 
ns3::Ptr<ns3::Node> smartNodePointer[MAX_NODES];	// array of smart pointers
Ptr<ListPositionAllocator> positionAlloc[MAX_NODES];	// array of position allocators 
NetDeviceContainer devices[MAX_NODES];			// array of network devices 
// Ns2MobilityHelper ns2mobility[MAX_NODES];		// array of ns2 mobility helpers TODO definitely tackle this problem but a while later.
TapBridgeHelper tapBridge[MAX_NODES];			// aarray of tap bridge helpers 

//std::string traceFile("/home/anirudh/Desktop/ns-3-dev/examples/tap/synthetic.trace");

void InitializeNetworkComponents() {
  
  // We're going to use 802.11p data channel ,so set up a wifi helper to reflect that.
  wifi = WifiHelper::Default ();
  wifi.SetStandard (WIFI_PHY_STANDARD_80211p_SCH); // 802.11p radios 
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("OfdmRate3MbpsBW10MHz")); // want to see if the range improves now 

  // No reason for pesky access points, so we'll use an ad-hoc network.
  wifiMac = NqosWifiMacHelper::Default ();
  wifiMac.SetType ("ns3::AdhocWifiMac");

  // Configure the physcial layer.
  wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy = YansWifiPhyHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());

  wifiPhy.Set ("TxPowerStart", DoubleValue(33));
  wifiPhy.Set ("TxPowerEnd", DoubleValue(33)); // changing the transmit power to refelct higher allowed transmit powers for 802.11p 

}

static void InitializeNode(int nodeId) {
 

   smartNodePointer[nodeId]= ns3::Ptr<ns3::Node>::Ptr(&node[nodeId]);
   devices[nodeId] = wifi.Install (wifiPhy, wifiMac, smartNodePointer[nodeId]); // connect node with phy and mac to create device
   // We need location information since we are talking about wifi, so add location information to the ghost nodes.
   
   if(mobilityMode==1) // constant position mobility fixed nodes
                       {
  
     std::cout<<"Terrain side is  " << terrainDimension<<std::endl;  
     positionAlloc[nodeId] = CreateObject<ListPositionAllocator> ();
   
     float randomxPosition=((float)rand()/RAND_MAX)*(terrainDimension) ; // All nodes are randomly distributed in a square of side terrainDimension
     float randomyPosition=((float)rand()/RAND_MAX)*(terrainDimension) ; // All nodes are randomly distributed in a square of side terrainDimension
     positionAlloc[nodeId]->Add (Vector (randomxPosition,randomyPosition, 0.0));

     //std::cout<<"Node : "<<i<<": Node position is x : "<< randomxPosition<<" y : "<<randomyPosition<<endl;
     mobilityArray[nodeId].SetPositionAllocator (positionAlloc[nodeId]);
     mobilityArray[nodeId].SetMobilityModel ("ns3::ConstantPositionMobilityModel");
   
   }
 
 
  else if(mobilityMode==2)  {// ie random mobility model
     std::cout<<"Terrain dimension is "<<terrainDimension<<std::endl;
    mobilityArray[nodeId].SetPositionAllocator ("ns3::GridPositionAllocator",
      "MinX", DoubleValue (0.0),
      "MinY", DoubleValue (0.0),
      "DeltaX", DoubleValue (1),
      "DeltaY", DoubleValue (1),
      "GridWidth", UintegerValue (1),
      "LayoutType", StringValue ("RowFirst"));
  
    mobilityArray[nodeId].SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
      "Bounds", RectangleValue (Rectangle (0, terrainDimension, 0, terrainDimension)));
  
  }


// else if(mobilityMode ==3) { // feed input from trace. 
//   Ns2MobilityHelper ns2mobility(traceFile);
// }

  // for now the above code is commented because of some issues with getting smart pointers to work 
//  Ns2MobilityHelper mobility(traceFile);

  mobilityArray[nodeId].Install (smartNodePointer[nodeId]); 

//  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",MakeBoundCallback (&CourseChange, &os));

  mobilityArray[nodeId].EnableAsciiAll(std::cout);

  // Use the TapBridgeHelper to connect to the pre-configured tap devices for 
  // the left side.  We go with "UseLocal" mode since the wifi devices do not
  // support promiscuous mode (because of their natures0.  This is a special
  // case mode that allows us to extend a linux bridge into ns-3 IFF we will
  // only see traffic from one other device on that bridge.  That is the case
  // for this configuration.

  tapBridge[nodeId].SetAttribute ("Mode", StringValue ("UseLocal")); // TODO Difference between useBridge and useLocal 
  char tapString[10];
  sprintf(tapString,"tap%d",nodeId);
  tapBridge[nodeId].SetAttribute ("DeviceName", StringValue (tapString));
  tapBridge[nodeId].Install (smartNodePointer[nodeId], devices[nodeId].Get (0));   // devices is a container but it contains only one object in this case
}




int main (int argc, char *argv[])
{
//  LogComponentEnable("YansWifiPhy",LOG_ALL);
//  LogComponentEnable("TapBridge",LOG_ALL);
//  LogComponentEnable("RealtimeSimulatorImpl",LOG_ALL);
  srand(time(NULL));
  CommandLine cmd;
  cmd.Parse (argc, argv);
  int simulationTime=0;
  if(argc < 5) {
	std::cout<<"Usage: Enter number of Nodes followed by the size of terrain followed by mobility mode "<<std::endl;
	std::cout<<"1. Constant position, 2. Mobile based on random walk, 3. Trace driven"<<std::endl;
	std::cout<<"Followed by simulation time "<<std::endl;
	exit(0);
  }
  else {
 	numberOfNodes=atoi(argv[1]);
	terrainDimension=atoi(argv[2]);
	mobilityMode=atoi(argv[3]);
	simulationTime=atoi(argv[4]);
  }
  std::string logFile("/home/anirudh/Desktop/ns-3-dev/examples/tap/mobility.log");
  std::ofstream os;
  os.open(logFile.c_str());
  // We are interacting with the outside, real, world.  This means we have to 
  // interact in real-time and therefore means we have to use the real-time
  // simulator and take the time to calculate checksums.
  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));
  // Install the wireless devices onto our ghost nodes.
  // TODO: Fix the UseLocal for br
  InitializeNetworkComponents();
  int i=0;
  for(i=0;i<numberOfNodes;i++) {
	 Simulator::Schedule (Seconds (0), &InitializeNode, i);
  }
  std::cout<<"Simulating for "<<simulationTime<<" seconds "<<std::endl;
  Simulator::Stop (Seconds (simulationTime));
  Simulator::Run();
  Simulator::Destroy ();
}
