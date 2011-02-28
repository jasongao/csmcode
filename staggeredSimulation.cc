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
#include "ns3/simulator.h"
#include "ns3/nstime.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TapWifiVirtualMachineExample");

static void
CourseChange (std::ostream *os, std::string foo, Ptr<const MobilityModel> mobility)
{
  Vector pos = mobility->GetPosition (); // Get position
  Vector vel = mobility->GetVelocity (); // Get velocity

  // Prints position and velocities
  *os << Simulator::Now () << " POS: x=" << pos.x << ", y=" << pos.y
      << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
      << ", z=" << vel.z << std::endl;
}

// This is for ns-2 trace integration.  


// global variables relevant to the simulation to set up the nodes

#define MAX_NODES 60

WifiHelper wifi;
NqosWifiMacHelper wifiMac;
YansWifiChannelHelper wifiChannel;
YansWifiPhyHelper wifiPhy;
NetDeviceContainer device;		// ns-3's representation of the device running on this node
TapBridgeHelper tapBridge;		// ns-3's representation of the tap bridge that this node connects to. 
MobilityHelper mobility;		// ns-3's representation of the node's mobility

int mobilityMode =-1; 
int terrainDimension=0;
int numberOfNodes=0;
NodeContainer nodes;




void InitializeNetworkComponents() {
  
  // We're going to use 802.11p data channel ,so set up a wifi helper to reflect that.
  wifi = WifiHelper::Default ();
  wifi.SetStandard (WIFI_PHY_STANDARD_80211p_SCH); // 802.11p radios , tweaking this to 802.11a to see if it works out 
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


static void InitializeNetworkedMobileNode(int nodeID) {
	
	std::cout<<"In Initialize Node on node ID  "<<nodeID<<std::endl;
	Ptr<Node> smartNodePointer;
	smartNodePointer=nodes.Get(nodeID);

	device = wifi.Install (wifiPhy, wifiMac, smartNodePointer); // connect node with phy and mac to create device
	if(mobilityMode==1) { // constant position mobility fixed nodes
	   // std::cout<<"Terrain side is  " << terrainDimension<<std::endl;  
   	     Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

	     float randomxPosition=((float)rand()/RAND_MAX)*(terrainDimension) ; // All nodes are randomly distributed in a square of side terrainDimension
	     float randomyPosition=((float)rand()/RAND_MAX)*(terrainDimension) ; // All nodes are randomly distributed in a square of side terrainDimension
	     positionAlloc->Add (Vector (randomxPosition,randomyPosition, 0.0));
	 
	     //std::cout<<"Node : "<<i<<": Node position is x : "<< randomxPosition<<" y : "<<randomyPosition<<endl;
	     mobility.SetPositionAllocator (positionAlloc);
	     mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	     mobility.Install(smartNodePointer);
	     mobility.EnableAscii(std::cout,nodeID);
        }
	
	else if (mobilityMode ==2) {

     //        mobility.Install(smartNodePointer); 
//	     mobility.EnableAscii(std::cout,nodeID);
	}

	// TODO: Add the option to feed in traces from ns-2 

	char tapString[10];
	sprintf(tapString,"tap%d",nodeID);
	tapBridge.SetAttribute ("Mode", StringValue ("UseLocal"));
	tapBridge.SetAttribute ("DeviceName", StringValue (tapString));
	tapBridge.Install (smartNodePointer, device.Get (0));   // devices is a container but it contains only one object in this case
}



int main (int argc, char *argv[])
{
//  LogComponentEnable("YansWifiPhy",LOG_ALL);
//  LogComponentEnable("TapBridge",LOG_ALL);
//  LogComponentEnable("RealtimeSimulatorImpl",LOG_ALL);
  srand(time(NULL));
  CommandLine cmd;
  cmd.Parse (argc, argv);
  std::string traceFile; 
 
  int simulationTime=0;
  if(argc < 5) {
	std::cout<<"Usage: Enter number of Nodes followed by the size of terrain followed by mobility mode "<<std::endl;
	std::cout<<"1. Constant position, 2. Mobile based on random walk, 3. Trace driven"<<std::endl;
	std::cout<<"Followed by simulation time  followed by traceFile name (in case of ns-2 trace) "<<std::endl;
	exit(0);
  }
  else {
 	numberOfNodes=atoi(argv[1]);
	terrainDimension=atoi(argv[2]);
	mobilityMode=atoi(argv[3]);
	simulationTime=atoi(argv[4]);
	if(mobilityMode==3) {
	    if(argc < 6) {
		std::cout<<"Usage: Enter number of Nodes followed by the size of terrain followed by mobility mode "<<std::endl;
		std::cout<<"1. Constant position, 2. Mobile based on random walk, 3. Trace driven"<<std::endl;
		std::cout<<"Followed by simulation time  followed by traceFile name (in case of ns-2 trace) "<<std::endl;
		exit(0);
 	     }
	   else {
		traceFile=argv[5];		// traceFile for ns-2 traces. 
	    } 
	}
  }

  // log output from ns-2 mobility trace course changes 
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
  nodes.Create (numberOfNodes);

  InitializeNetworkComponents();

  if (mobilityMode ==2) {

             std::cout<<"Terrain dimension is "<<terrainDimension<<std::endl;
             mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
              "MinX", DoubleValue (0.0),
              "MinY", DoubleValue (0.0),
              "DeltaX", DoubleValue (1),
              "DeltaY", DoubleValue (1),
              "GridWidth", UintegerValue (1),
              "LayoutType", StringValue ("RowFirst"));
             mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel","Bounds", RectangleValue (Rectangle (0, terrainDimension, 0, terrainDimension)));
	     mobility.Install(nodes);
	     mobility.EnableAsciiAll(std::cout);// God knows what is happening here? Now suddenly it all seems to work even without enabling the ASCII all. God, help me out here. 
             std::cout<<"Initiated mobility \n";

  }
  else  if (mobilityMode ==3) { // feed in ns-2 traces. 
             std::cout<<"Terrain dimension is "<<terrainDimension<<std::endl;
             Ns2MobilityHelper ns2mobility=Ns2MobilityHelper(traceFile);
	     ns2mobility.Install(); // TODO: configure call back if required. 
	     Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                   MakeBoundCallback (&CourseChange, &os));
             std::cout<<"Initiated mobility from ns-2 trace file \n";
  }

  int i=0;
  for(i=0;i<numberOfNodes;i++) {
//       std::cout<<"Now inserting node"<<i<<std::endl;
         Simulator::Schedule(Seconds((i+1)*0.5),&InitializeNetworkedMobileNode,i);
  }


  std::cout<<"Simulating for "<<simulationTime<<" seconds "<<std::endl;
  Simulator::Stop (Seconds (simulationTime));
  Simulator::Run();
  Simulator::Destroy ();
}
