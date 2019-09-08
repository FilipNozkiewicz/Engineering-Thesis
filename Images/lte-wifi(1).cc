         /* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
         /*
          * Copyright (c) 2017
          *
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
          */

         #include "ns3/core-module.h"
         #include "ns3/applications-module.h"
         #include "ns3/wifi-module.h"
         #include "ns3/mobility-module.h"
         #include "ns3/internet-module.h"
         #include "ns3/propagation-loss-model.h"
         #include "ns3/propagation-module.h"
         #include "ns3/node-list.h"
         #include "ns3/ipv4-l3-protocol.h"
         #include "ns3/point-to-point-module.h"
         #include "ns3/network-module.h"
         #include "ns3/csma-module.h"
         #include "ns3/flow-monitor-module.h"
         #include "ns3/ipv4-address.h"
         #include "ns3/spectrum-module.h"
         #include "ns3/config-store.h"
         //#include "ns3/mac-low.h"
         #include "ns3/packet.h"

         #include <iostream>
         #include <vector>
         #include <math.h>
         #include <string>
         #include <fstream>
         #include <string> 
         #include <ctime>
         #include <iomanip>
         #include <sys/stat.h>

         using namespace std;
         using namespace ns3;

         NS_LOG_COMPONENT_DEFINE ("lte-wifi");

         uint32_t MacRxCountFast=0;
         uint32_t MacRxCountSlow=0;
         double simulationTime = 10; //seconds
         double warmupTime = 5; //seconds

         //=============== MAC TX drop
         void
         MacRx(Ptr<const Packet> packet)
         {
           if (Simulator::Now().GetSeconds()>warmupTime) {
	           NS_LOG_INFO("MAC RX Packet");
	           Ptr<Packet> p = packet->Copy();
	           Ipv4Header ip;
	           LlcSnapHeader llc;
	           p->RemoveHeader (llc);
	           p->PeekHeader (ip); 
	           if (ip.GetSource().CombineMask (Ipv4Mask("255.255.255.0")) == Ipv4Address ("10.1.0.0")) {
		         //std::cout << Simulator::Now().GetSeconds() << "\t" << ip.GetSource()  << " fast\n";
		         MacRxCountFast++;
	           }
	           else if (ip.GetSource().CombineMask (Ipv4Mask("255.255.255.0")) == Ipv4Address ("10.1.1.0")){
		         //std::cout << Simulator::Now().GetSeconds() << "\t" << ip.GetSource()  << " slow\n";
		         MacRxCountSlow++;
	           }
           }  
         }


         // ============= Print Stats
         void
         PrintStats()
         {
           double realTime = simulationTime - warmupTime;
           std::cout << Simulator::Now().GetSeconds() << "\t" << MacRxCountFast << "\t" << MacRxCountSlow << "\n";
           std::cout << Simulator::Now().GetSeconds() << "\t" << MacRxCountFast*1500.0*8/realTime/1000000 << " Mb/s\t" << MacRxCountSlow*1500.0*8/realTime/1000000 << " Mb/s\n";
         }

         Ptr<SpectrumModel> SpectrumModelWifi5180MHz;

         class static_SpectrumModelWifi5180MHz_initializer
         {
         public:
           static_SpectrumModelWifi5180MHz_initializer ()
           {
             BandInfo bandInfo;
             bandInfo.fc = 5180e6;
             bandInfo.fl = 5180e6 - 10e6;
             bandInfo.fh = 5180e6 + 10e6;

             Bands bands;
             bands.push_back (bandInfo);

             SpectrumModelWifi5180MHz = Create<SpectrumModel> (bands);
           }

         } static_SpectrumModelWifi5180MHz_initializer_instance;

         /*******  Forward declaration of functions *******/

         void placeNodes(double **xy,NodeContainer &Nodes); // Place each node in 2D plane (X,Y)
         double **calculateSTApositions(double x_ap, double y_ap, int h, int n_stations); //calculate positions of the stations
         void installTrafficGenerator(Ptr<ns3::Node> fromNode, Ptr<ns3::Node> toNode, int port, string offeredLoad, int packetSize);
         void showPosition(NodeContainer &Nodes); // Show node positions
         void PopulateARPcache ();
         bool fileExists(const std::string& filename);

         /*******  End of all forward declaration of functions *******/



         int main (int argc, char *argv[])
         {

	         /* Variable declarations */

	         bool enableRtsCts = false; // RTS/CTS disabled by default
	         int nFast = 2; Fast stations
	         int nSlow = 2; //Slow stations
	         bool debug = false;
	         bool pcap = false;
	         bool xml = false;
	         string offeredLoad; //Mbps
	         string mcs;
	         int radius = 10; //radius of station placement [m]
	         bool uniformLoss = false;
	         double waveformPower = 1.0;
	         double ltePeriod = 0.01;
	         double lteDutyCycle = 0.5;
	         string outputCsv = "lte-wifi.csv";
                  int maxSlrc = 7;
	         int packetSize = 1500;

                  bool enableRtsCts;  //parameters which enable 
                  //or desable RTS/CTS frames
	         int nFast; //number Fast stations
	         int nSlow; //number of slow stations Slow stations
	         bool debug; // activate or deactivate debug mode
	         bool pcap; // enable or disable gathering info 
                  // about data flow o the files
	         bool xml; // activate xml mode
	         string offeredLoad; //data offered load 
	         string mcs;// used modulation coding scheme
	         int radius; //radius of station placement [m]
	         bool uniformLoss = false;
	         double waveformPower; //trasnmitting power used by interferer
	         double ltePeriod; // period of lte transmission [s]
	         double lteDutyCycle; // LTE duty cycle [%]
	         string outputCsv; // results output file
                  int maxSlrc = 7;
	         int packetSize = 1500; // ip packet size [B]
                  unsigned int RngRun; // add randomness to the simulation



	         /* Command line parameters */

	         CommandLine cmd;
	         cmd.AddValue ("simulationTime", "Simulation time [s]", simulationTime);
	         cmd.AddValue ("nFast", "Number of fast (54 Mbps) stations", nFast);
	         cmd.AddValue ("nSlow", "Number of slow (6 Mbps) stations", nSlow);	
	         cmd.AddValue ("debug", "Enable debug mode", debug);
	         cmd.AddValue ("rts", "Enable RTS/CTS", enableRtsCts);
	         cmd.AddValue ("pcap", "Enable PCAP generation", pcap);
	         cmd.AddValue ("xml", "Enable XML generation", xml);	
	         cmd.AddValue ("uniformLoss", "Set uniform loss on all links", uniformLoss);
	         cmd.AddValue ("waveformPower", "Waveform power", waveformPower);
	         cmd.AddValue ("ltePeriod", "Period of LTE interference [s]", ltePeriod);
	         cmd.AddValue ("lteDutyCycle", "Duty cycle of LTE interference [0 to 1]", lteDutyCycle);
	         cmd.AddValue ("outputCsv", "CSV file to store results", outputCsv);
	         cmd.AddValue ("maxSlrc", "Configure maximum number of retries", maxSlrc);
	         cmd.Parse (argc,argv);
	
           ConfigStore config;
           config.ConfigureDefaults ();	

	         offeredLoad = std::to_string(27.0/(nFast+nSlow)); //Scale offered load to number of stations (saturation conditions)

	         if(uniformLoss){
		         radius=0;
	         }

	         if(debug) {
		         offeredLoad = "1";
	         }
	
	         if(debug) {
		         LogComponentEnable ("UdpClient", LOG_LEVEL_INFO);
		         LogComponentEnable ("UdpServer", LOG_LEVEL_INFO);
	         }

	         /* Enable or disable RTS/CTS */

	         if (enableRtsCts) {
		         Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("100"));
	         }
	         else {
		         Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("1100000"));
	         }


	         /* Place AP in 3D (X,Y,Z) plane */

	         NodeContainer wifiApNodes;
	         wifiApNodes.Create(2);

	         double **APpositions=0;
	         int numAPs = 2;
	         APpositions = new double*[2];
	         APpositions[0]=new double[numAPs];
	         APpositions[1]=new double[numAPs];

	         for (int i=0;i<numAPs;i++){
		         APpositions[0][i]=0.0;
		         APpositions[1][i]=0.0;
	         }
	         placeNodes(APpositions,wifiApNodes);



	         /* Place each station randomly around AP */

	         NodeContainer wifiStaNodesFast, wifiStaNodesSlow;
	         wifiStaNodesFast.Create(nFast);
	         wifiStaNodesSlow.Create(nSlow);	

	         double **STApositions;
	         STApositions = calculateSTApositions(0.0, 0.0, radius, nFast);
	         placeNodes(STApositions,wifiStaNodesFast);
	         STApositions = calculateSTApositions(0.0, 0.0, radius, nSlow);
	         placeNodes(STApositions,wifiStaNodesSlow);	

	         /* Display node positions */

	         if(debug)
	         {
		         cout << "Position of APs:"<< endl;
		         showPosition(wifiApNodes);
		         cout <<"Fast station positions around AP:"<<endl;
		         showPosition(wifiStaNodesFast);
		         cout <<"Slow station positions around AP:"<<endl;
		         showPosition(wifiStaNodesSlow);		
	         }

	         /* Configure propagation model */
                  //YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
                  //wifiHelper.SetRemoteStationManager ("ns3::IdealWifiManager");



	         WifiMacHelper wifiMac;
	         WifiHelper wifiHelper;
	         
	         SpectrumWifiPhyHelper spectrumPhy = SpectrumWifiPhyHelper::Default ();
	         Ptr<MultiModelSpectrumChannel> spectrumChannel;
	
	         wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211a);
	         
	         wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
			         "DataMode", StringValue ("OfdmRate54Mbps"),
			         "NonUnicastMode", StringValue ("OfdmRate54Mbps"),
			         "ControlMode", StringValue ("OfdmRate54Mbps"),
			         "MaxSsrc", UintegerValue (7),
			         "MaxSlrc", UintegerValue (maxSlrc),
			         "FragmentationThreshold", UintegerValue (2500));
	

	        
	         Config::SetDefault ("ns3::WifiPhy::CcaMode1Threshold",
                  DoubleValue (-62.0));	

	         if(uniformLoss){
		         spectrumChannel = CreateObject<MultiModelSpectrumChannel> ();
		         Ptr<MatrixPropagationLossModel> lossModel = 
                           CreateObject<MatrixPropagationLossModel> ();
		         lossModel->SetDefaultLoss (50);
		         spectrumChannel->AddPropagationLossModel (lossModel);
	         }
	         else {
		         spectrumChannel = CreateObject<MultiModelSpectrumChannel> ();
		         Ptr<FriisPropagationLossModel> lossModel = 
                           CreateObject<FriisPropagationLossModel> ();
		         lossModel->SetFrequency (5.180e9);
		         spectrumChannel->AddPropagationLossModel (lossModel);
	         }
	
	         Ptr<ConstantSpeedPropagationDelayModel> delayModel = 
                  CreateObject<ConstantSpeedPropagationDelayModel> ();
	         spectrumChannel->SetPropagationDelayModel (delayModel);

	         spectrumPhy.SetChannel (spectrumChannel);
	         spectrumPhy.Set ("Frequency", UintegerValue (5180));	



	         //Ssid ssid;

	         //ssid = Ssid ("wifi-fast");
	         //wifiMac.SetType ("ns3::ApWifiMac","Ssid", SsidValue (ssid));
	         wifiMac.SetType ("ns3::AdhocWifiMac");
	
	         NetDeviceContainer apDeviceFast = wifiHelper.Install (spectrumPhy, wifiMac, wifiApNodes.Get(0));

	         //wifiMac.SetType ("ns3::StaWifiMac",	"Ssid", SsidValue (ssid),"ActiveProbing", BooleanValue (false));
	         NetDeviceContainer staDeviceFast = wifiHelper.Install (spectrumPhy, wifiMac, wifiStaNodesFast);
	
	         Config::Set("/NodeList/1/DeviceList/0/$ns3::WifiNetDevice/RemoteStationManager/$ns3::ConstantRateWifiManager/DataMode", StringValue ("OfdmRate6Mbps"));
	         Config::Set("/NodeList/1/DeviceList/0/$ns3::WifiNetDevice/RemoteStationManager/$ns3::ConstantRateWifiManager/ControlMode", StringValue ("OfdmRate6Mbps"));
	         Config::Set("/NodeList/1/DeviceList/0/$ns3::WifiNetDevice/RemoteStationManager/$ns3::ConstantRateWifiManager/NonUnicastMode", StringValue ("OfdmRate6Mbps"));	
	
	         wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
			         "DataMode", StringValue ("OfdmRate6Mbps"),
			         "NonUnicastMode", StringValue ("OfdmRate6Mbps"),
			         "ControlMode", StringValue ("OfdmRate6Mbps"));	
	

	         //ssid = Ssid ("wifi-slow");
	         //wifiMac.SetType ("ns3::ApWifiMac","Ssid", SsidValue (ssid));
	         NetDeviceContainer apDeviceSlow = wifiHelper.Install (spectrumPhy, wifiMac, wifiApNodes.Get(1));

	         //wifiMac.SetType ("ns3::StaWifiMac",	"Ssid", SsidValue (ssid),"ActiveProbing", BooleanValue (false));
	         NetDeviceContainer staDeviceSlow = wifiHelper.Install (spectrumPhy, wifiMac, wifiStaNodesSlow);
	
	         //Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::AdhocWifiMac/AckTimeout", StringValue ("+18000.0ns"));	
	         //Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/AckTimeout", TimeValue (MicroSeconds (30)));

	         /* Configure Internet stack */

	         InternetStackHelper stack;
	         stack.Install (wifiApNodes);
	         stack.Install (wifiStaNodesFast);
	         stack.Install (wifiStaNodesSlow);

	         Ipv4AddressHelper address;
	         address.SetBase ("10.1.0.0", "255.255.255.0");

	         address.Assign (apDeviceFast);
	         address.Assign (staDeviceFast);

	         address.SetBase ("10.1.1.0", "255.255.255.0");

	         address.Assign (apDeviceSlow);
	         address.Assign (staDeviceSlow);

	         PopulateARPcache ();
	
	         /* Create interfering (LTE) node */
	
	         NodeContainer interferingNode;
             interferingNode.Create(1);
	         STApositions = calculateSTApositions(0.0, 0.0, radius, 1);
	         placeNodes(STApositions,interferingNode);	

	         /* Configure applications */

	         if(debug) {
		         std::cout << "Offered load: " << offeredLoad  << std::endl;
	         }

	         int port=9;
	         for(int i = 0; i < nFast; ++i) {
		         installTrafficGenerator(wifiStaNodesFast.Get(i),wifiApNodes.Get(0), port++, offeredLoad, packetSize);
	         }
	         for(int i = nSlow-1; i >= 0; --i) {
		         installTrafficGenerator(wifiStaNodesSlow.Get(i),wifiApNodes.Get(1), port++, offeredLoad, packetSize);
	         }

	
	        
	         Ptr<SpectrumValue> wgPsd = Create<SpectrumValue>
                  (SpectrumModelWifi5180MHz);
	         *wgPsd = waveformPower / (100 * 180000);

	         NS_LOG_INFO ("wgPsd : " << *wgPsd << " integrated power: " 
                  << Integral (*(GetPointer (wgPsd))));

	         WaveformGeneratorHelper waveformGeneratorHelper;
	         waveformGeneratorHelper.SetChannel (spectrumChannel);
	         waveformGeneratorHelper.SetTxPowerSpectralDensity (wgPsd);

	         waveformGeneratorHelper.SetPhyAttribute ("Period", 
                  TimeValue (Seconds (ltePeriod)));

	         waveformGeneratorHelper.SetPhyAttribute ("DutyCycle", 
                  DoubleValue (lteDutyCycle));

	         NetDeviceContainer waveformGeneratorDevices =
                   waveformGeneratorHelper.Install (interferingNode);

	         Simulator::Schedule (Seconds (1), &WaveformGenerator::Start,
                  waveformGeneratorDevices.Get (0)->GetObject<NonCommunicatingNetDevice> ()
                  ->GetPhy ()->GetObject<WaveformGenerator> ());
	
	         


                  /* Configure tracing */


	         if(pcap) {
		         spectrumPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
		         spectrumPhy.EnablePcap ("lte-wifi-fast", apDeviceFast);
		         //spectrumPhy.EnablePcap ("lte-wifi-fast", staDeviceFast);
		         spectrumPhy.EnablePcap ("lte-wifi-slow", apDeviceSlow);
		         //spectrumPhy.EnablePcap ("lte-wifi-slow", staDeviceSlow);
	         }

	         FlowMonitorHelper flowmon;
	         Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
	         monitor->SetAttribute ("StartTime", TimeValue (Seconds (5)));
	
	
	         Config::ConnectWithoutContext("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx", MakeCallback(&MacRx));
	
	         config.ConfigureAttributes ();
	
	         /* Run simulation */

	         Simulator::Stop(Seconds(simulationTime));
	         Simulator::Run ();
	
	         PrintStats();

	         /* Calculate results */
	         double flowThr;
	         double flowDel;


	         /* Contents of CSV output file-style
	            Timestamp, OfferedLoad, nFast, nSlow, RngRun, SourceIP, DestinationIP, Throughput, Delay"
	          */
	          
	         if(xml){
		         monitor->SerializeToXmlFile("lte-wifi.xml", true, true);
	         }
	
	         ofstream myfile;
	         if (fileExists(outputCsv))
	         {
		         myfile.open (outputCsv, ios::app);
	         }
	         else {
		         myfile.open (outputCsv, ios::app);  
		         myfile << "Timestamp,OfferedLoad,nFast,nSlow,RngRun,SourceIP,DestinationIP,Throughput,Delay" << std::endl;
	         }
           
	         //Get timestamp
	         auto t = std::time(nullptr);
	         auto tm = *std::localtime(&t);

	         Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
	         std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
	         for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i) {
		         Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
		         flowThr=i->second.rxPackets * packetSize * 8.0 / (i->second.timeLastRxPacket.GetSeconds () - i->second.timeFirstTxPacket.GetSeconds ()) / 1000 / 1000;
		         flowDel=i->second.delaySum.GetSeconds () / i->second.rxPackets;
		         NS_LOG_UNCOND ("Flow " << i->first  << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\tThroughput: " <<  flowThr  << " Mbps\tTime: " << i->second.timeLastRxPacket.GetSeconds () - i->second.timeFirstTxPacket.GetSeconds () << "\tDelay: " << flowDel << " s\tTx packets " << i->second.txPackets << " s\tRx packets " << i->second.rxPackets << "\n");
		         myfile << std::put_time(&tm, "%Y-%m-%d %H:%M") << "," << offeredLoad << "," << nFast << "," << nSlow << "," << RngSeedManager::GetRun() << "," << t.sourceAddress << "," << t.destinationAddress << "," << flowThr << "," << flowDel;
		         myfile << std::endl;
	         }
	         myfile.close();
	
	
	         /* End of simulation */
	         Simulator::Destroy ();
	         return 0;
         }

         /***** Functions definition *****/

         void placeNodes(double **xy,NodeContainer &Nodes) {
	         uint32_t nNodes = Nodes.GetN ();
	         double height = 0;
	         MobilityHelper mobility;
	         Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

	         for(uint32_t i = 0; i < nNodes; ++i)
	         {
		         positionAlloc->Add (Vector (xy[0][i],xy[1][i],height));
	         }

	         mobility.SetPositionAllocator (positionAlloc);
	         mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	         mobility.Install (Nodes);
         }

         void showPosition(NodeContainer &Nodes) {

	         uint32_t NodeNumber = 0;

	         for(NodeContainer::Iterator nAP = Nodes.Begin (); nAP != Nodes.End (); ++nAP)
	         {
		         Ptr<Node> object = *nAP;
		         Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
		         NS_ASSERT (position != 0);
		         Vector pos = position->GetPosition ();
		         std::cout <<"Node Number"<<"("<< NodeNumber <<")"<<" has coordinates "<< "(" << pos.x << ", "<< pos.y <<", "<< pos.z <<")" << std::endl;
		         ++NodeNumber;
	         }
         }

         double **calculateSTApositions(double x_ap, double y_ap, int h, int n_stations) {

	         double PI  =3.141592653589793238463;


	         double tab[2][n_stations];
	         double** sta_co=0;
	         sta_co = new double*[2];
	         sta_co[0]=new double[n_stations];
	         sta_co[1]=new double[n_stations];
	         double ANG = 2*PI;

	         float X=1;
	         for(int i=0; i<n_stations; i++){
		         float sta_x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));
		         tab[0][i]= sta_x*h;

	         }

	         for (int j=0; j<n_stations; j++){
		         float angle = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/ANG));
		         tab[1][j]=angle;

	         }
	         for ( int k=0; k<n_stations; k++){
		         sta_co[0][k]=x_ap+cos(tab[1][k])*tab[0][k];
		         sta_co[1][k]=y_ap+sin(tab[1][k])*tab[0][k];

	         }


	         return sta_co;
         }

         void PopulateARPcache () {
	         Ptr<ArpCache> arp = CreateObject<ArpCache> ();
	         arp->SetAliveTimeout (Seconds (3600 * 24 * 365) );

	         for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i)
	         {
		         Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
		         NS_ASSERT (ip !=0);
		         ObjectVectorValue interfaces;
		         ip->GetAttribute ("InterfaceList", interfaces);

		         for (ObjectVectorValue::Iterator j = interfaces.Begin (); j != interfaces.End (); j++)
		         {
			         Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface> ();
			         NS_ASSERT (ipIface != 0);
			         Ptr<NetDevice> device = ipIface->GetDevice ();
			         NS_ASSERT (device != 0);
			         Mac48Address addr = Mac48Address::ConvertFrom (device->GetAddress () );

			         for (uint32_t k = 0; k < ipIface->GetNAddresses (); k++)
			         {
				         Ipv4Address ipAddr = ipIface->GetAddress (k).GetLocal();
				         if (ipAddr == Ipv4Address::GetLoopback ())
					         continue;

				         ArpCache::Entry *entry = arp->Add (ipAddr);
				         Ipv4Header ipv4Hdr;
				         ipv4Hdr.SetDestination (ipAddr);
				         Ptr<Packet> p = Create<Packet> (100);
				         entry->MarkWaitReply (ArpCache::Ipv4PayloadHeaderPair (p, ipv4Hdr));
				         entry->MarkAlive (addr);
			         }
		         }
	         }

	         for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i)
	         {
		         Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
		         NS_ASSERT (ip !=0);
		         ObjectVectorValue interfaces;
		         ip->GetAttribute ("InterfaceList", interfaces);

		         for (ObjectVectorValue::Iterator j = interfaces.Begin (); j != interfaces.End (); j ++)
		         {
			         Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface> ();
			         ipIface->SetAttribute ("ArpCache", PointerValue (arp) );
		         }
	         }
         }

         void installTrafficGenerator(Ptr<ns3::Node> fromNode, Ptr<ns3::Node> toNode, int port, string offeredLoad, int packetSize) {

	         Ptr<Ipv4> ipv4 = toNode->GetObject<Ipv4> (); // Get Ipv4 instance of the node
	         Ipv4Address addr = ipv4->GetAddress (1, 0).GetLocal (); // Get Ipv4InterfaceAddress of xth interface.
	         //addr.Set("255.255.255.255");
	
	         ApplicationContainer sourceApplications, sinkApplications;

	         uint8_t tosValue = 0x70; //AC_BE
	
	         //Add random fuzz to app start time
	         double min = 0.0;
	         double max = 1.0;
	         Ptr<UniformRandomVariable> fuzz = CreateObject<UniformRandomVariable> ();
	         fuzz->SetAttribute ("Min", DoubleValue (min));
	         fuzz->SetAttribute ("Max", DoubleValue (max));	

	         InetSocketAddress sinkSocket (addr, port);
	         sinkSocket.SetTos (tosValue);
	         //OnOffHelper onOffHelper ("ns3::TcpSocketFactory", sinkSocket);
	         OnOffHelper onOffHelper ("ns3::UdpSocketFactory", sinkSocket);
	         onOffHelper.SetConstantRate (DataRate (offeredLoad + "Mbps"), packetSize-20-8-8);
	         sourceApplications.Add (onOffHelper.Install (fromNode)); //fromNode
	
	
	         //PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", sinkSocket);
	         PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", sinkSocket);
	         sinkApplications.Add (packetSinkHelper.Install (toNode)); //toNode

	         sinkApplications.Start (Seconds (0.0));
	         sinkApplications.Stop (Seconds (simulationTime + 1));
	         sourceApplications.Start (Seconds (1.0+(roundf(fuzz->GetValue ()*1000)/1000))); //So that start is in ms
	         sourceApplications.Stop (Seconds (simulationTime + 1));

         }

         // Function: fileExists
         /**
             Check if a file exists
         @param[in] filename - the name of the file to check

         @return    true if the file exists, else false

         */
         bool fileExists(const std::string& filename)
         {
             struct stat buf;
             if (stat(filename.c_str(), &buf) != -1)
             {
                 return true;
             }
             return false;
         }

         /***** End of functions definition *****/






         ./waf --run "scratch/lte-wifi --RngRun=1 --offeredLoad=40 --nFast=3"





