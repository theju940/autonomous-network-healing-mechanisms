#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/wifi-radio-energy-model-helper.h"

#include <iostream>
#include <set>

using namespace ns3;

double energyThreshold = 5.0;
std::set<uint32_t> deadNodes;

void CheckEnergy(ns3::energy::EnergySourceContainer sources)
{
    uint32_t id = 0;

    for (auto i = sources.Begin(); i != sources.End(); ++i, ++id)
    {
        double energy = (*i)->GetRemainingEnergy();

        if (energy < energyThreshold && deadNodes.find(id) == deadNodes.end())
        {
            deadNodes.insert(id);

            Ptr<Node> node = NodeList::GetNode(id);

            std::cout << "[HEALING] Node " << id << " disabled (Energy=" << energy << "J)\n";

            for (uint32_t j = 0; j < node->GetNDevices(); j++)
            {
                node->GetDevice(j)->SetReceiveCallback(
                    MakeNullCallback<bool, Ptr<NetDevice>, Ptr<const Packet>, uint16_t, const Address &>());
            }
        }
    }

    Simulator::Schedule(Seconds(2.0), &CheckEnergy, sources);
}

void RunSim(uint32_t numNodes, uint32_t totalPackets)
{
    double simTime = 20.0;
    deadNodes.clear();

    NodeContainer nodes;
    nodes.Create(numNodes);

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);

    YansWifiPhyHelper phy;
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    phy.SetChannel(channel.Create());

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    MobilityHelper mobility;

    Ptr<RandomRectanglePositionAllocator> pos =
        CreateObject<RandomRectanglePositionAllocator>();

    pos->SetX(CreateObjectWithAttributes<UniformRandomVariable>(
        "Min", DoubleValue(0.0), "Max", DoubleValue(100.0)));

    pos->SetY(CreateObjectWithAttributes<UniformRandomVariable>(
        "Min", DoubleValue(0.0), "Max", DoubleValue(100.0)));

    mobility.SetPositionAllocator(pos);

    mobility.SetMobilityModel(
        "ns3::RandomWaypointMobilityModel",
        "Speed", StringValue("ns3::UniformRandomVariable[Min=15.0|Max=30.0]"),
        "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"),
        "PositionAllocator", PointerValue(pos));

    mobility.Install(nodes);

    DsdvHelper dsdv;
    InternetStackHelper internet;
    internet.SetRoutingHelper(dsdv);
    internet.Install(nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

    BasicEnergySourceHelper energy;
    energy.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(25));
    auto sources = energy.Install(nodes);

    WifiRadioEnergyModelHelper radio;
    radio.Install(devices, sources);

    Simulator::Schedule(Seconds(1.0), &CheckEnergy, sources);

    uint32_t flows = std::max(1U, numNodes / 2);
    uint32_t packetsPerFlow = totalPackets / flows;

    for (uint32_t i = 0; i < flows; i++)
    {
        uint32_t src = i;
        uint32_t dst = (i + 1) % numNodes;

        uint16_t port = 9000 + i;

        PacketSinkHelper sink("ns3::UdpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(), port));

        auto sinkApp = sink.Install(nodes.Get(dst));
        sinkApp.Start(Seconds(0.5));
        sinkApp.Stop(Seconds(20.0));

        UdpClientHelper client(interfaces.GetAddress(dst), port);
        client.SetAttribute("MaxPackets", UintegerValue(packetsPerFlow));
        client.SetAttribute("Interval", TimeValue(Seconds(0.02)));
        client.SetAttribute("PacketSize", UintegerValue(128));

        auto app = client.Install(nodes.Get(src));
        app.Start(Seconds(1.0));
        app.Stop(Seconds(20.0));
    }

    AnimationInterface anim("anim-10.xml");

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(20.0));
    Simulator::Run();

    monitor->CheckForLostPackets();
    auto stats = monitor->GetFlowStats();

    uint32_t tx = 0, rx = 0;
    double delay = 0, jitter = 0;

    for (auto &f : stats)
    {
        tx += f.second.txPackets;
        rx += f.second.rxPackets;
        delay += f.second.delaySum.GetSeconds();
        jitter += f.second.jitterSum.GetSeconds();
    }

    double pdr = (tx > 0) ? ((double)rx / tx * 100) : 0;
    double avgDelay = (rx > 0) ? (delay / rx) : 0;
    double avgJitter = (rx > 0) ? (jitter / rx) : 0;
    double throughput = (rx * 128 * 8) / (20.0 * 1000);

    std::cout << "\n----- Simulation Results -----\n";
    std::cout << "Nodes = 30\n";
    std::cout << "Packets Sent = " << tx << "\n";
    std::cout << "Packets Received = " << rx << "\n";
    std::cout << "PDR = " << pdr << " %\n";
    std::cout << "Delay = " << avgDelay << "\n";
    std::cout << "Jitter = " << avgJitter << "\n";
    std::cout << "Throughput = " << throughput << "\n";

    Simulator::Destroy();
}

int main()
{
    uint32_t packets;
    std::cin >> packets;
    RunSim(30, packets);
    return 0;
}