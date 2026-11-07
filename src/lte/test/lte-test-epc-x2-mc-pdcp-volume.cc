/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/**
 * \file lte-test-epc-x2-mc-pdcp-volume.cc
 * \ingroup lte-test
 *
 * \brief Tests the EpcX2 traces that count multi-connectivity PDCP PDUs at a cell's X2-U
 * boundary: McDlPdcpPduRx (every GTP-U McForwardDownlinkData PDU a node receives) and
 * McUlPdcpPduTx (every GTP-U McForwardUplinkData PDU a node sends). An en-gNB counts
 * DRB.PdcpSduVolumeX2DL/UL (3GPP TS 28.552 clauses 5.1.2.1.1.2 and 5.1.2.1.2.2) from them.
 *
 * Three EpcX2 entities, A (cell 1, the master eNB) and B, C (cells 2 and 3, en-gNBs), are joined
 * pairwise by real X2 links (point-to-point, UDP). Every receiving TEID has a user stub. Each
 * case drives one X2 path and checks every fire of both traces, with its node, local cell,
 * remote cell, TEID and PDU size (the PDCP PDU, the GTP-U header excluded), and where the PDU is
 * delivered:
 *  (a) A sends a PDCP PDU to B: one DL event at B (remote A), delivered to B's RLC user;
 *  (b) B forwards the TEID to C (secondary cell handover): one DL event at B (remote A) and one
 *      at C (remote B), same size, B's RLC user not called, C's called;
 *  (c) B forwards an RLC buffer PDU to C: one DL event at C (remote B) only;
 *  (d) B's RLC delivers an UL PDCP PDU to A: one UL event at B (remote A), no DL event,
 *      delivered to A's PDCP user;
 *  (e) A sends untyped UE data to B: no event, delivered to B's X2 SAP user.
 */

#include "ns3/epc-x2-sap.h"
#include "ns3/epc-x2.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/packet.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/test.h"

#include <array>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

/// The cells of nodes A, B and C, by node index
const std::array<uint16_t, 3> CELL = {1, 2, 3};
const uint32_t NODE_A = 0;
const uint32_t NODE_B = 1;
const uint32_t NODE_C = 2;
/// The GTP-U TEID of the bearer
const uint32_t TEID = 7;
/// A PDCP Data PDU: a 1308-byte SDU and the 2-byte NR PDCP header
const uint32_t PDU_BYTES = 1310;

/** One fire of McDlPdcpPduRx or McUlPdcpPduTx. */
struct X2Event
{
    uint32_t node;
    std::string trace;
    uint16_t localCellId;
    uint16_t remoteCellId;
    uint32_t teid;
    uint32_t pduBytes;
};

std::string
Format(const std::vector<X2Event>& events)
{
    std::ostringstream out;
    for (const auto& e : events)
    {
        out << "[node " << e.node << " " << e.trace << " local " << e.localCellId << " remote "
            << e.remoteCellId << " teid " << e.teid << " bytes " << e.pduBytes << "]";
    }
    return out.str();
}

std::string
Format(const std::vector<uint32_t>& sizes)
{
    std::ostringstream out;
    for (auto s : sizes)
    {
        out << "[" << s << "]";
    }
    return out.str();
}

void
RecordX2Event(std::vector<X2Event>* events,
              uint32_t node,
              std::string trace,
              uint16_t localCellId,
              uint16_t remoteCellId,
              uint32_t teid,
              uint32_t pduBytes)
{
    events->push_back({node, trace, localCellId, remoteCellId, teid, pduBytes});
}

/** The RLC user of a TEID at an en-gNB: records the size of every PDU X2 hands it. */
class RlcUserStub : public EpcX2RlcUser
{
  public:
    void SendMcPdcpSdu(UeDataParams params) override
    {
        sizes.push_back(params.ueData->GetSize());
    }

    std::vector<uint32_t> sizes;
};

/** The PDCP user of a TEID at the master eNB: records the size of every PDU X2 hands it. */
class PdcpUserStub : public EpcX2PdcpUser
{
  public:
    void ReceiveMcPdcpPdu(UeDataParams params) override
    {
        sizes.push_back(params.ueData->GetSize());
    }

    std::vector<uint32_t> sizes;
};

/** The X2 SAP user of a node: records untyped UE data; no other primitive is expected. */
class X2SapUserStub : public EpcX2SapUser
{
  public:
    void RecvUeData(UeDataParams params) override
    {
        ueDataSizes.push_back(params.ueData->GetSize());
    }

    void RecvHandoverRequest(HandoverRequestParams) override
    {
        Unexpected("RecvHandoverRequest");
    }

    void RecvHandoverRequestAck(HandoverRequestAckParams) override
    {
        Unexpected("RecvHandoverRequestAck");
    }

    void RecvHandoverPreparationFailure(HandoverPreparationFailureParams) override
    {
        Unexpected("RecvHandoverPreparationFailure");
    }

    void RecvSnStatusTransfer(SnStatusTransferParams) override
    {
        Unexpected("RecvSnStatusTransfer");
    }

    void RecvUeContextRelease(UeContextReleaseParams) override
    {
        Unexpected("RecvUeContextRelease");
    }

    void RecvLoadInformation(LoadInformationParams) override
    {
        Unexpected("RecvLoadInformation");
    }

    void RecvResourceStatusUpdate(ResourceStatusUpdateParams) override
    {
        Unexpected("RecvResourceStatusUpdate");
    }

    void RecvRlcSetupRequest(RlcSetupRequest) override
    {
        Unexpected("RecvRlcSetupRequest");
    }

    void RecvRlcSetupCompleted(UeDataParams) override
    {
        Unexpected("RecvRlcSetupCompleted");
    }

    void RecvUeSinrUpdate(UeImsiSinrParams) override
    {
        Unexpected("RecvUeSinrUpdate");
    }

    void RecvMcHandoverRequest(SecondaryHandoverParams) override
    {
        Unexpected("RecvMcHandoverRequest");
    }

    void RecvLteMmWaveHandoverCompleted(SecondaryHandoverParams) override
    {
        Unexpected("RecvLteMmWaveHandoverCompleted");
    }

    void RecvConnectionSwitchToMmWave(SwitchConnectionParams) override
    {
        Unexpected("RecvConnectionSwitchToMmWave");
    }

    void RecvSecondaryCellHandoverCompleted(SecondaryHandoverCompletedParams) override
    {
        Unexpected("RecvSecondaryCellHandoverCompleted");
    }

    std::vector<uint32_t> ueDataSizes;
    std::vector<std::string> unexpected;

  private:
    void Unexpected(const std::string& primitive)
    {
        unexpected.push_back(primitive);
    }
};

/**
 * \ingroup lte-test
 * \brief One X2 path between the EpcX2 entities of nodes A, B and C, and the trace fires and
 *        deliveries it must produce.
 */
class EpcX2McPdcpVolumeTestCase : public TestCase
{
  public:
    /// The X2 path a case drives
    enum Path
    {
        MN_TO_SN,                ///< (a) A sends a PDCP PDU to B
        MN_TO_SN_REDIRECTED,     ///< (b) the same, while B forwards the TEID to C
        SN_RLC_BUFFER_TO_SN,     ///< (c) B forwards an RLC buffer PDU to C
        SN_UPLINK_TO_MN,         ///< (d) B's RLC delivers an UL PDCP PDU to A
        MN_UNTYPED_UE_DATA_TO_SN ///< (e) A sends untyped UE data to B
    };

    EpcX2McPdcpVolumeTestCase(const std::string& name, Path path)
        : TestCase(name),
          m_path(path)
    {
    }

  private:
    void DoRun() override
    {
        NodeContainer nodes;
        nodes.Create(3);
        InternetStackHelper internet;
        internet.Install(nodes);
        std::array<Ptr<EpcX2>, 3> x2;
        for (uint32_t i = 0; i < 3; i++)
        {
            x2[i] = CreateObject<EpcX2>();
            nodes.Get(i)->AggregateObject(x2[i]);
            x2[i]->SetEpcX2SapUser(&m_sapUser[i]);
        }

        // Real X2 links A-B, A-C and B-C, each with its own /30 network
        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("10Gb/s"));
        p2p.SetChannelAttribute("Delay", StringValue("1ms"));
        Ipv4AddressHelper addresses("10.1.0.0", "255.255.255.252");
        for (const auto& [i, j] : {std::pair<uint32_t, uint32_t>{NODE_A, NODE_B},
                                   std::pair<uint32_t, uint32_t>{NODE_A, NODE_C},
                                   std::pair<uint32_t, uint32_t>{NODE_B, NODE_C}})
        {
            NetDeviceContainer link = p2p.Install(nodes.Get(i), nodes.Get(j));
            Ipv4InterfaceContainer ifaces = addresses.Assign(link);
            addresses.NewNetwork();
            x2[i]->AddX2Interface(CELL[i], ifaces.GetAddress(0), CELL[j], ifaces.GetAddress(1));
            x2[j]->AddX2Interface(CELL[j], ifaces.GetAddress(1), CELL[i], ifaces.GetAddress(0));
        }

        // A user stub for every receiving TEID: the RLC users of the en-gNBs, the PDCP user of
        // the master eNB
        x2[NODE_B]->SetMcEpcX2RlcUser(TEID, &m_rlcUserB);
        x2[NODE_C]->SetMcEpcX2RlcUser(TEID, &m_rlcUserC);
        x2[NODE_A]->SetMcEpcX2PdcpUser(TEID, &m_pdcpUserA);

        for (uint32_t i = 0; i < 3; i++)
        {
            NS_TEST_ASSERT_MSG_EQ(
                x2[i]->TraceConnectWithoutContext(
                    "McDlPdcpPduRx",
                    MakeBoundCallback(&RecordX2Event, &m_events, i, std::string("dl"))),
                true,
                "connect McDlPdcpPduRx of node " << i);
            NS_TEST_ASSERT_MSG_EQ(
                x2[i]->TraceConnectWithoutContext(
                    "McUlPdcpPduTx",
                    MakeBoundCallback(&RecordX2Event, &m_events, i, std::string("ul"))),
                true,
                "connect McUlPdcpPduTx of node " << i);
        }

        m_x2 = x2;
        Simulator::Schedule(MilliSeconds(10), &EpcX2McPdcpVolumeTestCase::Drive, this);
        Simulator::Stop(Seconds(1));
        Simulator::Run();
        Check();
        m_x2 = {};
        Simulator::Destroy();
    }

    /// Hands one PDU to the X2 primitive of the case's path
    void Drive()
    {
        EpcX2Sap::UeDataParams params;
        params.gtpTeid = TEID;
        params.ueData = Create<Packet>(PDU_BYTES);
        switch (m_path)
        {
        case MN_TO_SN_REDIRECTED:
            m_x2[NODE_B]->GetEpcX2SapProvider()->AddTeidToBeForwarded(TEID, CELL[NODE_C]);
            [[fallthrough]];
        case MN_TO_SN:
            params.sourceCellId = CELL[NODE_A];
            params.targetCellId = CELL[NODE_B];
            m_x2[NODE_A]->GetEpcX2PdcpProvider()->SendMcPdcpPdu(params);
            break;
        case SN_RLC_BUFFER_TO_SN:
            params.sourceCellId = CELL[NODE_B];
            params.targetCellId = CELL[NODE_C];
            m_x2[NODE_B]->GetEpcX2SapProvider()->ForwardRlcPdu(params);
            break;
        case SN_UPLINK_TO_MN:
            params.sourceCellId = CELL[NODE_B];
            params.targetCellId = CELL[NODE_A];
            m_x2[NODE_B]->GetEpcX2RlcProvider()->ReceiveMcPdcpSdu(params);
            break;
        case MN_UNTYPED_UE_DATA_TO_SN:
            params.sourceCellId = CELL[NODE_A];
            params.targetCellId = CELL[NODE_B];
            m_x2[NODE_A]->GetEpcX2SapProvider()->SendUeData(params);
            break;
        }
    }

    void Check()
    {
        std::vector<X2Event> events;
        std::vector<uint32_t> rlcB;
        std::vector<uint32_t> rlcC;
        std::vector<uint32_t> pdcpA;
        std::vector<uint32_t> ueDataB;
        switch (m_path)
        {
        case MN_TO_SN:
            events = {{NODE_B, "dl", CELL[NODE_B], CELL[NODE_A], TEID, PDU_BYTES}};
            rlcB = {PDU_BYTES};
            break;
        case MN_TO_SN_REDIRECTED:
            events = {{NODE_B, "dl", CELL[NODE_B], CELL[NODE_A], TEID, PDU_BYTES},
                      {NODE_C, "dl", CELL[NODE_C], CELL[NODE_B], TEID, PDU_BYTES}};
            rlcC = {PDU_BYTES};
            break;
        case SN_RLC_BUFFER_TO_SN:
            events = {{NODE_C, "dl", CELL[NODE_C], CELL[NODE_B], TEID, PDU_BYTES}};
            rlcC = {PDU_BYTES};
            break;
        case SN_UPLINK_TO_MN:
            events = {{NODE_B, "ul", CELL[NODE_B], CELL[NODE_A], TEID, PDU_BYTES}};
            pdcpA = {PDU_BYTES};
            break;
        case MN_UNTYPED_UE_DATA_TO_SN:
            ueDataB = {PDU_BYTES};
            break;
        }
        NS_TEST_EXPECT_MSG_EQ(Format(m_events),
                              Format(events),
                              "every McDlPdcpPduRx and McUlPdcpPduTx fire, in order");
        NS_TEST_EXPECT_MSG_EQ(Format(m_rlcUserB.sizes), Format(rlcB), "PDUs delivered to B's RLC");
        NS_TEST_EXPECT_MSG_EQ(Format(m_rlcUserC.sizes), Format(rlcC), "PDUs delivered to C's RLC");
        NS_TEST_EXPECT_MSG_EQ(Format(m_pdcpUserA.sizes),
                              Format(pdcpA),
                              "PDUs delivered to A's PDCP");
        NS_TEST_EXPECT_MSG_EQ(Format(m_sapUser[NODE_B].ueDataSizes),
                              Format(ueDataB),
                              "UE data delivered to B's X2 SAP user");
        for (uint32_t i = 0; i < 3; i++)
        {
            if (i != NODE_B)
            {
                NS_TEST_EXPECT_MSG_EQ(m_sapUser[i].ueDataSizes.size(),
                                      0,
                                      "UE data delivered to the X2 SAP user of node " << i);
            }
            NS_TEST_EXPECT_MSG_EQ(m_sapUser[i].unexpected.size(),
                                  0,
                                  "other X2 SAP primitives at node " << i);
        }
    }

    Path m_path;
    std::array<Ptr<EpcX2>, 3> m_x2;
    std::array<X2SapUserStub, 3> m_sapUser;
    RlcUserStub m_rlcUserB;
    RlcUserStub m_rlcUserC;
    PdcpUserStub m_pdcpUserA;
    std::vector<X2Event> m_events;
};

/**
 * \ingroup lte-test
 * \brief The epc-x2-mc-pdcp-volume test suite.
 */
class EpcX2McPdcpVolumeTestSuite : public TestSuite
{
  public:
    EpcX2McPdcpVolumeTestSuite()
        : TestSuite("epc-x2-mc-pdcp-volume", UNIT)
    {
        using Case = EpcX2McPdcpVolumeTestCase;
        AddTestCase(new Case("master eNB to en-gNB: one DL event at the en-gNB", Case::MN_TO_SN),
                    TestCase::QUICK);
        AddTestCase(new Case("master eNB to an en-gNB that forwards the TEID: one DL event at "
                             "each en-gNB",
                             Case::MN_TO_SN_REDIRECTED),
                    TestCase::QUICK);
        AddTestCase(new Case("en-gNB RLC buffer PDU to another en-gNB: one DL event at the target",
                             Case::SN_RLC_BUFFER_TO_SN),
                    TestCase::QUICK);
        AddTestCase(new Case("en-gNB RLC to the master eNB PDCP: one UL event at the en-gNB",
                             Case::SN_UPLINK_TO_MN),
                    TestCase::QUICK);
        AddTestCase(
            new Case("untyped UE data to an en-gNB: no event", Case::MN_UNTYPED_UE_DATA_TO_SN),
            TestCase::QUICK);
    }
};

/// The static instance of the suite
static EpcX2McPdcpVolumeTestSuite g_epcX2McPdcpVolumeTestSuite;

} // namespace
