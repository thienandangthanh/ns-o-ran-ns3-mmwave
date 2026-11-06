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
 * \file lte-test-pdcp-sn-wrap.cc
 * \ingroup lte-test
 *
 * \brief Tests the lossless-handover forwarding buffer across the PDCP and RLC SN wraps.
 *
 * The SDUs forwarded at a lossless handover must come in COUNT order. The RLC AM numbers its
 * PDUs in the order the PDCP hands it SDUs, which is COUNT order, so the forwarding buffer is
 * read in RLC transmission order: GetUnackedPdusInTransmissionOrder walks the transmitting
 * window from VT(A) across the 10-bit RLC SN wrap, RlcPdusToRlcSdus reassembles it, and
 * GetTransmittingRlcSduBuffer returns each PDCP SN once, in first-reassembly order. The suite
 * checks that:
 *  - IsPdcpSnInForwardingLookback, which both the eNB RRC and the UE RRC call to pick the
 *    transmitted SDUs that precede the first forwarded SDU, accepts exactly that SN and the
 *    two SNs before it modulo 4096, including when the first forwarded SN is 0 or 1;
 *  - SDUs with PDCP SNs 4094, 4095, 0, 1 come back in that order, both from hand-built RLC
 *    PDUs and from an RLC AM driven through its SAPs, where the SDU with SN 4095 straddles RLC
 *    SNs 1023 and 0 and some PDUs sit in the retransmission buffer after a STATUS PDU;
 *  - SNs that never wrapped but lie far apart ({10, 3010}) keep their transmission order;
 *  - an RLC SN wrap without a PDCP SN wrap reassembles the straddling SDU whole;
 *  - without any wrap the order is the SN order (the order before this change).
 * Every SAP-driven case first checks the RLC SN and the framing info of the PDUs it built, so
 * a fixture that does not produce the intended PDU layout fails instead of passing.
 */

#include "ns3/lte-mac-sap.h"
#include "ns3/lte-pdcp-header.h"
#include "ns3/lte-pdcp-sn-wrap.h"
#include "ns3/lte-rlc-am-header.h"
#include "ns3/lte-rlc-am.h"
#include "ns3/lte-rlc-sap.h"
#include "ns3/lte-rlc-sequence-number.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <algorithm>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

const uint16_t RNTI = 1;
const uint8_t LCID = 3;

std::string
Join(const std::vector<uint16_t>& values)
{
    std::ostringstream out;
    for (std::size_t i = 0; i < values.size(); i++)
    {
        out << (i ? "," : "") << values[i];
    }
    return out.str();
}

/** The payload size of the i-th SDU, distinct per SDU so a swapped or cut packet is detected. */
uint32_t
PayloadSize(std::size_t i)
{
    return 100 + static_cast<uint32_t>(i);
}

/** A PDCP data PDU (the RLC SDU) carrying \p pdcpSn and \p payloadSize payload bytes. */
Ptr<Packet>
PdcpPdu(uint16_t pdcpSn, uint32_t payloadSize)
{
    Ptr<Packet> sdu = Create<Packet>(payloadSize);
    LtePdcpHeader pdcpHeader;
    pdcpHeader.SetDcBit(LtePdcpHeader::DATA_PDU);
    pdcpHeader.SetSequenceNumber(pdcpSn);
    sdu->AddHeader(pdcpHeader);
    return sdu;
}

/**
 * One RLC AM data PDU that carries exactly one complete PDCP data PDU (framing info: first
 * and last byte present, no length indicator), as LteRlcAm builds it for transmission.
 */
LteRlcAm::RetxPdu
SingleSduRlcPdu(uint16_t rlcSn, uint16_t pdcpSn, uint32_t payloadSize)
{
    Ptr<Packet> pdu = PdcpPdu(pdcpSn, payloadSize);
    LteRlcAmHeader rlcHeader;
    rlcHeader.SetDataPdu();
    rlcHeader.SetSequenceNumber(SequenceNumber10(rlcSn));
    rlcHeader.SetResegmentationFlag(LteRlcAmHeader::PDU);
    rlcHeader.SetLastSegmentFlag(LteRlcAmHeader::LAST_PDU_SEGMENT);
    rlcHeader.SetSegmentOffset(0);
    rlcHeader.SetFramingInfo(LteRlcAmHeader::FIRST_BYTE | LteRlcAmHeader::LAST_BYTE);
    rlcHeader.SetPollingBit(LteRlcAmHeader::STATUS_REPORT_NOT_REQUESTED);
    rlcHeader.PushExtensionBit(LteRlcAmHeader::DATA_FIELD_FOLLOWS);
    pdu->AddHeader(rlcHeader);

    LteRlcAm::RetxPdu retxPdu;
    retxPdu.m_pdu = pdu;
    retxPdu.m_retxCount = 0;
    return retxPdu;
}

/** A MAC SAP provider that keeps a copy of every PDU the RLC transmits. */
class CapturingMacSapProvider : public LteMacSapProvider
{
  public:
    void TransmitPdu(TransmitPduParameters params) override
    {
        m_pdus.push_back(params.pdu->Copy());
    }

    void ReportBufferStatus(ReportBufferStatusParameters params) override
    {
    }

    std::vector<Ptr<Packet>> m_pdus; ///< the transmitted PDUs, in transmission order
};

/** The RLC SN and the framing info of a transmitted AMD PDU. */
struct PduLayout
{
    uint16_t rlcSn;
    uint8_t framingInfo;
};

PduLayout
LayoutOf(Ptr<Packet> pdu)
{
    LteRlcAmHeader header;
    pdu->PeekHeader(header);
    return PduLayout{header.GetSequenceNumber().GetValue(), header.GetFramingInfo()};
}

} // namespace

/**
 * \ingroup lte-test
 * \brief IsPdcpSnInForwardingLookback over the whole 12-bit SN space for one first SN.
 */
class PdcpForwardingLookbackTestCase : public TestCase
{
  public:
    PdcpForwardingLookbackTestCase(uint16_t firstForwardedSn, std::vector<uint16_t> expected)
        : TestCase("eNB and UE forwarding lookback at first forwarded SN " +
                   std::to_string(firstForwardedSn)),
          m_firstForwardedSn(firstForwardedSn),
          m_expected(std::move(expected))
    {
    }

  private:
    void DoRun() override
    {
        std::vector<uint16_t> accepted;
        for (uint32_t sn = 0; sn < PDCP_SN_SPACE; sn++)
        {
            if (IsPdcpSnInForwardingLookback(static_cast<uint16_t>(sn), m_firstForwardedSn))
            {
                accepted.push_back(static_cast<uint16_t>(sn));
            }
        }
        NS_TEST_ASSERT_MSG_EQ(Join(accepted),
                              Join(m_expected),
                              "the first forwarded SN and its two predecessors modulo 4096");
    }

    uint16_t m_firstForwardedSn;
    std::vector<uint16_t> m_expected;
};

/**
 * \ingroup lte-test
 * \brief Common checks on the forwarding SDU buffer of an RLC AM entity.
 */
class RlcForwardingTestCase : public TestCase
{
  protected:
    explicit RlcForwardingTestCase(const std::string& name)
        : TestCase(name)
    {
    }

    void DoTeardown() override
    {
        // The RLC entity schedules timers and its buffer-size trace; the simulator never runs
        // here, so drop those events together with the entity.
        if (m_rlcAm)
        {
            m_rlcAm->Dispose();
            m_rlcAm = nullptr;
        }
        Simulator::Destroy();
    }

    /**
     * Reads GetTransmittingRlcSduBuffer and checks that it holds exactly \p expectedSns in that
     * order, that each SDU carries its own SN and that each is whole: its size is the PDCP
     * header plus \p payloadBySn of that SN. Returns the SDUs by SN for further checks.
     */
    std::map<uint16_t, Ptr<Packet>> CheckForwardingBuffer(
        const std::vector<uint16_t>& expectedSns,
        const std::map<uint16_t, uint32_t>& payloadBySn,
        const std::string& what)
    {
        std::vector<uint16_t> keys;
        std::map<uint16_t, Ptr<Packet>> bySn;
        bool intact = true;
        std::ostringstream problems;
        for (const auto& [sn, sdu] : m_rlcAm->GetTransmittingRlcSduBuffer())
        {
            keys.push_back(static_cast<uint16_t>(sn));
            bySn[static_cast<uint16_t>(sn)] = sdu;
            LtePdcpHeader pdcpHeader;
            sdu->PeekHeader(pdcpHeader);
            auto payload = payloadBySn.find(static_cast<uint16_t>(sn));
            if (pdcpHeader.GetSequenceNumber() != sn || payload == payloadBySn.end() ||
                sdu->GetSize() != pdcpHeader.GetSerializedSize() + payload->second)
            {
                intact = false;
                problems << " SN " << sn << ": header SN " << pdcpHeader.GetSequenceNumber() << ", "
                         << sdu->GetSize() << " bytes;";
            }
        }
        NS_TEST_EXPECT_MSG_EQ(Join(keys), Join(expectedSns), what << ": SDUs in COUNT order");
        NS_TEST_EXPECT_MSG_EQ(intact,
                              true,
                              what << ": each SDU whole and carrying its own SN:"
                                   << problems.str());
        return bySn;
    }

    Ptr<LteRlcAm> m_rlcAm; ///< the RLC entity under test
};

/**
 * \ingroup lte-test
 * \brief The SDU buffer filled by RlcPdusToRlcSdus from hand-built single-SDU RLC PDUs, given
 *        in transmission order.
 */
class RlcAmSduBufferOrderTestCase : public RlcForwardingTestCase
{
  public:
    RlcAmSduBufferOrderTestCase(const std::string& name, std::vector<uint16_t> pdcpSnsInTxOrder)
        : RlcForwardingTestCase(name),
          m_pdcpSnsInTxOrder(std::move(pdcpSnsInTxOrder))
    {
    }

  private:
    void DoRun() override
    {
        m_rlcAm = CreateObject<LteRlcAm>();
        std::vector<LteRlcAm::RetxPdu> pdus;
        std::map<uint16_t, uint32_t> payloadBySn;
        for (std::size_t i = 0; i < m_pdcpSnsInTxOrder.size(); i++)
        {
            pdus.push_back(
                SingleSduRlcPdu(static_cast<uint16_t>(i), m_pdcpSnsInTxOrder[i], PayloadSize(i)));
            payloadBySn[m_pdcpSnsInTxOrder[i]] = PayloadSize(i);
        }
        m_rlcAm->RlcPdusToRlcSdus(pdus);
        CheckForwardingBuffer(m_pdcpSnsInTxOrder, payloadBySn, "transmitting SDU buffer");
    }

    std::vector<uint16_t> m_pdcpSnsInTxOrder;
};

/**
 * \ingroup lte-test
 * \brief An RLC AM entity driven only through its RLC and MAC SAPs: it transmits SDUs from VT(A)
 *        on, may receive a STATUS PDU, and its unacknowledged PDUs are then reassembled into
 *        the forwarding SDU buffer as the eNB and UE RRC do at a lossless handover.
 */
class RlcAmTransmissionOrderTestCase : public RlcForwardingTestCase
{
  public:
    /// How the SDUs are mapped onto PDUs.
    enum Layout
    {
        /// one whole SDU per PDU
        WHOLE_SDUS,
        /// four SDUs: PDU VT(A) = [SDU 0], VT(A)+1 = [head of SDU 1],
        /// VT(A)+2 = [tail of SDU 1 + SDU 2], VT(A)+3 = [SDU 3]
        SECOND_SDU_STRADDLES
    };

    RlcAmTransmissionOrderTestCase(const std::string& name,
                                   uint16_t vtA,
                                   Layout layout,
                                   std::vector<uint16_t> pdcpSnsInTxOrder,
                                   bool nackFirstTwoPdus,
                                   bool repeatForwarding)
        : RlcForwardingTestCase(name),
          m_vtA(vtA),
          m_layout(layout),
          m_pdcpSnsInTxOrder(std::move(pdcpSnsInTxOrder)),
          m_nackFirstTwoPdus(nackFirstTwoPdus),
          m_repeatForwarding(repeatForwarding)
    {
    }

  private:
    void SendSdu(Ptr<Packet> sdu)
    {
        LteRlcSapProvider::TransmitPdcpPduParameters params;
        params.pdcpPdu = sdu;
        params.rnti = RNTI;
        params.lcid = LCID;
        m_rlcAm->GetLteRlcSapProvider()->TransmitPdcpPdu(params);
    }

    void TxOpportunity(uint32_t bytes)
    {
        LteMacSapUser::TxOpportunityParameters params;
        params.bytes = bytes;
        params.layer = 0;
        params.harqId = 0;
        params.componentCarrierId = 0;
        params.rnti = RNTI;
        params.lcid = LCID;
        m_rlcAm->GetLteMacSapUser()->NotifyTxOpportunity(params);
    }

    void ReceiveStatus(uint16_t ackSn, const std::vector<uint16_t>& nackSns)
    {
        LteRlcAmHeader header;
        header.SetControlPdu(LteRlcAmHeader::STATUS_PDU);
        header.SetAckSn(SequenceNumber10(ackSn));
        for (uint16_t nack : nackSns)
        {
            header.PushNack(nack);
        }
        Ptr<Packet> status = Create<Packet>();
        status->AddHeader(header);
        LteMacSapUser::ReceivePduParameters params;
        params.p = status;
        params.rnti = RNTI;
        params.lcid = LCID;
        m_rlcAm->GetLteMacSapUser()->ReceivePdu(params);
    }

    /// Transmits and acknowledges filler SDUs until VT(A) = VT(S) = m_vtA. Acknowledgements
    /// come at most every 256 PDUs, inside the 512-PDU transmitting window.
    void AdvanceWindow()
    {
        for (uint32_t sent = 0; sent < m_vtA;)
        {
            uint32_t batch = std::min<uint32_t>(256, m_vtA - sent);
            for (uint32_t i = 0; i < batch; i++, sent++)
            {
                SendSdu(PdcpPdu(static_cast<uint16_t>(sent % PDCP_SN_SPACE), 10));
                TxOpportunity(1000);
            }
            ReceiveStatus(static_cast<uint16_t>(sent % 1024), {});
        }
        m_mac.m_pdus.clear();
    }

    void DoRun() override
    {
        m_rlcAm = CreateObject<LteRlcAm>();
        m_rlcAm->SetRnti(RNTI);
        m_rlcAm->SetLcId(LCID);
        m_rlcAm->SetLteMacSapProvider(&m_mac);

        AdvanceWindow();
        NS_TEST_ASSERT_MSG_EQ(m_rlcAm->GetTxedBufferSize() + m_rlcAm->GetRetxBufferSize(),
                              0u,
                              "every filler PDU is acknowledged before the case starts");

        // Build the PDUs, then check their RLC SNs and framing info.
        std::map<uint16_t, uint32_t> payloadBySn;
        std::vector<PduLayout> expectedLayout;
        auto rlcSn = [this](uint16_t offset) {
            return static_cast<uint16_t>((m_vtA + offset) % 1024);
        };
        const uint8_t whole = LteRlcAmHeader::FIRST_BYTE | LteRlcAmHeader::LAST_BYTE;
        if (m_layout == WHOLE_SDUS)
        {
            for (std::size_t i = 0; i < m_pdcpSnsInTxOrder.size(); i++)
            {
                payloadBySn[m_pdcpSnsInTxOrder[i]] = PayloadSize(i);
                SendSdu(PdcpPdu(m_pdcpSnsInTxOrder[i], PayloadSize(i)));
                TxOpportunity(1000);
                expectedLayout.push_back(PduLayout{rlcSn(i), whole});
            }
        }
        else
        {
            NS_ASSERT(m_pdcpSnsInTxOrder.size() == 4);
            for (std::size_t i = 0; i < 4; i++)
            {
                payloadBySn[m_pdcpSnsInTxOrder[i]] = PayloadSize(i);
            }
            SendSdu(PdcpPdu(m_pdcpSnsInTxOrder[0], PayloadSize(0)));
            TxOpportunity(1000);
            // A 54-byte opportunity leaves 50 data bytes: the head of the 103-byte SDU 1.
            SendSdu(PdcpPdu(m_pdcpSnsInTxOrder[1], PayloadSize(1)));
            TxOpportunity(54);
            // The rest of SDU 1 and the whole SDU 2 share the next PDU.
            SendSdu(PdcpPdu(m_pdcpSnsInTxOrder[2], PayloadSize(2)));
            TxOpportunity(1000);
            SendSdu(PdcpPdu(m_pdcpSnsInTxOrder[3], PayloadSize(3)));
            TxOpportunity(1000);
            expectedLayout = {
                PduLayout{rlcSn(0), whole},
                PduLayout{rlcSn(1), LteRlcAmHeader::FIRST_BYTE | LteRlcAmHeader::NO_LAST_BYTE},
                PduLayout{rlcSn(2), LteRlcAmHeader::NO_FIRST_BYTE | LteRlcAmHeader::LAST_BYTE},
                PduLayout{rlcSn(3), whole}};
        }
        std::ostringstream want;
        std::ostringstream got;
        for (const PduLayout& l : expectedLayout)
        {
            want << l.rlcSn << "/FI" << static_cast<uint32_t>(l.framingInfo) << ";";
        }
        for (Ptr<Packet> pdu : m_mac.m_pdus)
        {
            PduLayout l = LayoutOf(pdu);
            got << l.rlcSn << "/FI" << static_cast<uint32_t>(l.framingInfo) << ";";
        }
        NS_TEST_ASSERT_MSG_EQ(got.str(),
                              want.str(),
                              "precondition: the PDUs have the intended RLC SNs and framing info");

        if (m_nackFirstTwoPdus)
        {
            // ACK_SN = VT(A) + 2 with NACK_SN {VT(A), VT(A) + 1}: those two move to the
            // retransmission buffer, the later two stay in the transmitted buffer.
            ReceiveStatus(rlcSn(2), {rlcSn(0), rlcSn(1)});
            NS_TEST_ASSERT_MSG_GT(m_rlcAm->GetRetxBufferSize(),
                                  0u,
                                  "precondition: the NACKed PDUs wait for retransmission");
            NS_TEST_ASSERT_MSG_GT(m_rlcAm->GetTxedBufferSize(),
                                  0u,
                                  "precondition: the later PDUs wait for acknowledgement");
        }

        m_rlcAm->RlcPdusToRlcSdus(m_rlcAm->GetUnackedPdusInTransmissionOrder());
        std::map<uint16_t, Ptr<Packet>> first =
            CheckForwardingBuffer(m_pdcpSnsInTxOrder, payloadBySn, "first forwarding");

        if (m_repeatForwarding)
        {
            // A second forwarding from the same entity: each SN once, as its latest copy. The
            // position of an SN across forwardings is not part of the contract.
            m_rlcAm->RlcPdusToRlcSdus(m_rlcAm->GetUnackedPdusInTransmissionOrder());
            std::vector<uint16_t> sns;
            bool replaced = true;
            for (const auto& [sn, sdu] : m_rlcAm->GetTransmittingRlcSduBuffer())
            {
                sns.push_back(static_cast<uint16_t>(sn));
                auto earlier = first.find(static_cast<uint16_t>(sn));
                replaced = replaced && earlier != first.end() && earlier->second != sdu &&
                           sdu->GetSize() == earlier->second->GetSize();
            }
            std::vector<uint16_t> expected = m_pdcpSnsInTxOrder;
            std::sort(sns.begin(), sns.end());
            std::sort(expected.begin(), expected.end());
            NS_TEST_EXPECT_MSG_EQ(Join(sns), Join(expected), "second forwarding: each SN once");
            NS_TEST_EXPECT_MSG_EQ(replaced,
                                  true,
                                  "second forwarding: each SDU is the latest whole copy");
        }
    }

    uint16_t m_vtA;
    Layout m_layout;
    std::vector<uint16_t> m_pdcpSnsInTxOrder;
    bool m_nackFirstTwoPdus;
    bool m_repeatForwarding;
    CapturingMacSapProvider m_mac;
};

/**
 * \ingroup lte-test
 * \brief The pdcp-sn-wrap test suite.
 */
class PdcpSnWrapTestSuite : public TestSuite
{
  public:
    PdcpSnWrapTestSuite()
        : TestSuite("pdcp-sn-wrap", UNIT)
    {
        using Rlc = RlcAmTransmissionOrderTestCase;

        // The previous-SDU lookback of the eNB and UE forwarding buffers.
        AddTestCase(new PdcpForwardingLookbackTestCase(0, {0, 4094, 4095}), TestCase::QUICK);
        AddTestCase(new PdcpForwardingLookbackTestCase(1, {0, 1, 4095}), TestCase::QUICK);
        AddTestCase(new PdcpForwardingLookbackTestCase(2, {0, 1, 2}), TestCase::QUICK);
        AddTestCase(new PdcpForwardingLookbackTestCase(4095, {4093, 4094, 4095}), TestCase::QUICK);

        // The SDU buffer over hand-built PDUs carrying PDCP SNs across the PDCP SN wrap.
        AddTestCase(new RlcAmSduBufferOrderTestCase("SDU buffer of PDCP SNs 4094, 4095, 0, 1",
                                                    {4094, 4095, 0, 1}),
                    TestCase::QUICK);
        AddTestCase(new RlcAmSduBufferOrderTestCase("no wrap: SDU buffer of PDCP SNs 10 to 13",
                                                    {10, 11, 12, 13}),
                    TestCase::QUICK);

        // PDCP SNs far apart that never wrapped keep their transmission order.
        AddTestCase(
            new Rlc("SAP: PDCP SNs sent as 10, 3010", 0, Rlc::WHOLE_SDUS, {10, 3010}, false, false),
            TestCase::QUICK);
        AddTestCase(
            new Rlc("SAP: PDCP SNs sent as 3010, 10", 0, Rlc::WHOLE_SDUS, {3010, 10}, false, false),
            TestCase::QUICK);

        // PDCP and RLC SN wraps together: SDU 4095 straddles RLC SNs 1023 and 0.
        AddTestCase(new Rlc("SAP: VT(A) 1022, PDCP SNs 4094, 4095, 0, 1",
                            1022,
                            Rlc::SECOND_SDU_STRADDLES,
                            {4094, 4095, 0, 1},
                            false,
                            false),
                    TestCase::QUICK);
        AddTestCase(new Rlc("SAP: VT(A) 1022, PDCP SNs 4094, 4095, 0, 1, NACK 1022 and 1023",
                            1022,
                            Rlc::SECOND_SDU_STRADDLES,
                            {4094, 4095, 0, 1},
                            true,
                            false),
                    TestCase::QUICK);

        // No wrap at all: the SN order, unchanged.
        AddTestCase(new Rlc("no wrap: SAP: VT(A) 5, PDCP SNs 100 to 103",
                            5,
                            Rlc::SECOND_SDU_STRADDLES,
                            {100, 101, 102, 103},
                            false,
                            false),
                    TestCase::QUICK);
        AddTestCase(new Rlc("no wrap: SAP: VT(A) 5, PDCP SNs 100 to 103, NACK 5 and 6",
                            5,
                            Rlc::SECOND_SDU_STRADDLES,
                            {100, 101, 102, 103},
                            true,
                            false),
                    TestCase::QUICK);

        // An RLC SN wrap without a PDCP SN wrap: SDU 101 straddles RLC SNs 1023 and 0; a second
        // forwarding returns each SN once, as its latest copy.
        AddTestCase(new Rlc("SAP: VT(A) 1022, PDCP SNs 100 to 103, forwarded twice",
                            1022,
                            Rlc::SECOND_SDU_STRADDLES,
                            {100, 101, 102, 103},
                            false,
                            true),
                    TestCase::QUICK);
    }
};

/// The static instance that registers the suite.
static PdcpSnWrapTestSuite g_pdcpSnWrapTestSuite;
