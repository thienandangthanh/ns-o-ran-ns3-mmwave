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
 * \file lte-test-mc-pdcp-nr-header.cc
 * \ingroup lte-test
 *
 * \brief Tests the NR PDCP data-PDU header of the dual-connectivity (EN-DC) bearer.
 *
 * In EN-DC an MN-terminated split bearer uses NR PDCP (3GPP TS 37.340 clause 4.2.2). Its Data
 * PDU with a 12-bit SN has a two-octet header (3GPP TS 38.323 V19.1.0 clause 6.2.2.2, Figure
 * 6.2.2.2-1): octet 1 = D/C, R, R, R, SN[11:8]; octet 2 = SN[7:0]; D/C = 1 for a Data PDU
 * (clause 6.3.7); R = 0 on transmission and ignored on reception (clause 6.3.6). The suite checks:
 *  - the NrPdcpHeader codec at SNs 0, 1, 2047, 2048 and 4095;
 *  - that McEnbPdcp (downlink) and McUePdcp (uplink) carry NrPdcpHeader, in their type and at
 *    run time, and that a Data PDU they transmit at each of those SNs is the SDU plus exactly
 *    those two octets, with the payload byte for byte;
 *  - that McUePdcp (downlink) and McEnbPdcp (uplink) deliver the SDU of a hand-built Data PDU
 *    byte for byte and advance their RX SN, also across 4095 -> 0;
 *  - that the E-UTRA PDCP entity of an SRB keeps LtePdcpHeader and its two-octet header;
 *  - payload preservation through the forwarding of the RLC AM buffers, run by the per-bearer
 *    functions the eNB (lossless handover, switch to mmWave) and the UE (switch of RLC entity)
 *    forwarding call: SDUs sent by an MC PDCP entity through an RLC AM entity, unacknowledged or
 *    still queued, are collected keyed by the SN of the bearer's NR header, across the PDCP SN
 *    wrap 4095 -> 0 (with and without an RLC SN wrap and a STATUS PDU), then delivered without
 *    packet tags and, for SDU forwarding, without that header: on the eNB path into a target
 *    McEnbPdcp or, as whole PDUs, to the target RLC; on the UE path back into the same
 *    McUePdcp. The PDUs sent again carry new SNs, and the peer entity delivers the original SDU
 *    bytes in the forwarded order;
 *  - the forwarding of an RLC UM low-latency buffer whose head is the unsent tail of a cut SDU:
 *    the whole-PDU forward of the secondary cell handover drops the tail (an RLC segment), and
 *    the X2 SDU forward pins its recorded tail defect.
 */

#include "ns3/abort.h"
#include "ns3/lte-mac-sap.h"
#include "ns3/lte-pdcp-header.h"
#include "ns3/lte-pdcp-sap.h"
#include "ns3/lte-pdcp.h"
#include "ns3/lte-rlc-am-header.h"
#include "ns3/lte-rlc-am.h"
#include "ns3/lte-rlc-forwarding-buffer.h"
#include "ns3/lte-rlc-sap.h"
#include "ns3/lte-rlc-sdu-status-tag.h"
#include "ns3/lte-rlc-sequence-number.h"
#include "ns3/lte-rlc-um-lowlat.h"
#include "ns3/mc-enb-pdcp.h"
#include "ns3/mc-ue-pdcp.h"
#include "ns3/nr-pdcp-header.h"
#include "ns3/packet-metadata.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

using namespace ns3;

namespace
{

const uint16_t RNTI = 1;
const uint8_t DRB_LCID = 3;
const uint8_t SRB1_LCID = 1;
const uint32_t NR_PDCP_HEADER_OCTETS = 2;

using Bytes = std::vector<uint8_t>;

/** The SNs at which the header layout is checked: both ends and both sides of the SN[11:8] edge. */
const std::vector<uint16_t> LAYOUT_SNS = {0, 1, 2047, 2048, 4095};

/** \p size payload bytes, distinct per \p seed and position, so a cut or swapped SDU is seen. */
Bytes
Payload(uint32_t seed, uint32_t size)
{
    Bytes bytes(size);
    for (uint32_t i = 0; i < size; i++)
    {
        bytes[i] = static_cast<uint8_t>(seed * 31 + i * 7 + 1);
    }
    return bytes;
}

Ptr<Packet>
PacketOf(const Bytes& bytes)
{
    return Create<Packet>(bytes.data(), static_cast<uint32_t>(bytes.size()));
}

Bytes
BytesOf(Ptr<const Packet> p)
{
    Bytes bytes(p->GetSize());
    p->CopyData(bytes.data(), static_cast<uint32_t>(bytes.size()));
    return bytes;
}

std::string
Hex(const Bytes& bytes, std::size_t max = 6)
{
    std::ostringstream out;
    out << std::hex;
    for (std::size_t i = 0; i < std::min(max, bytes.size()); i++)
    {
        out << (i ? " " : "") << static_cast<uint32_t>(bytes[i]);
    }
    return out.str();
}

/** The two header octets of a Data PDU at \p sn, written out from the clause, not by a codec. */
Bytes
ExpectedDataPduHeaderOctets(uint16_t sn)
{
    return Bytes{static_cast<uint8_t>(0x80 | (sn >> 8)), static_cast<uint8_t>(sn & 0xFF)};
}

/** A Data PDU built byte by byte: the header octets of \p sn, then \p sdu. */
Ptr<Packet>
HandBuiltDataPdu(uint16_t sn, const Bytes& sdu)
{
    Bytes pdu = ExpectedDataPduHeaderOctets(sn);
    pdu.insert(pdu.end(), sdu.begin(), sdu.end());
    return PacketOf(pdu);
}

/**
 * Turns on packet metadata, so that a packet records the class of each header added to it.
 * It must come before the first packet of the process, which holds when test.py runs this suite
 * in its own process.
 */
void
EnableHeaderMetadata()
{
    Packet::EnablePrinting();
}

/**
 * The TypeId name of the outermost header recorded in \p p's metadata, the class whose
 * Serialize wrote the first octets: "<no header>" when the packet starts with raw bytes. At 12
 * bits the E-UTRA and NR PDCP headers are byte-identical, so only this tells which one was used.
 */
std::string
OuterHeaderClass(Ptr<const Packet> p)
{
    PacketMetadata::ItemIterator items = p->BeginItem();
    if (items.HasNext())
    {
        PacketMetadata::Item item = items.Next();
        if (item.type == PacketMetadata::Item::HEADER)
        {
            return item.tid.GetName();
        }
    }
    return "<no header>";
}

/** The TypeId name and size of \p pdcp's own Data PDU header object. */
std::string
EntityHeader(Ptr<const LtePdcp> pdcp)
{
    std::unique_ptr<Header> header = pdcp->CreateDataPduHeader();
    return header->GetInstanceTypeId().GetName() + "/" +
           std::to_string(header->GetSerializedSize());
}

/** An RLC SAP provider that keeps a copy of every PDCP PDU handed to it. */
class CapturingRlcSapProvider : public LteRlcSapProvider
{
  public:
    void TransmitPdcpPdu(TransmitPdcpPduParameters params) override
    {
        m_pdus.push_back(params.pdcpPdu->Copy());
    }

    std::vector<Ptr<Packet>> m_pdus; ///< the PDCP PDUs, in order
};

/** A PDCP SAP user that keeps every delivered SDU. */
class CapturingPdcpSapUser : public LtePdcpSapUser
{
  public:
    void ReceivePdcpSdu(ReceivePdcpSduParameters params) override
    {
        m_sdus.push_back(BytesOf(params.pdcpSdu));
    }

    std::vector<Bytes> m_sdus; ///< the delivered SDUs, in order
};

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

/** Hands \p sdu to \p pdcp through its PDCP SAP, as the RRC does. */
template <class Pdcp>
void
TransmitSdu(Ptr<Pdcp> pdcp, Ptr<Packet> sdu, uint8_t lcid)
{
    LtePdcpSapProvider::TransmitPdcpSduParameters params;
    params.pdcpSdu = sdu;
    params.rnti = RNTI;
    params.lcid = lcid;
    pdcp->GetLtePdcpSapProvider()->TransmitPdcpSdu(params);
}

/** Wires \p pdcp between a capturing RLC SAP provider and a capturing PDCP SAP user. */
template <class Pdcp>
void
Wire(Ptr<Pdcp> pdcp, uint8_t lcid, CapturingRlcSapProvider* rlc, CapturingPdcpSapUser* user)
{
    pdcp->SetRnti(RNTI);
    pdcp->SetLcId(lcid);
    pdcp->SetLtePdcpSapUser(user);
    pdcp->SetLteRlcSapProvider(rlc);
}

/** The status with TX SN \p txSn and RX SN 0 for the entity type \p Pdcp. */
template <class Pdcp>
typename Pdcp::Status
TxStatus(uint16_t txSn)
{
    typename Pdcp::Status status;
    status.txSn = txSn;
    status.rxSn = 0;
    return status;
}

/** Disposes the PDCP entity and drops any events the entities under test scheduled. */
template <class Pdcp>
void
DisposeAll(Ptr<Pdcp> pdcp)
{
    if (pdcp)
    {
        pdcp->Dispose();
    }
    Simulator::Destroy();
}

} // namespace

/**
 * \ingroup lte-test
 * \brief The NrPdcpHeader codec at one SN: octets on serialisation, fields on deserialisation.
 */
class NrPdcpHeaderCodecTestCase : public TestCase
{
  public:
    explicit NrPdcpHeaderCodecTestCase(uint16_t sn)
        : TestCase("NrPdcpHeader codec at SN " + std::to_string(sn)),
          m_sn(sn)
    {
    }

  private:
    void DoRun() override
    {
        EnableHeaderMetadata();
        NrPdcpHeader header;
        header.SetDcBit(NrPdcpHeader::DATA_PDU);
        header.SetSequenceNumber(m_sn);
        NS_TEST_ASSERT_MSG_EQ(header.GetSerializedSize(), NR_PDCP_HEADER_OCTETS, "two octets");
        NS_TEST_ASSERT_MSG_EQ(header.GetInstanceTypeId().GetName(),
                              "ns3::NrPdcpHeader",
                              "TypeId name");

        Ptr<Packet> p = Create<Packet>();
        p->AddHeader(header);
        NS_TEST_ASSERT_MSG_EQ(Hex(BytesOf(p)),
                              Hex(ExpectedDataPduHeaderOctets(m_sn)),
                              "octet 1 = D/C 1, R R R 0, SN[11:8]; octet 2 = SN[7:0]");

        // The receiver ignores the R bits (clause 6.3.6): set them all and decode.
        Bytes withR = ExpectedDataPduHeaderOctets(m_sn);
        withR[0] |= 0x70;
        NrPdcpHeader decoded;
        PacketOf(withR)->RemoveHeader(decoded);
        NS_TEST_ASSERT_MSG_EQ(+decoded.GetDcBit(), +NrPdcpHeader::DATA_PDU, "D/C decoded as data");
        NS_TEST_ASSERT_MSG_EQ(decoded.GetSequenceNumber(), m_sn, "SN decoded, R bits ignored");

        // D/C 0 is decoded as a Control PDU, so a consumer can tell it apart.
        Bytes control = ExpectedDataPduHeaderOctets(m_sn);
        control[0] &= 0x7F;
        NrPdcpHeader decodedControl;
        PacketOf(control)->RemoveHeader(decodedControl);
        NS_TEST_ASSERT_MSG_EQ(+decodedControl.GetDcBit(),
                              +NrPdcpHeader::CONTROL_PDU,
                              "D/C 0 decoded as control");
    }

    uint16_t m_sn;
};

/**
 * \ingroup lte-test
 * \brief The header class of the MC PDCP entities, in their type and at run time, and of the
 *        E-UTRA PDCP entity.
 */
class McPdcpHeaderClassTestCase : public TestCase
{
  public:
    McPdcpHeaderClassTestCase()
        : TestCase("McEnbPdcp and McUePdcp carry NrPdcpHeader; LtePdcp carries LtePdcpHeader")
    {
    }

  private:
    void DoRun() override
    {
        EnableHeaderMetadata();
        bool enbIsNr = std::is_same<McEnbPdcp::DataPduHeader, NrPdcpHeader>::value;
        bool ueIsNr = std::is_same<McUePdcp::DataPduHeader, NrPdcpHeader>::value;
        NS_TEST_EXPECT_MSG_EQ(enbIsNr, true, "McEnbPdcp::DataPduHeader is NrPdcpHeader");
        NS_TEST_EXPECT_MSG_EQ(ueIsNr, true, "McUePdcp::DataPduHeader is NrPdcpHeader");

        Ptr<LtePdcp> enb = CreateObject<McEnbPdcp>();
        Ptr<LtePdcp> ue = CreateObject<McUePdcp>();
        Ptr<LtePdcp> srb = CreateObject<LtePdcp>();
        NS_TEST_EXPECT_MSG_EQ(EntityHeader(enb),
                              "ns3::NrPdcpHeader/2",
                              "McEnbPdcp's header object, through the base class");
        NS_TEST_EXPECT_MSG_EQ(EntityHeader(ue),
                              "ns3::NrPdcpHeader/2",
                              "McUePdcp's header object, through the base class");
        NS_TEST_EXPECT_MSG_EQ(EntityHeader(srb), "ns3::LtePdcpHeader/2", "LtePdcp's header object");
        enb->Dispose();
        ue->Dispose();
        srb->Dispose();
        Simulator::Destroy();
    }
};

/**
 * \ingroup lte-test
 * \brief One Data PDU transmitted by an MC PDCP entity at a given SN: the SDU plus exactly the
 *        two NR header octets, the payload byte for byte, TX SN advanced modulo 4096.
 */
template <class Pdcp>
class McPdcpDataPduTxTestCase : public TestCase
{
  public:
    McPdcpDataPduTxTestCase(const std::string& direction, uint16_t sn)
        : TestCase(direction + " Data PDU transmitted at SN " + std::to_string(sn)),
          m_sn(sn)
    {
    }

  private:
    void DoRun() override
    {
        EnableHeaderMetadata();
        m_pdcp = CreateObject<Pdcp>();
        Wire(m_pdcp, DRB_LCID, &m_rlc, &m_user);
        m_pdcp->SetStatus(TxStatus<Pdcp>(m_sn));

        const Bytes sdu = Payload(m_sn, 100);
        TransmitSdu(m_pdcp, PacketOf(sdu), DRB_LCID);

        NS_TEST_ASSERT_MSG_EQ(m_rlc.m_pdus.size(), 1u, "one PDU handed to the RLC");
        if (m_rlc.m_pdus.size() != 1u)
        {
            return; // the runner continues after a failed assertion by default
        }
        Bytes pdu = BytesOf(m_rlc.m_pdus.front());
        NS_TEST_ASSERT_MSG_EQ(pdu.size(), sdu.size() + NR_PDCP_HEADER_OCTETS, "PDU = SDU + 2");
        NS_TEST_EXPECT_MSG_EQ(Hex(Bytes(pdu.begin(), pdu.begin() + NR_PDCP_HEADER_OCTETS)),
                              Hex(ExpectedDataPduHeaderOctets(m_sn)),
                              "the header octets of clause 6.2.2.2");
        bool payloadEqual = std::equal(sdu.begin(), sdu.end(), pdu.begin() + NR_PDCP_HEADER_OCTETS);
        NS_TEST_EXPECT_MSG_EQ(payloadEqual, true, "the payload equals the SDU byte for byte");
        NS_TEST_EXPECT_MSG_EQ(m_pdcp->GetStatus().txSn,
                              static_cast<uint16_t>((m_sn + 1) % NrPdcpHeader::SN_SPACE),
                              "TX SN advanced modulo 4096");
        NS_TEST_EXPECT_MSG_EQ(OuterHeaderClass(m_rlc.m_pdus.front()),
                              "ns3::NrPdcpHeader",
                              "the class that wrote the header octets");
        NS_TEST_EXPECT_MSG_EQ(EntityHeader(m_pdcp), "ns3::NrPdcpHeader/2", "the entity's header");
    }

    void DoTeardown() override
    {
        DisposeAll(m_pdcp);
        m_pdcp = nullptr;
    }

    uint16_t m_sn;
    Ptr<Pdcp> m_pdcp;
    CapturingRlcSapProvider m_rlc;
    CapturingPdcpSapUser m_user;
};

/**
 * \ingroup lte-test
 * \brief A hand-built Data PDU received by an MC PDCP entity: it delivers the SDU byte for byte
 *        and sets its RX SN to the next SN modulo 4096.
 */
template <class Pdcp>
class McPdcpDataPduRxTestCase : public TestCase
{
  public:
    McPdcpDataPduRxTestCase(const std::string& direction, uint16_t sn)
        : TestCase(direction + " hand-built Data PDU received at SN " + std::to_string(sn)),
          m_sn(sn)
    {
    }

  private:
    void DoRun() override
    {
        EnableHeaderMetadata();
        m_pdcp = CreateObject<Pdcp>();
        Wire(m_pdcp, DRB_LCID, &m_rlc, &m_user);

        // Larger than the 40-byte minimum below which the MC entities deliver nothing.
        const Bytes sdu = Payload(m_sn + 7, 120);
        m_pdcp->GetLteRlcSapUser()->ReceivePdcpPdu(HandBuiltDataPdu(m_sn, sdu));

        NS_TEST_ASSERT_MSG_EQ(m_user.m_sdus.size(), 1u, "one SDU delivered");
        if (m_user.m_sdus.size() != 1u)
        {
            return; // the runner continues after a failed assertion by default
        }
        NS_TEST_EXPECT_MSG_EQ(m_user.m_sdus.front().size(), sdu.size(), "SDU size");
        NS_TEST_EXPECT_MSG_EQ((m_user.m_sdus.front() == sdu), true, "SDU byte for byte");
        NS_TEST_EXPECT_MSG_EQ(m_pdcp->GetStatus().rxSn,
                              static_cast<uint16_t>((m_sn + 1) % NrPdcpHeader::SN_SPACE),
                              "RX SN advanced modulo 4096");
    }

    void DoTeardown() override
    {
        DisposeAll(m_pdcp);
        m_pdcp = nullptr;
    }

    uint16_t m_sn;
    Ptr<Pdcp> m_pdcp;
    CapturingRlcSapProvider m_rlc;
    CapturingPdcpSapUser m_user;
};

/**
 * \ingroup lte-test
 * \brief An MC PDCP entity discards a received PDCP Control PDU (D/C 0, 3GPP TS 38.323 V19.1.0
 *        clauses 6.3.7 and 6.3.8): the entity is configured with no Control PDU, so the PDU
 *        carries reserved or invalid values for it (clause 5.10). After a Data PDU at SN 2047,
 *        a Control PDU larger than the delivery minimum delivers nothing and leaves RX SN 2048.
 */
template <class Pdcp>
class McPdcpControlPduRxTestCase : public TestCase
{
  public:
    McPdcpControlPduRxTestCase(const std::string& direction, uint8_t pduType)
        : TestCase(direction + " PDCP Control PDU of PDU type " + std::to_string(pduType) +
                   " discarded"),
          m_pduType(pduType)
    {
    }

  private:
    void DoRun() override
    {
        EnableHeaderMetadata();
        m_pdcp = CreateObject<Pdcp>();
        Wire(m_pdcp, DRB_LCID, &m_rlc, &m_user);
        uint32_t rxPdus = 0;
        m_pdcp->TraceConnectWithoutContext(
            "RxPDU",
            MakeBoundCallback(
                +[](uint32_t* n, uint16_t, uint8_t, uint32_t, uint64_t) { ++*n; },
                &rxPdus));

        const uint16_t sn = 2047;
        const Bytes sdu = Payload(sn + 7, 120);
        m_pdcp->GetLteRlcSapUser()->ReceivePdcpPdu(HandBuiltDataPdu(sn, sdu));

        // Octet 1 = D/C 0, PDU type, R R R R (Figure 6.2.3.1-1), then 100 octets: large enough
        // that a parse as a Data PDU would deliver an SDU and move RX SN.
        Bytes control(1, static_cast<uint8_t>(m_pduType << 4));
        const Bytes body = Payload(m_pduType + 3, 100);
        control.insert(control.end(), body.begin(), body.end());
        m_pdcp->GetLteRlcSapUser()->ReceivePdcpPdu(PacketOf(control));

        NS_TEST_EXPECT_MSG_EQ(rxPdus, 2u, "both PDUs counted as received from the RLC");
        NS_TEST_EXPECT_MSG_EQ(m_user.m_sdus.size(), 1u, "only the Data PDU's SDU delivered");
        NS_TEST_EXPECT_MSG_EQ(m_pdcp->GetStatus().rxSn,
                              static_cast<uint16_t>(sn + 1),
                              "RX SN unchanged by the Control PDU");
    }

    void DoTeardown() override
    {
        DisposeAll(m_pdcp);
        m_pdcp = nullptr;
    }

    uint8_t m_pduType;
    Ptr<Pdcp> m_pdcp;
    CapturingRlcSapProvider m_rlc;
    CapturingPdcpSapUser m_user;
};

/**
 * \ingroup lte-test
 * \brief The E-UTRA PDCP entity of SRB1 is unchanged: LtePdcpHeader, two octets with the same
 *        layout on transmission, and a hand-built PDU delivered with its RX SN advanced.
 */
class SrbLtePdcpUnchangedTestCase : public TestCase
{
  public:
    explicit SrbLtePdcpUnchangedTestCase(uint16_t sn)
        : TestCase("SRB1 LtePdcp unchanged at SN " + std::to_string(sn)),
          m_sn(sn)
    {
    }

  private:
    void DoRun() override
    {
        EnableHeaderMetadata();
        m_pdcp = CreateObject<LtePdcp>();
        Wire(m_pdcp, SRB1_LCID, &m_rlc, &m_user);
        NS_TEST_ASSERT_MSG_EQ(EntityHeader(m_pdcp),
                              "ns3::LtePdcpHeader/2",
                              "the SRB keeps the E-UTRA PDCP header");

        LtePdcp::Status status;
        status.txSn = m_sn;
        status.rxSn = 0;
        m_pdcp->SetStatus(status);
        const Bytes sdu = Payload(m_sn + 3, 30);
        TransmitSdu(m_pdcp, PacketOf(sdu), SRB1_LCID);
        NS_TEST_ASSERT_MSG_EQ(m_rlc.m_pdus.size(), 1u, "one PDU handed to the RLC");
        if (m_rlc.m_pdus.size() != 1u)
        {
            return; // the runner continues after a failed assertion by default
        }
        Bytes pdu = BytesOf(m_rlc.m_pdus.front());
        NS_TEST_ASSERT_MSG_EQ(pdu.size(),
                              sdu.size() + LtePdcpHeader().GetSerializedSize(),
                              "PDU = SDU + the E-UTRA header");
        NS_TEST_EXPECT_MSG_EQ(Hex(Bytes(pdu.begin(), pdu.begin() + 2)),
                              Hex(ExpectedDataPduHeaderOctets(m_sn)),
                              "the E-UTRA 12-bit SN header octets");
        bool payloadEqual = std::equal(sdu.begin(), sdu.end(), pdu.begin() + 2);
        NS_TEST_EXPECT_MSG_EQ(payloadEqual, true, "the payload equals the SDU byte for byte");
        NS_TEST_EXPECT_MSG_EQ(OuterHeaderClass(m_rlc.m_pdus.front()),
                              "ns3::LtePdcpHeader",
                              "the class that wrote the header octets");

        m_pdcp->GetLteRlcSapUser()->ReceivePdcpPdu(HandBuiltDataPdu(m_sn, sdu));
        NS_TEST_ASSERT_MSG_EQ(m_user.m_sdus.size(), 1u, "one SDU delivered");
        if (m_user.m_sdus.size() != 1u)
        {
            return; // the runner continues after a failed assertion by default
        }
        NS_TEST_EXPECT_MSG_EQ((m_user.m_sdus.front() == sdu), true, "SDU byte for byte");
        NS_TEST_EXPECT_MSG_EQ(m_pdcp->GetStatus().rxSn,
                              static_cast<uint16_t>((m_sn + 1) % 4096),
                              "RX SN advanced modulo 4096");
    }

    void DoTeardown() override
    {
        DisposeAll(m_pdcp);
        m_pdcp = nullptr;
    }

    uint16_t m_sn;
    Ptr<LtePdcp> m_pdcp;
    CapturingRlcSapProvider m_rlc;
    CapturingPdcpSapUser m_user;
};

namespace
{

/** Where a forwarding hands the PDCP packets it recovered from the RLC buffers. */
enum class ForwardingTarget
{
    ENB_TARGET_PDCP, ///< eNB: PDCP SDUs, sent again by the target's McEnbPdcp (X2 forwarding)
    ENB_TARGET_RLC,  ///< eNB: whole PDCP PDUs, for the target's RLC (secondary cell handover)
    UE_SAME_PDCP,    ///< UE: PDCP SDUs, sent again by the same McUePdcp (switch of RLC entity)
};

/** Whether \p p carries any packet tag (byte tags are not counted). */
bool
HasPacketTags(Ptr<const Packet> p)
{
    return p->GetPacketTagIterator().HasNext();
}

} // namespace

/**
 * \ingroup lte-test
 * \brief Payload preservation through the forwarding of the RLC AM buffers of a
 *        dual-connectivity bearer, run by the per-bearer functions that UeManager::
 *        ForwardRlcBuffers and LteUeRrc::CopyRlcBuffers call (CollectRlcForwardingBuffer,
 *        DeliverRlcForwardingBuffer), with the bearer's NR PDCP header class.
 *
 * \tparam SourcePdcp the MC entity that transmits the SDUs and whose RLC AM entity forwards them
 * \tparam PeerPdcp the MC entity on the other side of the air interface
 *
 * The source sends four SDUs from PDCP SN 4094 (SNs 4094, 4095, 0, 1) to an RLC AM entity at
 * VT(A), driven only through its SAPs. Either the RLC sends them all and nothing is
 * acknowledged (or, with a STATUS PDU, the first two PDUs are NACKed into the retransmission
 * buffer); the second SDU straddles two RLC PDUs, so at VT(A) 1022 it straddles the RLC SN wrap
 * 1023 -> 0. Or the RLC gets no transmission opportunity and the SDUs stay queued.
 *
 * CollectRlcForwardingBuffer must then hold SNs 4094, 4095, 0, 1 in that order, each SDU whole
 * under the NR header the source wrote. When the SDUs were sent, the RLC AM's copy of the last
 * SDU it started (SN 1) follows them: the RLC AM keeps that copy after the SDU has gone out
 * whole and never resets it, so the forwarding sends SDU 3 a second time. That repetition is
 * an RLC AM defect outside the header class; the case pins it, so a fix shows up here.
 *
 * DeliverRlcForwardingBuffer then hands every packet, without packet tags, to the target as
 * the production callers do:
 *  - eNB, PDCP SDUs (the NR header removed): into a target McEnbPdcp whose TX SN is 4095;
 *  - eNB, PDCP PDUs (the NR header kept, original SN): straight to the peer;
 *  - UE, PDCP SDUs (the NR header removed): back into the same McUePdcp, now wired to another
 *    RLC, at TX SN 2.
 * The PDUs sent again must carry the new SNs (an SN change, wrapping again on the eNB path),
 * and the peer entity must deliver the original SDUs byte for byte, in the forwarded order.
 */
template <class SourcePdcp, class PeerPdcp>
class McPdcpForwardingTestCase : public TestCase
{
  public:
    McPdcpForwardingTestCase(const std::string& name,
                             ForwardingTarget target,
                             uint16_t vtA,
                             bool transmitted,
                             bool nackFirstTwoPdus)
        : TestCase(name),
          m_target(target),
          m_vtA(vtA),
          m_transmitted(transmitted),
          m_nackFirstTwoPdus(nackFirstTwoPdus)
    {
        NS_ABORT_MSG_IF(IS_ENB == (target == ForwardingTarget::UE_SAME_PDCP),
                        "the forwarding target does not match the source entity");
        NS_ABORT_MSG_IF(nackFirstTwoPdus && !transmitted, "a NACK needs transmitted PDUs");
    }

  private:
    using PdcpHeader = typename SourcePdcp::DataPduHeader;

    static constexpr bool IS_ENB = std::is_same<SourcePdcp, McEnbPdcp>::value;

    void TxOpportunity(uint32_t bytes)
    {
        LteMacSapUser::TxOpportunityParameters params;
        params.bytes = bytes;
        params.layer = 0;
        params.harqId = 0;
        params.componentCarrierId = 0;
        params.rnti = RNTI;
        params.lcid = DRB_LCID;
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
        params.lcid = DRB_LCID;
        m_rlcAm->GetLteMacSapUser()->ReceivePdu(params);
    }

    uint16_t RlcSn(uint16_t offset) const
    {
        return static_cast<uint16_t>((m_vtA + offset) % 1024);
    }

    /// Moves VT(A) = VT(S) to m_vtA with acknowledged filler PDUs, at most 256 per STATUS PDU.
    void AdvanceWindow()
    {
        LteRlcSapProvider* rlcSap = m_rlcAm->GetLteRlcSapProvider();
        for (uint32_t sent = 0; sent < m_vtA;)
        {
            uint32_t batch = std::min<uint32_t>(256, m_vtA - sent);
            for (uint32_t i = 0; i < batch; i++, sent++)
            {
                LteRlcSapProvider::TransmitPdcpPduParameters params;
                params.pdcpPdu = HandBuiltDataPdu(static_cast<uint16_t>(sent % 4096), Bytes(10));
                params.rnti = RNTI;
                params.lcid = DRB_LCID;
                rlcSap->TransmitPdcpPdu(params);
                TxOpportunity(1000);
            }
            ReceiveStatus(static_cast<uint16_t>(sent % 1024), {});
        }
        m_mac.m_pdus.clear();
    }

    /// Checks that the four SDUs went out in the intended RLC PDUs (SN and framing info).
    void CheckRlcPdus()
    {
        std::ostringstream want;
        std::ostringstream got;
        const uint8_t whole = LteRlcAmHeader::FIRST_BYTE | LteRlcAmHeader::LAST_BYTE;
        const uint8_t framing[] = {whole,
                                   LteRlcAmHeader::FIRST_BYTE | LteRlcAmHeader::NO_LAST_BYTE,
                                   LteRlcAmHeader::NO_FIRST_BYTE | LteRlcAmHeader::LAST_BYTE,
                                   whole};
        for (uint16_t i = 0; i < 4; i++)
        {
            want << RlcSn(i) << "/FI" << static_cast<uint32_t>(framing[i]) << ";";
        }
        for (Ptr<Packet> pdu : m_mac.m_pdus)
        {
            LteRlcAmHeader header;
            pdu->PeekHeader(header);
            got << header.GetSequenceNumber().GetValue() << "/FI"
                << static_cast<uint32_t>(header.GetFramingInfo()) << ";";
        }
        NS_TEST_ASSERT_MSG_EQ(got.str(),
                              want.str(),
                              "precondition: the PDUs have the intended RLC SNs and framing info");
    }

    void DoRun() override
    {
        EnableHeaderMetadata();
        // Source: the MC entity above an RLC AM entity.
        m_rlcAm = CreateObject<LteRlcAm>();
        m_rlcAm->SetRnti(RNTI);
        m_rlcAm->SetLcId(DRB_LCID);
        m_rlcAm->SetLteMacSapProvider(&m_mac);
        AdvanceWindow();
        NS_TEST_ASSERT_MSG_EQ(m_rlcAm->GetTxedBufferSize() + m_rlcAm->GetRetxBufferSize(),
                              0u,
                              "precondition: every filler PDU is acknowledged");

        m_source = CreateObject<SourcePdcp>();
        m_source->SetRnti(RNTI);
        m_source->SetLcId(DRB_LCID);
        m_source->SetLtePdcpSapUser(&m_sourceUser);
        m_source->SetLteRlcSapProvider(m_rlcAm->GetLteRlcSapProvider());
        m_rlcAm->SetLteRlcSapUser(m_source->GetLteRlcSapUser());
        m_source->SetStatus(TxStatus<SourcePdcp>(4094));

        // SDU i has 100 + i payload bytes (PDUs of 102 to 105 bytes). When sent: PDU VT(A) =
        // [SDU 0], VT(A)+1 = [head of SDU 1] (a 54-byte opportunity leaves 50 data bytes),
        // VT(A)+2 = [tail of SDU 1 + SDU 2], VT(A)+3 = [SDU 3].
        std::vector<Bytes> sdus;
        const uint32_t opportunities[] = {1000, 54, 1000, 1000};
        for (uint32_t i = 0; i < 4; i++)
        {
            sdus.push_back(Payload(40 + i, 100 + i));
            TransmitSdu(m_source, PacketOf(sdus.back()), DRB_LCID);
            if (m_transmitted)
            {
                TxOpportunity(opportunities[i]);
            }
        }
        if (m_transmitted)
        {
            CheckRlcPdus();
        }
        else
        {
            NS_TEST_ASSERT_MSG_EQ(m_mac.m_pdus.size(), 0u, "precondition: nothing sent");
            NS_TEST_ASSERT_MSG_EQ(m_rlcAm->GetTxBufferSize(),
                                  102u + 103u + 104u + 105u,
                                  "precondition: the four PDCP PDUs wait in the RLC");
        }
        if (m_nackFirstTwoPdus)
        {
            ReceiveStatus(RlcSn(2), {RlcSn(0), RlcSn(1)});
            NS_TEST_ASSERT_MSG_GT(m_rlcAm->GetRetxBufferSize(),
                                  0u,
                                  "precondition: the NACKed PDUs wait for retransmission");
            NS_TEST_ASSERT_MSG_GT(m_rlcAm->GetTxedBufferSize(),
                                  0u,
                                  "precondition: the later PDUs wait for acknowledgement");
        }

        // The forwarding order: the SDU index of each forwarded packet (see the class comment).
        const std::vector<uint16_t> sourceSns = {4094, 4095, 0, 1};
        std::vector<std::size_t> order = {0, 1, 2, 3};
        if (m_transmitted)
        {
            order.push_back(3);
        }

        // Collect, with the bearer's header class, as the production callers do.
        std::vector<Ptr<Packet>> buffer;
        uint32_t bufferSize = 0;
        CollectRlcForwardingBuffer<PdcpHeader>(m_rlcAm, buffer, bufferSize);
        NS_TEST_EXPECT_MSG_EQ(m_rlcAm->GetTxBufferSize(), 0u, "the RLC queue is handed over");
        NS_TEST_ASSERT_MSG_EQ(buffer.size(), order.size(), "the forwarded packet count");
        if (buffer.size() != order.size())
        {
            return; // the runner continues after a failed assertion by default
        }
        bool anyPacketTag = false;
        for (std::size_t k = 0; k < order.size(); k++)
        {
            Bytes pdu = BytesOf(buffer[k]);
            Bytes wantPdu = BytesOf(HandBuiltDataPdu(sourceSns[order[k]], sdus[order[k]]));
            NS_TEST_EXPECT_MSG_EQ(OuterHeaderClass(buffer[k]),
                                  "ns3::NrPdcpHeader",
                                  "forwarded packet " << k << ": the header the source wrote");
            NS_TEST_EXPECT_MSG_EQ(Hex(pdu),
                                  Hex(wantPdu),
                                  "forwarded packet " << k << ": SN " << sourceSns[order[k]]);
            NS_TEST_EXPECT_MSG_EQ((pdu == wantPdu),
                                  true,
                                  "forwarded packet " << k << ": SDU " << order[k] << " whole");
            anyPacketTag = anyPacketTag || HasPacketTags(buffer[k]);
        }
        NS_TEST_ASSERT_MSG_EQ(anyPacketTag, true, "precondition: the RLC tags are still on");

        // Deliver, handing every packet to the target as the production callers do.
        const bool sduForwarding = m_target != ForwardingTarget::ENB_TARGET_RLC;
        CapturingRlcSapProvider peerSideRlc;
        Ptr<SourcePdcp> resender;
        if (m_target == ForwardingTarget::ENB_TARGET_PDCP)
        {
            m_targetPdcp = CreateObject<SourcePdcp>();
            m_targetPdcp->SetRnti(RNTI);
            m_targetPdcp->SetLcId(DRB_LCID);
            m_targetPdcp->SetLtePdcpSapUser(&m_sourceUser);
            m_targetPdcp->SetLteRlcSapProvider(&peerSideRlc);
            m_targetPdcp->SetStatus(TxStatus<SourcePdcp>(4095));
            resender = m_targetPdcp;
        }
        else if (m_target == ForwardingTarget::UE_SAME_PDCP)
        {
            m_source->SetLteRlcSapProvider(&peerSideRlc);
            resender = m_source;
        }
        std::vector<Ptr<Packet>> delivered;
        DeliverRlcForwardingBuffer<PdcpHeader>(buffer,
                                               bufferSize,
                                               sduForwarding,
                                               [&delivered, resender](Ptr<Packet> forwarded) {
                                                   delivered.push_back(forwarded->Copy());
                                                   if (resender)
                                                   {
                                                       TransmitSdu(resender, forwarded, DRB_LCID);
                                                   }
                                               });
        NS_TEST_EXPECT_MSG_EQ(buffer.empty(), true, "the forwarding buffer is emptied");
        NS_TEST_ASSERT_MSG_EQ(delivered.size(), order.size(), "every packet delivered");
        if (delivered.size() != order.size())
        {
            return; // the runner continues after a failed assertion by default
        }
        for (std::size_t k = 0; k < order.size(); k++)
        {
            const Bytes& sdu = sdus[order[k]];
            Bytes want = sduForwarding ? sdu : BytesOf(HandBuiltDataPdu(sourceSns[order[k]], sdu));
            const std::string wantClass = sduForwarding ? "<no header>" : "ns3::NrPdcpHeader";
            NS_TEST_EXPECT_MSG_EQ(OuterHeaderClass(delivered[k]),
                                  wantClass,
                                  "delivered packet " << k << ": the NR header "
                                                      << (sduForwarding ? "removed" : "kept"));
            NS_TEST_EXPECT_MSG_EQ(Hex(BytesOf(delivered[k])),
                                  Hex(want),
                                  "delivered packet " << k << ": first octets");
            NS_TEST_EXPECT_MSG_EQ((BytesOf(delivered[k]) == want),
                                  true,
                                  "delivered packet " << k << ": SDU " << order[k]
                                                      << (sduForwarding ? "" : " under its SN"));
            NS_TEST_EXPECT_MSG_EQ(HasPacketTags(delivered[k]),
                                  false,
                                  "delivered packet " << k << ": no packet tag");
        }

        // The SN change: the PDUs sent again carry new SNs, the original payload after them.
        std::vector<Ptr<Packet>> toPeer = delivered;
        if (sduForwarding)
        {
            const uint16_t firstNewSn = IS_ENB ? 4095 : 2;
            NS_TEST_ASSERT_MSG_EQ(peerSideRlc.m_pdus.size(), order.size(), "all sent again");
            if (peerSideRlc.m_pdus.size() != order.size())
            {
                return; // the runner continues after a failed assertion by default
            }
            for (std::size_t k = 0; k < order.size(); k++)
            {
                const uint16_t newSn = static_cast<uint16_t>((firstNewSn + k) % 4096);
                Bytes pdu = BytesOf(peerSideRlc.m_pdus[k]);
                NS_TEST_EXPECT_MSG_EQ(Hex(pdu),
                                      Hex(BytesOf(HandBuiltDataPdu(newSn, sdus[order[k]]))),
                                      "packet " << k << " sent again at SN " << newSn);
                NS_TEST_EXPECT_MSG_EQ(OuterHeaderClass(peerSideRlc.m_pdus[k]),
                                      "ns3::NrPdcpHeader",
                                      "packet " << k << " sent again with the NR header");
            }
            toPeer = peerSideRlc.m_pdus;
        }

        // The peer entity delivers the original SDUs, byte for byte and in the forwarded order.
        m_peer = CreateObject<PeerPdcp>();
        CapturingRlcSapProvider unusedRlc;
        Wire(m_peer, DRB_LCID, &unusedRlc, &m_peerUser);
        uint16_t lastSn = 0;
        for (Ptr<Packet> pdu : toPeer)
        {
            NrPdcpHeader header;
            pdu->PeekHeader(header);
            lastSn = header.GetSequenceNumber();
            m_peer->GetLteRlcSapUser()->ReceivePdcpPdu(pdu->Copy());
        }
        NS_TEST_ASSERT_MSG_EQ(m_peerUser.m_sdus.size(), order.size(), "all delivered by the peer");
        if (m_peerUser.m_sdus.size() != order.size())
        {
            return; // the runner continues after a failed assertion by default
        }
        for (std::size_t k = 0; k < order.size(); k++)
        {
            NS_TEST_EXPECT_MSG_EQ((m_peerUser.m_sdus[k] == sdus[order[k]]),
                                  true,
                                  "SDU " << k << " delivered by the peer equals SDU " << order[k]);
        }
        NS_TEST_EXPECT_MSG_EQ(m_peer->GetStatus().rxSn,
                              static_cast<uint16_t>((lastSn + 1) % NrPdcpHeader::SN_SPACE),
                              "the peer's RX SN follows the last SN");
    }

    void DoTeardown() override
    {
        if (m_rlcAm)
        {
            m_rlcAm->Dispose();
            m_rlcAm = nullptr;
        }
        for (Ptr<LtePdcp> pdcp : std::vector<Ptr<LtePdcp>>{m_source, m_targetPdcp, m_peer})
        {
            if (pdcp)
            {
                pdcp->Dispose();
            }
        }
        m_source = nullptr;
        m_targetPdcp = nullptr;
        m_peer = nullptr;
        Simulator::Destroy();
    }

    ForwardingTarget m_target;
    uint16_t m_vtA;
    bool m_transmitted;
    bool m_nackFirstTwoPdus;
    Ptr<LteRlcAm> m_rlcAm;
    Ptr<SourcePdcp> m_source;
    Ptr<SourcePdcp> m_targetPdcp;
    Ptr<PeerPdcp> m_peer;
    CapturingMacSapProvider m_mac;
    CapturingPdcpSapUser m_sourceUser;
    CapturingPdcpSapUser m_peerUser;
};

/**
 * \ingroup lte-test
 * \brief The forwarding of an RLC UM low-latency buffer whose head is the unsent tail of a cut
 *        SDU, by DeliverRlcForwardingBuffer as UeManager::ForwardRlcBuffers calls it.
 *
 * An McEnbPdcp sends SDU A (a 102-octet PDCP PDU) into an LteRlcUmLowLat entity. One TX
 * opportunity of 42 octets (2 RLC header octets, 40 data octets) cuts it: the RLC gives the
 * unsent 62-octet tail back to the head of its TX buffer with status LAST_SEGMENT. Every
 * payload octet of A has its top bit set, so the tail's first octet reads as D/C = 1. SDU B is
 * then queued whole (FULL_SDU).
 *  - ENB_TARGET_RLC, the secondary cell handover (whole PDCP PDUs to the target RLC): the tail
 *    is an RLC segment and is not forwarded; B's PDU is forwarded byte for byte under its SN.
 *  - ENB_TARGET_PDCP, the X2 forwarding of PDCP SDUs, which does not test the RLC status: the
 *    tail passes the D/C test and loses its first two octets, then B's SDU follows. The case
 *    pins that recorded defect ("RLC UM tail re-submission"), so a fix shows up here.
 */
class UmTailForwardingTestCase : public TestCase
{
  public:
    explicit UmTailForwardingTestCase(ForwardingTarget target)
        : TestCase(target == ForwardingTarget::ENB_TARGET_RLC
                       ? "eNB RLC PDU forwarding, RLC UM low latency, head = unsent tail"
                       : "eNB X2 forwarding of SDUs, RLC UM low latency, head = unsent tail"),
          m_target(target)
    {
        NS_ABORT_MSG_IF(target == ForwardingTarget::UE_SAME_PDCP, "an eNB forwarding only");
    }

  private:
    static constexpr uint16_t SN_A = 100;
    static constexpr uint32_t OPPORTUNITY = 42;
    static constexpr uint32_t HEAD_OCTETS = OPPORTUNITY - 2; // the RLC UM fixed header

    /// The RLC status of \p p, or 0 when it carries no LteRlcSduStatusTag.
    static uint32_t RlcStatus(Ptr<const Packet> p)
    {
        LteRlcSduStatusTag tag;
        return p->PeekPacketTag(tag) ? static_cast<uint32_t>(tag.GetStatus()) : 0;
    }

    void DoRun() override
    {
        EnableHeaderMetadata();
        m_rlc = CreateObject<LteRlcUmLowLat>();
        m_rlc->SetRnti(RNTI);
        m_rlc->SetLcId(DRB_LCID);
        m_rlc->SetLteMacSapProvider(&m_mac);
        m_source = CreateObject<McEnbPdcp>();
        m_source->SetRnti(RNTI);
        m_source->SetLcId(DRB_LCID);
        m_source->SetLtePdcpSapUser(&m_user);
        m_source->SetLteRlcSapProvider(m_rlc->GetLteRlcSapProvider());
        m_rlc->SetLteRlcSapUser(m_source->GetLteRlcSapUser());
        m_source->SetStatus(TxStatus<McEnbPdcp>(SN_A));

        Bytes sduA(100);
        for (uint32_t i = 0; i < sduA.size(); i++)
        {
            sduA[i] = static_cast<uint8_t>(0x80 | (i & 0x7F));
        }
        const Bytes sduB = Payload(7, 60);
        const Bytes pduA = BytesOf(HandBuiltDataPdu(SN_A, sduA));
        const Bytes pduB = BytesOf(HandBuiltDataPdu(SN_A + 1, sduB));
        const Bytes tail(pduA.begin() + HEAD_OCTETS, pduA.end());

        TransmitSdu(m_source, PacketOf(sduA), DRB_LCID);
        LteMacSapUser::TxOpportunityParameters params;
        params.bytes = OPPORTUNITY;
        params.layer = 0;
        params.harqId = 0;
        params.componentCarrierId = 0;
        params.rnti = RNTI;
        params.lcid = DRB_LCID;
        m_rlc->GetLteMacSapUser()->NotifyTxOpportunity(params);
        TransmitSdu(m_source, PacketOf(sduB), DRB_LCID);

        // Preconditions: one PDU sent; the TX buffer is [tail of A (LAST_SEGMENT), B (FULL)].
        NS_TEST_ASSERT_MSG_EQ(m_mac.m_pdus.size(), 1u, "precondition: the head of A went out");
        Bytes macPdu = BytesOf(m_mac.m_pdus[0]);
        NS_TEST_ASSERT_MSG_EQ(macPdu.size(), OPPORTUNITY, "precondition: the opportunity is full");
        NS_TEST_EXPECT_MSG_EQ((Bytes(macPdu.end() - HEAD_OCTETS, macPdu.end()) ==
                               Bytes(pduA.begin(), pduA.begin() + HEAD_OCTETS)),
                              true,
                              "precondition: the MAC PDU carries the head of A");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(tail.size(),
                                    3u,
                                    "precondition: the tail is long enough for the D/C test");
        std::vector<Ptr<Packet>> txBuffer = m_rlc->GetTxBuffer();
        NS_TEST_ASSERT_MSG_EQ(txBuffer.size(), 2u, "precondition: tail of A, then B, queued");
        if (txBuffer.size() != 2)
        {
            return; // the runner continues after a failed assertion by default
        }
        Bytes head = BytesOf(txBuffer[0]);
        NS_TEST_EXPECT_MSG_EQ(RlcStatus(txBuffer[0]),
                              static_cast<uint32_t>(LteRlcSduStatusTag::LAST_SEGMENT),
                              "precondition: the head of the TX buffer is the tail of A");
        NS_TEST_EXPECT_MSG_EQ(Hex(head), Hex(tail), "precondition: the tail's first octets");
        NS_TEST_EXPECT_MSG_EQ((head == tail), true, "precondition: the whole tail of A");
        NS_TEST_ASSERT_MSG_EQ((head.front() & 0x80),
                              0x80,
                              "precondition: the tail's first octet reads as D/C = 1");
        Bytes queuedB = BytesOf(txBuffer[1]);
        NS_TEST_EXPECT_MSG_EQ(RlcStatus(txBuffer[1]),
                              static_cast<uint32_t>(LteRlcSduStatusTag::FULL_SDU),
                              "precondition: B is queued whole");
        NS_TEST_EXPECT_MSG_EQ((queuedB == pduB), true, "precondition: B's PDU at SN_A + 1");

        // Collect and deliver with the bearer's header class, as UeManager::ForwardRlcBuffers.
        std::vector<Ptr<Packet>> buffer;
        uint32_t bufferSize = 0;
        CollectRlcForwardingBuffer<McEnbPdcp::DataPduHeader>(m_rlc, buffer, bufferSize);
        NS_TEST_ASSERT_MSG_EQ(buffer.size(), 2u, "precondition: both packets collected");
        const bool sduForwarding = m_target == ForwardingTarget::ENB_TARGET_PDCP;
        std::vector<Bytes> delivered;
        DeliverRlcForwardingBuffer<McEnbPdcp::DataPduHeader>(
            buffer,
            bufferSize,
            sduForwarding,
            [&delivered](Ptr<Packet> forwarded) { delivered.push_back(BytesOf(forwarded)); });
        NS_TEST_EXPECT_MSG_EQ(buffer.empty(), true, "the forwarding buffer is emptied");
        if (!sduForwarding)
        {
            // Only the whole-PDU path ends at 0: the SDU forwards subtract each packet after its
            // header is removed (a log-only count, outside this case).
            NS_TEST_EXPECT_MSG_EQ(bufferSize, 0u, "the byte count follows the emptied buffer");
        }

        std::vector<Bytes> want;
        if (sduForwarding)
        {
            want.emplace_back(tail.begin() + NR_PDCP_HEADER_OCTETS, tail.end());
            want.push_back(sduB);
        }
        else
        {
            want.push_back(pduB);
        }
        NS_TEST_ASSERT_MSG_EQ(delivered.size(),
                              want.size(),
                              (sduForwarding ? "the tail and B forwarded as SDUs"
                                             : "only B's whole PDU forwarded to the target RLC"));
        if (delivered.size() != want.size())
        {
            return; // the runner continues after a failed assertion by default
        }
        for (std::size_t k = 0; k < want.size(); k++)
        {
            NS_TEST_EXPECT_MSG_EQ(Hex(delivered[k]),
                                  Hex(want[k]),
                                  "delivered packet " << k << ": first octets");
            NS_TEST_EXPECT_MSG_EQ((delivered[k] == want[k]),
                                  true,
                                  "delivered packet " << k << ": byte for byte");
        }
    }

    void DoTeardown() override
    {
        if (m_rlc)
        {
            m_rlc->Dispose();
            m_rlc = nullptr;
        }
        DisposeAll(m_source);
        m_source = nullptr;
    }

    ForwardingTarget m_target;
    Ptr<LteRlcUmLowLat> m_rlc;
    Ptr<McEnbPdcp> m_source;
    CapturingMacSapProvider m_mac;
    CapturingPdcpSapUser m_user;
};

/**
 * \ingroup lte-test
 * \brief The mc-pdcp-nr-header test suite.
 */
class McPdcpNrHeaderTestSuite : public TestSuite
{
  public:
    McPdcpNrHeaderTestSuite()
        : TestSuite("mc-pdcp-nr-header", UNIT)
    {
        for (uint16_t sn : LAYOUT_SNS)
        {
            AddTestCase(new NrPdcpHeaderCodecTestCase(sn), TestCase::QUICK);
        }
        AddTestCase(new McPdcpHeaderClassTestCase(), TestCase::QUICK);
        for (uint16_t sn : LAYOUT_SNS)
        {
            AddTestCase(new McPdcpDataPduTxTestCase<McEnbPdcp>("DL McEnbPdcp", sn),
                        TestCase::QUICK);
            AddTestCase(new McPdcpDataPduTxTestCase<McUePdcp>("UL McUePdcp", sn), TestCase::QUICK);
            AddTestCase(new McPdcpDataPduRxTestCase<McUePdcp>("DL McUePdcp", sn), TestCase::QUICK);
            AddTestCase(new McPdcpDataPduRxTestCase<McEnbPdcp>("UL McEnbPdcp", sn),
                        TestCase::QUICK);
            AddTestCase(new SrbLtePdcpUnchangedTestCase(sn), TestCase::QUICK);
        }

        // Status report, interspersed ROHC feedback (neither configured) and a reserved type.
        for (uint8_t pduType : {0, 1, 5})
        {
            AddTestCase(new McPdcpControlPduRxTestCase<McUePdcp>("DL McUePdcp", pduType),
                        TestCase::QUICK);
            AddTestCase(new McPdcpControlPduRxTestCase<McEnbPdcp>("UL McEnbPdcp", pduType),
                        TestCase::QUICK);
        }

        using EnbForwarding = McPdcpForwardingTestCase<McEnbPdcp, McUePdcp>;
        using UeForwarding = McPdcpForwardingTestCase<McUePdcp, McEnbPdcp>;
        const ForwardingTarget toPdcp = ForwardingTarget::ENB_TARGET_PDCP;
        const ForwardingTarget toRlc = ForwardingTarget::ENB_TARGET_RLC;
        const ForwardingTarget samePdcp = ForwardingTarget::UE_SAME_PDCP;
        AddTestCase(new EnbForwarding("eNB X2 forwarding of SDUs, VT(A) 1022, SNs 4094 to 1 sent",
                                      toPdcp,
                                      1022,
                                      true,
                                      false),
                    TestCase::QUICK);
        AddTestCase(
            new EnbForwarding(
                "eNB X2 forwarding of SDUs, VT(A) 1022, SNs 4094 to 1 sent, NACK 1022, 1023",
                toPdcp,
                1022,
                true,
                true),
            TestCase::QUICK);
        AddTestCase(new EnbForwarding("eNB X2 forwarding of SDUs, VT(A) 5, SNs 4094 to 1 sent",
                                      toPdcp,
                                      5,
                                      true,
                                      false),
                    TestCase::QUICK);
        AddTestCase(new EnbForwarding("eNB X2 forwarding of SDUs, VT(A) 1022, SNs 4094 to 1 queued",
                                      toPdcp,
                                      1022,
                                      false,
                                      false),
                    TestCase::QUICK);
        AddTestCase(new EnbForwarding("eNB RLC PDU forwarding, VT(A) 1022, SNs 4094 to 1 sent",
                                      toRlc,
                                      1022,
                                      true,
                                      false),
                    TestCase::QUICK);
        AddTestCase(new EnbForwarding("eNB RLC PDU forwarding, VT(A) 1022, SNs 4094 to 1 queued",
                                      toRlc,
                                      1022,
                                      false,
                                      false),
                    TestCase::QUICK);
        AddTestCase(new UeForwarding("UE RLC switch, VT(A) 1022, SNs 4094 to 1 sent",
                                     samePdcp,
                                     1022,
                                     true,
                                     false),
                    TestCase::QUICK);
        AddTestCase(
            new UeForwarding("UE RLC switch, VT(A) 1022, SNs 4094 to 1 sent, NACK 1022, 1023",
                             samePdcp,
                             1022,
                             true,
                             true),
            TestCase::QUICK);
        AddTestCase(new UeForwarding("UE RLC switch, VT(A) 5, SNs 4094 to 1 sent",
                                     samePdcp,
                                     5,
                                     true,
                                     false),
                    TestCase::QUICK);
        AddTestCase(new UeForwarding("UE RLC switch, VT(A) 1022, SNs 4094 to 1 queued",
                                     samePdcp,
                                     1022,
                                     false,
                                     false),
                    TestCase::QUICK);
        AddTestCase(new UmTailForwardingTestCase(ForwardingTarget::ENB_TARGET_RLC),
                    TestCase::QUICK);
        AddTestCase(new UmTailForwardingTestCase(ForwardingTarget::ENB_TARGET_PDCP),
                    TestCase::QUICK);
    }
};

/// The static instance that registers the suite.
static McPdcpNrHeaderTestSuite g_mcPdcpNrHeaderTestSuite;
