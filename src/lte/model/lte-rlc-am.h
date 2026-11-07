/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 * Copyright (c) 2016, 2018, University of Padova, Dep. of Information Engineering, SIGNET lab
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
 * Author: Manuel Requena <manuel.requena@cttc.es>
 *
 * Modified by Michele Polese <michele.polese@gmail.com> to add DC functionalities
 *
 * Modified by: Tommaso Zugno <tommasozugno@gmail.com>
 *              Integration of Carrier Aggregation for the mmWave module
 */

#ifndef LTE_RLC_AM_H
#define LTE_RLC_AM_H

#include "ns3/codel-queue-disc.h"
#include <ns3/epc-x2-sap.h>
#include <ns3/event-id.h>
#include <ns3/lte-rlc-sequence-number.h>
#include <ns3/lte-rlc.h>
#include <ns3/nr-pdcp-header.h>

#include <fstream>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace ns3
{

/**
 * LTE RLC Acknowledged Mode (AM), see 3GPP TS 36.322
 */
class LteRlcAm : public LteRlc
{
  public:
    LteRlcAm();
    virtual ~LteRlcAm();
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId(void);
    virtual void DoDispose();

    struct RetxPdu
    {
        Ptr<Packet> m_pdu;
        uint16_t m_retxCount;
    };

    /**
     * RLC SAP
     *
     * \param p packet
     */
    virtual void DoTransmitPdcpPdu(Ptr<Packet> p);

    /**
     * RLC EPC X2 SAP
     */
    virtual void DoSendMcPdcpSdu(EpcX2Sap::UeDataParams params);

    // LL HO
    std::vector<Ptr<Packet>> GetTxBuffer();
    uint32_t GetTxBufferSize() const;

    uint32_t GetTxedBufferSize() const;
    uint32_t GetRetxBufferSize() const;

    /**
     * The AMD PDUs not yet positively acknowledged, in transmission order, for reassembly into
     * the SDUs forwarded at a lossless handover (RlcPdusToRlcSdus).
     *
     * The walk covers VT(A) <= SN < VT(S) with VT(A) as the modulus base, so across the 10-bit
     * SN wrap PDU 1023 comes before PDU 0 (ETSI TS 136 322 V16.0.0 clause 7.1: VT(A) is the
     * modulus base of the transmitting side, VT(S) the SN of the next new AMD PDU; clause
     * 5.1.3.1.1: only SNs inside the transmitting window VT(A) <= SN < VT(MS) are delivered,
     * and clause 7.2: AM_Window_Size = 512 is less than half the SN space, so the walk is
     * unambiguous). At each SN the PDU is taken from the transmitted buffer, else from the
     * retransmission buffer; an SN is never in both.
     */
    std::vector<RetxPdu> GetUnackedPdusInTransmissionOrder() const;

    /**
     * The SDUs reassembled from the unacknowledged PDUs for forwarding at a lossless handover,
     * each with its PDCP SN, in the order its SDU was first reassembled. The RLC numbers PDUs
     * in the order its SDUs arrive from the PDCP, which assigns SNs in COUNT order, so across
     * the PDCP SN wrap the SDUs with SNs 4094, 4095, 0, 1 come in that order. Each SN appears
     * once, as its latest reassembled copy.
     */
    std::vector<std::pair<uint16_t, Ptr<Packet>>> GetTransmittingRlcSduBuffer();
    uint32_t GetTransmittingRlcSduBufferSize() const;

    Ptr<Packet> GetSegmentedRlcsdu();

    /**
     * Reassembles RLC PDUs into the RLC SDUs forwarded at a lossless handover, puts them into
     * m_transmittingRlcSdus and keys each by its PDCP SN (GetTransmittingRlcSduBuffer).
     *
     * \tparam PdcpDataPduHeader the class of the header on the bearer's PDCP Data PDUs, from
     *         which the PDCP SN is read. The RLC entity does not know its bearer; the caller,
     *         which holds the bearer's PDCP entity, names it (McEnbPdcp::DataPduHeader or
     *         McUePdcp::DataPduHeader). Only NrPdcpHeader compiles (IfNrPdcpHeader): every
     *         bearer whose RLC buffers are forwarded is a dual-connectivity bearer.
     * \param Pdus the RLC PDUs, in transmission order
     */
    template <class PdcpDataPduHeader>
    IfNrPdcpHeader<PdcpDataPduHeader> RlcPdusToRlcSdus(std::vector<RetxPdu> Pdus);

    std::vector<Ptr<Packet>> GetTxedRlcSduBuffer()
    {
        return m_txedRlcSduBuffer;
    }

  private:
    // whether the last SDU in the txonBuffer is a complete SDU.
    bool is_fragmented;

    //
    std::vector<Ptr<Packet>> m_txedRlcSduBuffer;
    uint32_t m_txedRlcSduBufferSize;

  public:
    /**
     * MAC SAP
     *
     * \param txOpParams the LteMacSapUser::TxOpportunityParameters
     */
    virtual void DoNotifyTxOpportunity(LteMacSapUser::TxOpportunityParameters txOpParams);
    /**
     * Notify HARQ delivery failure
     */
    virtual void DoNotifyHarqDeliveryFailure();
    virtual void DoNotifyDlHarqDeliveryFailure(uint8_t harqId);
    virtual void DoNotifyUlHarqDeliveryFailure(uint8_t harqId);
    virtual void DoReceivePdu(LteMacSapUser::ReceivePduParameters rxPduParams);

  private:
    /**
     * This method will schedule a timeout at WaitReplyTimeout interval
     * in the future, unless a timer is already running for the cache,
     * in which case this method does nothing.
     */
    void ExpireReorderingTimer(void);
    /// Expire poll retransmitter
    void ExpirePollRetransmitTimer(void);
    /// Expire RBS timer
    void ExpireRbsTimer(void);

    /**
     * method called when the T_status_prohibit timer expires
     *
     */
    void ExpireStatusProhibitTimer(void);

    /**
     * method called when the T_status_prohibit timer expires
     *
     * \param seqNumber SequenceNumber10
     * \returns true is inside receivign window
     */
    bool IsInsideReceivingWindow(SequenceNumber10 seqNumber);

    // LL HO
    bool IsInsideTransmittingWindow();
    // Create RlcSduBuffer <seqNumber, RlcSDU> based on m_transmittingRlcSdus, reading each
    // PDCP SN with the bearer's header class (see RlcPdusToRlcSdus).
    template <class PdcpDataPduHeader>
    void CreateRlcSduBuffer();

    //
    //   void ReassembleOutsideWindow (void);
    //   void ReassembleSnLessThan (uint16_t seqNumber);
    //

    /**
     * Reassemble and deliver
     *
     * \param packet the packet
     */
    void ReassembleAndDeliver(Ptr<Packet> packet);
    void TriggerReceivePdcpPdu(Ptr<Packet> p);

    void Reassemble(Ptr<Packet> Packet);

    /**
     * Report buffer status
     */
    void DoReportBufferStatus();

    std::string GetBufferSizeFilename();
    void SetBufferSizeFilename(std::string filename);
    void BufferSizeTrace();

  private:
    std::vector<Ptr<Packet>> m_txonBuffer; ///< Transmission buffer

    struct RetxSegPdu
    {
        Ptr<Packet> m_pdu;    ///< PDU
        uint16_t m_retxCount; ///< retransmit count
        bool m_lastSegSent;   // all segments sent, waiting for ACK
    };

    // LL HO: store a complete version of the incomplete RLC SDU at the
    // edge of the m_txonBuffer during the segmentation process.
    // This SDU will be forwarded to target eNB in lossless HO
    // to assure no packet is lost.
    Ptr<Packet> m_segmented_rlcsdu;

    std::vector<RetxPdu> m_txedBuffer;       ///< Buffer for transmitted and retransmitted PDUs
                                             ///< that have not been acked but are not considered
                                             ///< for retransmission
    std::vector<RetxPdu> m_retxBuffer;       ///< Buffer for PDUs considered for retransmission
    std::vector<RetxSegPdu> m_retxSegBuffer; // buffer for AM PDU segments

    Ptr<CoDelQueueDisc> m_txonQueue;

    ///< LL HO: stores RLC SDUs that is not acked
    ///< and forwarded to target eNB during lossless handover.
    std::vector<Ptr<Packet>> m_transmittingRlcSdus;
    uint32_t m_transmittingRlcSduBufferSize;
    // Keyed by PDCP SN, so an SDU reassembled again replaces its earlier copy. The key order is
    // the raw SN order, not the COUNT order across the SN wrap: read it only through
    // GetTransmittingRlcSduBuffer, which follows m_transmittingRlcSduOrder.
    std::map<uint16_t, Ptr<Packet>> m_transmittingRlcSduBuffer;
    // The PDCP SNs of m_transmittingRlcSduBuffer in the order each was first reassembled. It
    // has the map's lifetime: an SN is appended only when it enters the map, and both are
    // cleared together.
    std::vector<uint16_t> m_transmittingRlcSduOrder;

    uint32_t m_txonBufferSize; ///< transmit on buffer size
    uint32_t m_retxBufferSize; ///< transmit on buffer size
    uint32_t m_txedBufferSize; ///< transmit ed buffer size

    bool m_statusPduRequested;      ///< status PDU requested
    uint32_t m_statusPduBufferSize; ///< status PDU buffer size

    /// PduBuffer structure
    struct PduBuffer
    {
        SequenceNumber10 m_seqNumber;          ///< sequence number
        std::list<Ptr<Packet>> m_byteSegments; ///< byte segments

        bool m_pduComplete; ///< PDU complete?
        uint16_t m_totalSize;
        uint16_t m_currSize;
    };

    std::map<uint16_t, PduBuffer> m_rxonBuffer; ///< Reception buffer

    Ptr<Packet> m_controlPduBuffer; ///< Control PDU buffer (just one PDU)

    // SDU reassembly
    //   std::vector < Ptr<Packet> > m_reasBuffer;     // Reassembling buffer
    //
    std::list<Ptr<Packet>> m_sdusBuffer; ///< List of SDUs in a packet (PDU)
    std::list<Ptr<Packet>> m_sdusAssembleBuffer;

    /**
     * State variables. See section 7.1 in TS 36.322
     */
    // Transmitting side
    SequenceNumber10 m_vtA;    ///< VT(A)
    SequenceNumber10 m_vtMs;   ///< VT(MS)
    SequenceNumber10 m_vtS;    ///< VT(S)
    SequenceNumber10 m_pollSn; ///< POLL_SN

    // Receiving side
    SequenceNumber10 m_vrR;  ///< VR(R)
    SequenceNumber10 m_vrMr; ///< VR(MR)
    SequenceNumber10 m_vrX;  ///< VR(X)
    SequenceNumber10 m_vrMs; ///< VR(MS)
    SequenceNumber10 m_vrH;  ///< VR(H)

    /**
     * Counters. See section 7.1 in TS 36.322
     */
    uint32_t m_pduWithoutPoll;  ///< PDU without poll
    uint32_t m_byteWithoutPoll; ///< byte without poll

    /**
     * Constants. See section 7.2 in TS 36.322
     */
    uint16_t m_windowSize;

    /**
     * Timers. See section 7.3 in TS 36.322
     */
    EventId m_pollRetransmitTimer;   ///< poll retransmit timer
    Time m_pollRetransmitTimerValue; ///< poll retransmit time value
    EventId m_reorderingTimer;       ///< reordering timer
    Time m_reorderingTimerValue;     ///< reordering timer value
    EventId m_statusProhibitTimer;   ///< status prohibit timer
    Time m_statusProhibitTimerValue; ///< status prohibit timer value
    EventId m_rbsTimer;              ///< RBS timer
    Time m_rbsTimerValue;            ///< RBS timer value

    /**
     * Configurable parameters. See section 7.4 in TS 36.322
     */
    uint16_t m_maxRetxThreshold; ///< \todo How these parameters are configured???
    uint16_t m_pollPdu;          ///< poll PDU
    uint16_t m_pollByte;         ///< poll byte

    bool m_txOpportunityForRetxAlwaysBigEnough; ///< transmit opportunity for retransmit?
    bool m_pollRetransmitTimerJustExpired;      ///< poll retransmit timer just expired?

    /**
     * SDU Reassembling state
     */
    typedef enum
    {
        NONE = 0,
        WAITING_S0_FULL = 1,
        WAITING_SI_SF = 2
    } ReassemblingState_t;

    ReassemblingState_t m_reassemblingState; ///< reassembling state
    ReassemblingState_t
        m_assemblingState; // state of the RlcPduToRlcSdu assembling used for handover.
    Ptr<Packet> m_keepS0;  ///< keep S0
    Ptr<Packet> m_keepS0Reassemble;

    /**
     * Expected Sequence Number
     */
    SequenceNumber10 m_expectedSeqNumber;

    SequenceNumber10 m_reassembleExpectedSeqNumber;

    std::map<uint8_t, uint16_t> m_harqIdToSnMap;

    uint32_t m_maxTxBufferSize;

    std::string m_bufferSizeFilename;
    std::ofstream m_bufferSizeFile;
    EventId m_traceBufferSizeEvent;

    bool m_enableAqm;
};

} // namespace ns3

#endif // LTE_RLC_AM_H
