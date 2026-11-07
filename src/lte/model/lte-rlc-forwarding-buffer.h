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

#ifndef LTE_RLC_FORWARDING_BUFFER_H
#define LTE_RLC_FORWARDING_BUFFER_H

#include "ns3/nr-pdcp-header.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"

#include <cstdint>
#include <functional>
#include <vector>

namespace ns3
{

class LteRlc;

/**
 * \ingroup lte
 * \brief Appends to a forwarding buffer the PDCP PDUs an RLC entity has not delivered.
 *
 * This is the per-bearer work shared by the eNB forwarding (UeManager::ForwardRlcBuffers: X2
 * forwarding at a lossless handover, the LTE to mmWave switch) and the UE copy
 * (LteUeRrc::CopyRlcBuffers: the switch between the LTE and the mmWave RLC).
 *
 * RLC AM: when PDUs are unacknowledged, they are reassembled in transmission order into SDUs
 * keyed by PDCP SN (LteRlcAm::RlcPdusToRlcSdus). If the RLC then holds such SDUs, the buffer
 * gets them in that order, then the RLC's copy of its last started SDU, then the transmission
 * buffer; and, at its front, every already transmitted SDU in the forwarding lookback of the
 * buffer's head SN (IsPdcpSnInForwardingLookback). Otherwise the buffer is replaced by the
 * transmission buffer. RLC UM and UM low latency: the buffer is replaced by the transmission
 * buffer. The RLC transmission buffer is emptied in all three cases.
 *
 * \tparam PdcpDataPduHeader the Data PDU header class of the bearer's PDCP entity, from which
 *         the PDCP SNs are read. The caller, which holds that entity, names it
 *         (McEnbPdcp::DataPduHeader or McUePdcp::DataPduHeader). Only NrPdcpHeader compiles
 *         (IfNrPdcpHeader): every bearer whose RLC buffers are forwarded is a
 *         dual-connectivity bearer.
 * \param rlc the RLC entity of the bearer
 * \param buffer the forwarding buffer, in forwarding order
 * \param bufferSize the byte count kept with \p buffer, for logging
 */
template <class PdcpDataPduHeader>
IfNrPdcpHeader<PdcpDataPduHeader> CollectRlcForwardingBuffer(Ptr<LteRlc> rlc,
                                                             std::vector<Ptr<Packet>>& buffer,
                                                             uint32_t& bufferSize);

/**
 * \ingroup lte
 * \brief Empties a forwarding buffer, handing each PDCP Data PDU in it to \p deliver, in order.
 *
 * A packet shorter than 3 bytes, or whose D/C bit marks a Control PDU, is dropped. A Data PDU
 * loses its packet tags (byte tags stay) and, when \p removePdcpHeader, its PDCP header, so
 * \p deliver gets the PDCP SDU; otherwise \p deliver gets the whole PDCP PDU.
 *
 * When \p deliver gets whole PDCP PDUs (the secondary cell handover, which hands them to the
 * target RLC), an RLC segment is dropped before the D/C test: a packet whose LteRlcSduStatusTag
 * is not FULL_SDU, such as the unsent tail of an SDU that an RLC UM entity cut, whose first
 * octet is payload. A packet without that tag (an RLC AM SDU reassembled from unacknowledged
 * PDUs) is whole and keeps the D/C test. The PDCP SDU forwards do not test the tag: a tail
 * whose first octet reads as D/C = 1 still loses its first two octets there (the recorded
 * follow-up "RLC UM tail re-submission").
 *
 * \tparam PdcpDataPduHeader as for CollectRlcForwardingBuffer
 * \param buffer the forwarding buffer; empty on return
 * \param bufferSize the byte count kept with \p buffer, for logging
 * \param removePdcpHeader whether \p deliver gets the PDCP SDU rather than the PDCP PDU
 * \param deliver the sink of the forwarded packets
 */
template <class PdcpDataPduHeader>
IfNrPdcpHeader<PdcpDataPduHeader> DeliverRlcForwardingBuffer(
    std::vector<Ptr<Packet>>& buffer,
    uint32_t& bufferSize,
    bool removePdcpHeader,
    const std::function<void(Ptr<Packet>)>& deliver);

} // namespace ns3

#endif /* LTE_RLC_FORWARDING_BUFFER_H */
