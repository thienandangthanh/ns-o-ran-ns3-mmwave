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

#include "lte-rlc-forwarding-buffer.h"

#include "lte-pdcp-sn-wrap.h"
#include "lte-rlc-am.h"
#include "lte-rlc-um-lowlat.h"
#include "lte-rlc-um.h"
#include "lte-rlc.h"
#include "nr-pdcp-header.h"

#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LteRlcForwardingBuffer");

// This code from the LL HO implementation was shared by the eNB and the UE RRC; it lives here
// so that both run the same steps.
template <class PdcpDataPduHeader>
IfNrPdcpHeader<PdcpDataPduHeader>
CollectRlcForwardingBuffer(Ptr<LteRlc> rlc, std::vector<Ptr<Packet>>& buffer, uint32_t& bufferSize)
{
    // RlcBuffers forwarding only for RlcAm bearers.
    if (rlc->GetObject<LteRlcAm>())
    {
        // Copy lte-rlc-am.m_txOnBuffer to the forwarding buffer.
        Ptr<LteRlcAm> rlcAm = rlc->GetObject<LteRlcAm>();
        uint32_t txonBufferSize = rlcAm->GetTxBufferSize();
        std::vector<Ptr<Packet>> txonBuffer = rlcAm->GetTxBuffer();
        uint32_t txedBufferSize = rlcAm->GetTxedBufferSize();
        uint32_t retxBufferSize = rlcAm->GetRetxBufferSize();

        // Translate Pdus in Rlc txed/retx buffer into RLC Sdus
        // and put these Sdus into rlcAm->m_transmittingRlcSdus.
        NS_LOG_INFO("retxBuffer size = " << retxBufferSize);
        NS_LOG_INFO("txedBuffer size = " << txedBufferSize);
        // The unacknowledged PDUs in transmission order (from VT(A), across the RLC SN wrap),
        // so an SDU segmented over RLC SNs 1023 and 0 is reassembled whole.
        if (retxBufferSize + txedBufferSize > 0)
        {
            rlcAm->RlcPdusToRlcSdus<PdcpDataPduHeader>(rlcAm->GetUnackedPdusInTransmissionOrder());
        }

        // Construct the forwarding buffer
        // Forwarding buffer = retxBuffer + txedBuffer + txonBuffer.
        PdcpDataPduHeader pdcpHeader;
        uint32_t pos = 0;
        for (std::vector<Ptr<Packet>>::iterator it = txonBuffer.begin(); it != txonBuffer.end();
             ++it)
        {
            pos++;
            if ((*it)->GetSize() > 3)
            {
                (*it)->PeekHeader(pdcpHeader);
                NS_LOG_DEBUG("txonBuffer SEQ = " << pdcpHeader.GetSequenceNumber()
                                                 << " Size = " << (*it)->GetSize());
            }
            else
            {
                NS_LOG_UNCOND("Fragment too small in txonBuffer, pos " << pos);
            }
        }

        // this cycle adds the SDUs given by the merge of txed and retxed buffers
        if (rlcAm->GetTransmittingRlcSduBufferSize() > 0)
        { // something inside the RLC AM's transmitting buffer
            NS_LOG_DEBUG("ADDING TRANSMITTING SDUS OF RLC AM TO FORWARDING BUFFER... Size = "
                         << rlcAm->GetTransmittingRlcSduBufferSize());
            // copy the RlcSdu buffer to forwardingBuffer, in transmission order (COUNT order),
            // so SDUs from before the PDCP SN wrap come first.
            std::vector<std::pair<uint16_t, Ptr<Packet>>> rlcAmTransmittingBuffer =
                rlcAm->GetTransmittingRlcSduBuffer();
            NS_LOG_DEBUG(" *** SIZE = " << rlcAmTransmittingBuffer.size());
            for (std::vector<std::pair<uint16_t, Ptr<Packet>>>::iterator it =
                     rlcAmTransmittingBuffer.begin();
                 it != rlcAmTransmittingBuffer.end();
                 ++it)
            {
                if (it->second)
                {
                    NS_LOG_DEBUG("add to forwarding buffer SEQ = " << it->first << " Ptr<Packet> = "
                                                                   << it->second);
                    buffer.push_back(it->second);
                }
            }
            NS_LOG_DEBUG("ADDING TXONBUFFER OF RLC AM, Size = " << txonBufferSize);

            Ptr<Packet> segmentedRlcsdu = rlcAm->GetSegmentedRlcsdu();
            if (segmentedRlcsdu != nullptr)
            {
                segmentedRlcsdu->PeekHeader(pdcpHeader);
                NS_LOG_DEBUG("SegmentedRlcSdu = " << segmentedRlcsdu->GetSize()
                                                  << " SEQ = " << pdcpHeader.GetSequenceNumber());
                // insert the complete version of the fragmented SDU to the front of txonBuffer.
                txonBuffer.insert(txonBuffer.begin(), segmentedRlcsdu);
            }
            buffer.insert(buffer.end(), txonBuffer.begin(), txonBuffer.end());
            bufferSize += rlcAm->GetTransmittingRlcSduBufferSize() + txonBufferSize;

            // Get the rlcAm
            std::vector<Ptr<Packet>> rlcAmTxedSduBuffer = rlcAm->GetTxedRlcSduBuffer();
            PdcpDataPduHeader pdcpHeader_1;
            buffer.at(0)->PeekHeader(pdcpHeader_1);
            uint16_t i = 0;
            for (std::vector<Ptr<Packet>>::iterator it = rlcAmTxedSduBuffer.begin();
                 it != rlcAmTxedSduBuffer.end();
                 ++it)
            {
                if ((*it) != nullptr)
                {
                    (*it)->PeekHeader(pdcpHeader);
                    NS_LOG_DEBUG("rlcAmTxedSduBuffer SEQ = " << pdcpHeader.GetSequenceNumber()
                                                             << " Size = " << (*it)->GetSize());

                    // add the previous SDU of the forwarding buffer to the forwarding buffer:
                    // the head SN and the two SNs before it, modulo the PDCP SN space.
                    if (IsPdcpSnInForwardingLookback(pdcpHeader.GetSequenceNumber(),
                                                     pdcpHeader_1.GetSequenceNumber()))
                    {
                        NS_LOG_DEBUG("Added previous SDU to forwarding buffer SEQ = "
                                     << pdcpHeader.GetSequenceNumber()
                                     << " Size = " << (*it)->GetSize());
                        buffer.insert(buffer.begin() + i, (*it)->Copy());
                        ++i;
                    }
                }
            }
        }
        else
        { // TransmittingBuffer is empty. Only copy TxonBuffer.
            NS_LOG_DEBUG("ADDING TXONBUFFER OF RLC AM, Size = " << txonBufferSize);
            buffer = txonBuffer;
            bufferSize += txonBufferSize;
        }
    }
    // For RlcUM, no forwarding available as the simulator itself (seamless HO).
    // However, as the LTE-UMTS book, PDCP txbuffer should be forwarded for seamless
    // HO. Enable this code for txbuffer forwarding in seamless HO (which is believe to
    // be correct).
    else if (rlc->GetObject<LteRlcUm>())
    {
        // Copy lte-rlc-um.m_txOnBuffer to the forwarding buffer.
        NS_LOG_DEBUG("Copying txonBuffer from RLC UM");
        buffer = rlc->GetObject<LteRlcUm>()->GetTxBuffer();
        bufferSize = rlc->GetObject<LteRlcUm>()->GetTxBufferSize();
    }
    else if (rlc->GetObject<LteRlcUmLowLat>())
    {
        // Copy lte-rlc-um-low-lat.m_txOnBuffer to the forwarding buffer.
        NS_LOG_DEBUG("Copying txonBuffer from RLC UM low latency");
        buffer = rlc->GetObject<LteRlcUmLowLat>()->GetTxBuffer();
        bufferSize = rlc->GetObject<LteRlcUmLowLat>()->GetTxBufferSize();
    }
    // LteRlcAm m_txBuffer stores PDCP "PDU".
    NS_LOG_DEBUG("forwarding buffer size = " << bufferSize);
}

template <class PdcpDataPduHeader>
IfNrPdcpHeader<PdcpDataPduHeader>
DeliverRlcForwardingBuffer(std::vector<Ptr<Packet>>& buffer,
                           uint32_t& bufferSize,
                           bool removePdcpHeader,
                           const std::function<void(Ptr<Packet>)>& deliver)
{
    while (!buffer.empty())
    {
        Ptr<Packet> rlcSdu = buffer.at(0);
        PdcpDataPduHeader pdcpHeader;
        NS_LOG_DEBUG("RlcSdu size = " << rlcSdu->GetSize());

        // only forward data PDCP PDUs (1-DATA_PDU,0-CTR_PDU)
        if (rlcSdu->GetSize() >= 3)
        {
            rlcSdu->PeekHeader(pdcpHeader);
            if (pdcpHeader.GetDcBit() == 1)
            { // ignore control SDU.
                NS_LOG_LOGIC("SEQ = " << pdcpHeader.GetSequenceNumber());
                rlcSdu->RemoveAllPacketTags(); // this does not remove byte tags
                NS_LOG_LOGIC("removed tags, size = " << rlcSdu->GetSize());
                if (removePdcpHeader)
                {
                    rlcSdu->RemoveHeader(pdcpHeader);
                    NS_LOG_LOGIC("removed pdcp header, size = " << rlcSdu->GetSize());
                }
                deliver(rlcSdu);
            }
        }
        else
        {
            NS_LOG_UNCOND("Too small, not forwarded");
        }
        bufferSize -= buffer.front()->GetSize();
        buffer.erase(buffer.begin());
        NS_LOG_LOGIC("After forwarding: buffer size = " << bufferSize);
    }
}

template void CollectRlcForwardingBuffer<NrPdcpHeader>(Ptr<LteRlc> rlc,
                                                       std::vector<Ptr<Packet>>& buffer,
                                                       uint32_t& bufferSize);

template void DeliverRlcForwardingBuffer<NrPdcpHeader>(
    std::vector<Ptr<Packet>>& buffer,
    uint32_t& bufferSize,
    bool removePdcpHeader,
    const std::function<void(Ptr<Packet>)>& deliver);

} // namespace ns3
