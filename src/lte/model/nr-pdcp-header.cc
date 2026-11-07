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

#include "nr-pdcp-header.h"

#include "ns3/abort.h"
#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrPdcpHeader");

NS_OBJECT_ENSURE_REGISTERED(NrPdcpHeader);

NrPdcpHeader::NrPdcpHeader()
    : m_dcBit(DATA_PDU),
      m_sequenceNumber(0)
{
}

void
NrPdcpHeader::SetDcBit(uint8_t dcBit)
{
    NS_ABORT_MSG_IF(dcBit > DATA_PDU, "the D/C field is one bit, got " << +dcBit);
    m_dcBit = dcBit;
}

void
NrPdcpHeader::SetSequenceNumber(uint16_t sequenceNumber)
{
    NS_ABORT_MSG_IF(sequenceNumber >= SN_SPACE,
                    "a 12-bit PDCP SN is below " << SN_SPACE << ", got " << sequenceNumber);
    m_sequenceNumber = sequenceNumber;
}

uint8_t
NrPdcpHeader::GetDcBit() const
{
    return m_dcBit;
}

uint16_t
NrPdcpHeader::GetSequenceNumber() const
{
    return m_sequenceNumber;
}

TypeId
NrPdcpHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrPdcpHeader")
                            .SetParent<Header>()
                            .SetGroupName("Lte")
                            .AddConstructor<NrPdcpHeader>();
    return tid;
}

TypeId
NrPdcpHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

void
NrPdcpHeader::Print(std::ostream& os) const
{
    os << "D/C=" << +m_dcBit << " SN=" << m_sequenceNumber;
}

uint32_t
NrPdcpHeader::GetSerializedSize() const
{
    return 2;
}

void
NrPdcpHeader::Serialize(Buffer::Iterator start) const
{
    // Octet 1: D/C in bit 8, the three R bits as 0 (clause 6.3.6), SN bits 11..8; octet 2: SN
    // bits 7..0 (clause 6.2.2.2, Figure 6.2.2.2-1).
    start.WriteU8(static_cast<uint8_t>((m_dcBit << 7) | (m_sequenceNumber >> 8)));
    start.WriteU8(static_cast<uint8_t>(m_sequenceNumber & 0xFF));
}

uint32_t
NrPdcpHeader::Deserialize(Buffer::Iterator start)
{
    uint8_t octet1 = start.ReadU8();
    uint8_t octet2 = start.ReadU8();
    m_dcBit = (octet1 >> 7) & 0x01;
    // The R bits (0x70) are ignored by the receiver (clause 6.3.6).
    m_sequenceNumber = static_cast<uint16_t>(((octet1 & 0x0F) << 8) | octet2);
    return GetSerializedSize();
}

} // namespace ns3
