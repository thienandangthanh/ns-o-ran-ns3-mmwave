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

#ifndef NR_PDCP_HEADER_H
#define NR_PDCP_HEADER_H

#include "ns3/header.h"

#include <cstdint>
#include <ostream>
#include <type_traits>

namespace ns3
{

/**
 * \ingroup lte
 * \brief The header of an NR PDCP Data PDU of a DRB with a 12-bit PDCP SN.
 *
 * 3GPP TS 38.323 V19.1.0 clause 6.2.2.2 (Figure 6.2.2.2-1), two octets:
 *  - octet 1: D/C (1 bit), R, R, R (3 bits), PDCP SN bits 11..8;
 *  - octet 2: PDCP SN bits 7..0.
 * The SN length is 12 bits (clause 6.3.2, Table 6.3.2-1). D/C is 1 for a Data PDU and 0 for a
 * Control PDU (clause 6.3.7). The R bits are set to 0 and ignored by the receiver (clause 6.3.6).
 * No MAC-I is appended: it is present on a DRB only when integrity protection is configured
 * (clause 6.3.4), which this model never does.
 *
 * In EN-DC an MN-terminated split bearer uses NR PDCP (3GPP TS 37.340 clause 4.2.2), so the
 * dual-connectivity PDCP entities (McEnbPdcp, McUePdcp) write and read this header.
 */
class NrPdcpHeader : public Header
{
  public:
    /// The values of the D/C field (3GPP TS 38.323 V19.1.0 clause 6.3.7, Table 6.3.7-1).
    enum DcBit_t : uint8_t
    {
        CONTROL_PDU = 0,
        DATA_PDU = 1
    };

    /// The size of the PDCP SN space: SNs 0 to 4095 (12 bits, clause 6.3.2).
    static constexpr uint32_t SN_SPACE = 4096;

    /// A Data PDU header with SN 0.
    NrPdcpHeader();

    /**
     * \param dcBit the D/C field, CONTROL_PDU or DATA_PDU
     */
    void SetDcBit(uint8_t dcBit);

    /**
     * \param sequenceNumber the PDCP SN; it must be below SN_SPACE
     */
    void SetSequenceNumber(uint16_t sequenceNumber);

    /// \return the D/C field
    uint8_t GetDcBit() const;

    /// \return the PDCP SN
    uint16_t GetSequenceNumber() const;

    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    void Print(std::ostream& os) const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;

  private:
    uint8_t m_dcBit;           ///< the D/C field
    uint16_t m_sequenceNumber; ///< the 12-bit PDCP SN
};

/**
 * \ingroup lte
 * void when \p PdcpDataPduHeader is NrPdcpHeader, and no type otherwise. A function template
 * that takes the Data PDU header class of a dual-connectivity bearer returns this, so a call
 * that names another header class (the E-UTRA LtePdcpHeader is byte-identical at 12 bits) does
 * not compile.
 */
template <class PdcpDataPduHeader>
using IfNrPdcpHeader = std::enable_if_t<std::is_same<PdcpDataPduHeader, NrPdcpHeader>::value>;

} // namespace ns3

#endif // NR_PDCP_HEADER_H
