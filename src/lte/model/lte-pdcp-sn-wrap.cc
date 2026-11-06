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

#include "lte-pdcp-sn-wrap.h"

#include "ns3/assert.h"

namespace ns3
{

namespace
{

/** The number of SN steps from \p from forward to \p to, modulo the SN space. */
uint32_t
PdcpSnDistance(uint16_t from, uint16_t to)
{
    NS_ASSERT_MSG(from < PDCP_SN_SPACE && to < PDCP_SN_SPACE, "PDCP SN outside the 12-bit space");
    return (static_cast<uint32_t>(to) + PDCP_SN_SPACE - from) % PDCP_SN_SPACE;
}

} // namespace

bool
IsPdcpSnInForwardingLookback(uint16_t sn, uint16_t firstForwardedSn)
{
    return PdcpSnDistance(sn, firstForwardedSn) <= 2;
}

} // namespace ns3
