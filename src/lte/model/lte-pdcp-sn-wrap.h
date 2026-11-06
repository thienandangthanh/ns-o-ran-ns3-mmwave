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

#ifndef LTE_PDCP_SN_WRAP_H
#define LTE_PDCP_SN_WRAP_H

#include <cstdint>

namespace ns3
{

/**
 * The size of the PDCP SN space of the data radio bearers in this module: every DRB PDCP
 * entity (LtePdcp, McEnbPdcp, McUePdcp) uses a 12-bit SN that wraps from 4095 to 0
 * (TS 36.323 clause 6.2.3, TS 38.323 clause 6.2.2.2), and LtePdcpHeader carries 12 SN bits.
 */
constexpr uint32_t PDCP_SN_SPACE = 4096;

/**
 * \brief Whether an already transmitted SDU joins a handover forwarding buffer.
 *
 * At a lossless handover the source forwards, ahead of its unacknowledged SDUs, the
 * transmitted SDUs whose SN is the first forwarded SN or one of the two SNs before it.
 * "Before" is modulo the SN space, so for a first forwarded SN of 0 the window is
 * {4094, 4095, 0} and for 1 it is {4095, 0, 1}.
 *
 * \param sn the PDCP SN of a transmitted SDU, below PDCP_SN_SPACE
 * \param firstForwardedSn the PDCP SN at the head of the forwarding buffer, below PDCP_SN_SPACE
 * \return true iff (firstForwardedSn - sn) modulo PDCP_SN_SPACE is 0, 1 or 2
 */
bool IsPdcpSnInForwardingLookback(uint16_t sn, uint16_t firstForwardedSn);

} // namespace ns3

#endif /* LTE_PDCP_SN_WRAP_H */
