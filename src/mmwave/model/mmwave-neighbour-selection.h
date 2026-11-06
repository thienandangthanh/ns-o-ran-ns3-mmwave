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

#ifndef MMWAVE_NEIGHBOUR_SELECTION_H
#define MMWAVE_NEIGHBOUR_SELECTION_H

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

namespace ns3
{

namespace mmwave
{

/**
 * \brief One neighbour cell of a UE's L3 report: its cell ID and its linear L3 SINR.
 */
struct NeighbourSinr
{
    uint16_t cellId;         ///< the neighbour cell's ID
    long double sinrLinear;  ///< the UE's L3 SINR towards that cell, linear scale
};

/**
 * \brief Select the neighbour cells reported in a UE's L3 RRC measurement report.
 *
 * The eligible cells are the entries of \p sinrByCell whose cell ID is not
 * \p servingCellId and whose SINR is a valid linear value (>= 0; NaN and negative
 * readings are skipped, so the cell is not reported, and a gate that expects
 * min(\p maxNeighbours, other cells) neighbours sees the gap). The result holds the min(\p maxNeighbours, eligible) eligible
 * cells with the highest SINR, in descending SINR order; among equal SINRs the
 * higher cell ID comes first, which is the order the former reverse walk over the
 * SINR-sorted multimap produced. The result size is the number of neighbours emitted,
 * so callers pad their fixed-width records from it.
 *
 * \param sinrByCell the UE's L3 SINR per cell ID (linear scale)
 * \param servingCellId the cell ID of the reporting (serving) cell
 * \param maxNeighbours the most neighbours a report may carry
 * \return the selected neighbours, strongest first
 */
std::vector<NeighbourSinr> SelectNeighbourCells(const std::map<uint16_t, long double>& sinrByCell,
                                                uint16_t servingCellId,
                                                std::size_t maxNeighbours);

} // namespace mmwave

} // namespace ns3

#endif /* MMWAVE_NEIGHBOUR_SELECTION_H */
