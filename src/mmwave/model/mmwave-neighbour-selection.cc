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

#include "mmwave-neighbour-selection.h"

#include <algorithm>

namespace ns3
{

namespace mmwave
{

std::vector<NeighbourSinr>
SelectNeighbourCells(const std::map<uint16_t, long double>& sinrByCell,
                     uint16_t servingCellId,
                     std::size_t maxNeighbours)
{
    std::vector<NeighbourSinr> eligible;
    eligible.reserve(sinrByCell.size());
    for (const auto& entry : sinrByCell)
    {
        // A reading is a valid linear SINR only if it is >= 0. NaN fails this test, so the
        // comparator below only ever sees ordered values (strict weak ordering holds, +inf
        // included), and no reading whose dB conversion would be NaN reaches the report.
        if (entry.first != servingCellId && entry.second >= 0)
        {
            eligible.push_back(NeighbourSinr{entry.first, entry.second});
        }
    }
    // Strongest first; equal SINRs put the higher cell ID first, so the order is a
    // total order and never depends on the container's insertion history.
    std::sort(eligible.begin(),
              eligible.end(),
              [](const NeighbourSinr& a, const NeighbourSinr& b) {
                  if (a.sinrLinear != b.sinrLinear)
                  {
                      return a.sinrLinear > b.sinrLinear;
                  }
                  return a.cellId > b.cellId;
              });
    if (eligible.size() > maxNeighbours)
    {
        eligible.resize(maxNeighbours);
    }
    return eligible;
}

} // namespace mmwave

} // namespace ns3
