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
 * \file mmwave-neighbour-list-test.cc
 * \ingroup test
 *
 * \brief Tests the neighbour list of a UE's L3 RRC report (SelectNeighbourCells).
 *
 * The report carries the min(8, eligible) strongest cells other than the serving
 * cell, strongest first, with equal SINRs ordered by the higher cell ID first.
 * The cases cover map sizes 1, 2, 8, 9 and 10, equal-SINR ties, a serving cell
 * that is absent from the map, and the 2-entry map whose serving cell is both the
 * higher cell ID and the strongest, which the former reverse loop (stopping at
 * --begin(), undefined behaviour) reported with no neighbour. Invalid readings (NaN,
 * negative) are never reported, including when the cap cuts the list; zero and
 * +infinity are valid and ordered.
 */

#include "ns3/mmwave-enb-net-device.h"
#include "ns3/mmwave-neighbour-selection.h"
#include "ns3/test.h"

#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;
using namespace mmwave;

namespace
{

/** The neighbour cap the eNB device applies (E2SM_REPORT_MAX_NEIGH). */
const std::size_t kMaxNeighbours = MmWaveEnbNetDevice::E2SM_REPORT_MAX_NEIGH;

std::string
CellIds(const std::vector<NeighbourSinr>& list)
{
    std::ostringstream out;
    for (std::size_t i = 0; i < list.size(); i++)
    {
        out << (i ? "," : "") << list[i].cellId;
    }
    return out.str();
}

std::string
CellIds(const std::vector<uint16_t>& ids)
{
    std::ostringstream out;
    for (std::size_t i = 0; i < ids.size(); i++)
    {
        out << (i ? "," : "") << ids[i];
    }
    return out.str();
}

/** A map of cells 1..n where cell c has linear SINR c * step (a higher cell is stronger). */
std::map<uint16_t, long double>
AscendingMap(uint16_t n, long double step)
{
    std::map<uint16_t, long double> m;
    for (uint16_t c = 1; c <= n; c++)
    {
        m[c] = c * step;
    }
    return m;
}

} // namespace

/**
 * \brief One neighbour-list case: a SINR map, a serving cell and the exact expected cell order.
 */
class MmWaveNeighbourListTestCase : public TestCase
{
  public:
    MmWaveNeighbourListTestCase(const std::string& name,
                                std::map<uint16_t, long double> sinrByCell,
                                uint16_t servingCellId,
                                std::vector<uint16_t> expected)
        : TestCase(name),
          m_sinrByCell(std::move(sinrByCell)),
          m_servingCellId(servingCellId),
          m_expected(std::move(expected))
    {
    }

  private:
    void DoRun() override
    {
        std::vector<NeighbourSinr> got =
            SelectNeighbourCells(m_sinrByCell, m_servingCellId, kMaxNeighbours);
        NS_TEST_ASSERT_MSG_EQ(CellIds(got),
                              CellIds(m_expected),
                              "neighbour cells and their order");
        for (const NeighbourSinr& n : got)
        {
            NS_TEST_ASSERT_MSG_NE(n.cellId, m_servingCellId, "the serving cell is never a neighbour");
            NS_TEST_ASSERT_MSG_EQ(n.sinrLinear,
                                  m_sinrByCell.at(n.cellId),
                                  "each neighbour keeps its own SINR");
        }
    }

    std::map<uint16_t, long double> m_sinrByCell;
    uint16_t m_servingCellId;
    std::vector<uint16_t> m_expected;
};

/**
 * \brief The mmwave-neighbour-list test suite.
 */
class MmWaveNeighbourListTestSuite : public TestSuite
{
  public:
    MmWaveNeighbourListTestSuite()
        : TestSuite("mmwave-neighbour-list", UNIT)
    {
        // Size 1: only the serving cell, so no neighbour.
        AddTestCase(new MmWaveNeighbourListTestCase("size 1, serving only", {{2, 5.0L}}, 2, {}),
                    TestCase::QUICK);
        // Size 2, the case the former loop dropped: the serving cell (3) is the higher cell
        // ID and the strongest, so the only neighbour (2) must still be reported.
        AddTestCase(new MmWaveNeighbourListTestCase("size 2, serving higher and strongest",
                                                    {{2, 1.0L}, {3, 9.0L}},
                                                    3,
                                                    {2}),
                    TestCase::QUICK);
        AddTestCase(new MmWaveNeighbourListTestCase("size 2, serving lower and strongest",
                                                    {{2, 9.0L}, {3, 1.0L}},
                                                    2,
                                                    {3}),
                    TestCase::QUICK);
        AddTestCase(new MmWaveNeighbourListTestCase("size 2, serving higher and weaker",
                                                    {{2, 9.0L}, {3, 1.0L}},
                                                    3,
                                                    {2}),
                    TestCase::QUICK);
        AddTestCase(new MmWaveNeighbourListTestCase("size 2, serving lower and weaker",
                                                    {{2, 1.0L}, {3, 9.0L}},
                                                    2,
                                                    {3}),
                    TestCase::QUICK);
        // Size 8 with the serving cell inside: 7 eligible, all reported (the former count was 8).
        AddTestCase(new MmWaveNeighbourListTestCase("size 8, serving strongest",
                                                    AscendingMap(8, 1.0L),
                                                    8,
                                                    {7, 6, 5, 4, 3, 2, 1}),
                    TestCase::QUICK);
        AddTestCase(new MmWaveNeighbourListTestCase("size 8, serving weakest",
                                                    AscendingMap(8, 1.0L),
                                                    1,
                                                    {8, 7, 6, 5, 4, 3, 2}),
                    TestCase::QUICK);
        // Size 9: 8 eligible, all reported.
        AddTestCase(new MmWaveNeighbourListTestCase("size 9, serving in the middle",
                                                    AscendingMap(9, 1.0L),
                                                    5,
                                                    {9, 8, 7, 6, 4, 3, 2, 1}),
                    TestCase::QUICK);
        // Size 10: 9 eligible, capped at the 8 strongest.
        AddTestCase(new MmWaveNeighbourListTestCase("size 10, serving strongest",
                                                    AscendingMap(10, 1.0L),
                                                    10,
                                                    {9, 8, 7, 6, 5, 4, 3, 2}),
                    TestCase::QUICK);
        AddTestCase(new MmWaveNeighbourListTestCase("size 10, serving weakest",
                                                    AscendingMap(10, 1.0L),
                                                    1,
                                                    {10, 9, 8, 7, 6, 5, 4, 3}),
                    TestCase::QUICK);
        // Equal-SINR ties: the higher cell ID first, and the cap cuts inside the tie group.
        AddTestCase(new MmWaveNeighbourListTestCase("size 2, tie",
                                                    {{2, 4.0L}, {3, 4.0L}},
                                                    3,
                                                    {2}),
                    TestCase::QUICK);
        {
            std::map<uint16_t, long double> tied;
            for (uint16_t c = 1; c <= 10; c++)
            {
                tied[c] = 4.0L;
            }
            AddTestCase(new MmWaveNeighbourListTestCase("size 10, all tied",
                                                        tied,
                                                        4,
                                                        {10, 9, 8, 7, 6, 5, 3, 2}),
                        TestCase::QUICK);
        }
        AddTestCase(new MmWaveNeighbourListTestCase(
                        "size 9, ties among the strongest",
                        {{1, 1.0L}, {2, 7.0L}, {3, 7.0L}, {4, 2.0L}, {5, 7.0L},
                         {6, 3.0L}, {7, 9.0L}, {8, 0.5L}, {9, 9.0L}},
                        4,
                        {9, 7, 5, 3, 2, 6, 1, 8}),
                    TestCase::QUICK);
        // Invalid readings: NaN and negative linear SINRs are skipped before ordering.
        {
            const long double nan = std::numeric_limits<long double>::quiet_NaN();
            std::map<uint16_t, long double> withNan = AscendingMap(10, 1.0L);
            withNan[3] = nan;
            withNan[9] = nan;
            AddTestCase(new MmWaveNeighbourListTestCase("NaN readings skipped",
                                                        withNan,
                                                        1,
                                                        {10, 8, 7, 6, 5, 4, 2}),
                        TestCase::QUICK);
            std::map<uint16_t, long double> capped = AscendingMap(11, 1.0L);
            capped[5] = nan;
            AddTestCase(new MmWaveNeighbourListTestCase("NaN reading skipped, cap applies",
                                                        capped,
                                                        1,
                                                        {11, 10, 9, 8, 7, 6, 4, 3}),
                        TestCase::QUICK);
            AddTestCase(new MmWaveNeighbourListTestCase(
                            "zero and infinity kept, negative skipped",
                            {{2, 0.0L},
                             {3, std::numeric_limits<long double>::infinity()},
                             {4, -1.0L},
                             {5, 2.0L}},
                            9,
                            {3, 5, 2}),
                        TestCase::QUICK);
        }
        // A serving cell absent from the map: every entry is eligible.
        AddTestCase(new MmWaveNeighbourListTestCase("serving absent",
                                                    {{2, 1.0L}, {3, 9.0L}},
                                                    7,
                                                    {3, 2}),
                    TestCase::QUICK);
    }
};

/// The static instance that registers the suite.
static MmWaveNeighbourListTestSuite g_mmwaveNeighbourListTestSuite;
