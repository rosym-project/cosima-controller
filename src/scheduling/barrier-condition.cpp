/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2018 by Dennis Leroy Wigand <dwigand at cor-lab dot uni-bielefeld dot de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the ``LGPL''),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CoR-Lab, Research Institute for Cognition and Robotics
 *     Bielefeld University
 *
 * ============================================================ */

#include "../../include/cosima-controller/scheduling/barrier-condition.hpp"

using namespace cosima;

BarrierCondition::BarrierCondition()
{
}

BarrierCondition::BarrierCondition(std::string ttcName)
{
    m_targetTaskContextName = ttcName;
}

void BarrierCondition::setBarrierTaskContextNamesBatch(std::vector<std::string> btcNames)
{
    m_barrierTaskContextNames = btcNames;
}

void BarrierCondition::addBarrierTaskContextName(std::string btcName)
{
    m_barrierTaskContextNames.push_back(btcName);
}

const std::string BarrierCondition::getTargetTaskContextName()
{
    return m_targetTaskContextName;
}

const std::vector<std::string> BarrierCondition::getBarrierTaskContextNames()
{
    return m_barrierTaskContextNames;
}

void BarrierCondition::addBarrierData(std::shared_ptr<BarrierData> data)
{
    std::lock_guard<std::mutex> lock(mutex);
    m_barrierData.push_back(data);
}

std::vector<std::shared_ptr<BarrierData>> BarrierCondition::getBarrierData()
{
    std::lock_guard<std::mutex> lock(mutex);
    return m_barrierData;
}

bool BarrierCondition::isFulfilled()
{
    std::lock_guard<std::mutex> lock(mutex);
    for (auto bd : m_barrierData)
    {
        if (!bd->getDataState())
        {
            return false;
        }
    }
    return true;
}

std::string BarrierCondition::printState()
{
    std::string ret = "";
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto bd : m_barrierData)
        {
            if (bd->getDataState())
            {
                ret += "" + bd->getDataName() + " = true\n";
            }
            else
            {
                ret += "" + bd->getDataName() + " = false\n";
            }
        }
    }
    return ret;
}

bool BarrierCondition::isBarrierDataRelated(std::shared_ptr<BarrierData> bd)
{
    int s = -1;
    {
        std::lock_guard<std::mutex> lock(mutex);
        s = m_barrierData.size();
    }
    for (int i = 0; i < s; i++)
    {
        if (m_barrierData.at(i) == bd)
        {
            return true;
        }
    }
    return false;
}

void BarrierCondition::resetAllBarrierData()
{
    std::lock_guard<std::mutex> lock(mutex);
    for (auto bd : m_barrierData)
    {
        bd->setDataState(false);
    }
}