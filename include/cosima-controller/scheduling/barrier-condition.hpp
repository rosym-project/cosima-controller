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

#pragma once

#include <string>
#include <vector>
#include <map>
#include <mutex>
// #include <shared_mutex> // perhaps in the future...

#include <rtt/Port.hpp>

#include "barrier-data.hpp"

namespace cosima
{

class BarrierCondition
{
public:
  /**
     * Constructor.
     */
  BarrierCondition();

  /**
     * Constructor that takes the name of the target TaskContext.
     */
  BarrierCondition(std::string ttcName);

  /**
     * Set the target task context name.
     */
  void setTargetTaskContextName(std::string ttcName);

  /**
     * Set the barrier task context names as batch.
     */
  void setBarrierTaskContextNamesBatch(std::vector<std::string> btcNames);

  /**
     * Add a battier task context name.
     */
  void addBarrierTaskContextName(std::string btcName);

  /**
     * Get the target task context name.
     */
  const std::string getTargetTaskContextName();

  /**
     * Get all barrier task context names.
     */
  const std::vector<std::string> getBarrierTaskContextNames();

  /**
     * Add new data to the barrier data storage.
     */
  void addBarrierData(std::shared_ptr<BarrierData> data);

  /**
     * Get the barrier data storage.
     */
  std::vector<std::shared_ptr<BarrierData>> getBarrierData();

  /**
     * Check if all barrier data is true.
     */
  bool isFulfilled();

  /**
   * Check if the barrier data is used in this barrier condition.
   */
  bool isBarrierDataRelated(std::shared_ptr<BarrierData> bd);

  /**
   * Reset all barrier data.
   */
  void resetAllBarrierData();

  /**
   * Return a string containing the current state.
   */
  std::string printState();

private:
  /**
     * Name of the TaskContext for which the barrier applies.
     */
  std::string m_targetTaskContextName;

  /**
     * Vector containing all the TaskContext names considered by the barrier for m_targetTaskContextName.
     */
  std::vector<std::string> m_barrierTaskContextNames;

  /**
     * This vector holds all the data associated with this barrier condition.
     *
     * TODO perhaps data could also hold a pointer to the port etc. instead of just being a bool?
     */
  std::vector<std::shared_ptr<BarrierData>> m_barrierData;

  /**
   * Mutex to sync data access.
   */
  // shared_mutex // perhaps in the future...
  std::mutex mutex; // mutable // ?
};

} // namespace cosima