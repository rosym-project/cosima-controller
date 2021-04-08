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
#include <utility>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/TaskContext.hpp>

namespace cosima
{

class CoreSchedulerService : public RTT::Service
{

public:
  /**
   * Constructor.
   */
  CoreSchedulerService(RTT::TaskContext *owner);

private:
  /**
   * Set the execution order of the task contexts for a specific core scheduler.
   */
  void setExecutionOrder(std::string const &csName, std::vector<std::string> tcNames);

  /**
   * Add a PTG formula in the following form: source(before) -> target(after).
   */
  void addPTGFormula(std::string const &sourceTcName, std::string const &targetTcName);

  /**
   * Set the involved core schedulers.
   */
  void setInvolvedCoreScheduler(std::vector<std::string> csNames);

  /**
   * Set the component that is executed last in the PTG.
   */
  void setLastComponentInPTG(std::string csName);

  /**
   * Print debug information.
   */
  void printDebugInformation();

  /**
   * Do your thang!
   */
  bool configure();

  /**
   * Start all involved CS!
   */
  bool start();

  /**
   * Owner pointer.
   */
  RTT::TaskContext *gOwner;

  /**
   * Store for the execution order of the core scheduler.
   */
  std::map<std::string, std::vector<std::string>> m_execution_order;

  /**
   * Store for the PTG formulas.
   */
  std::vector<std::pair<std::string, std::string>> m_ptg_formulas;

  /**
   * Store all manual registered core scheduler task contexts until we find a way to automatically detect those.
   */
  std::vector<std::string> m_csTaskContexts;

  /**
   * Storage for core scheduler task context pointers.
   */
  std::vector<RTT::TaskContext *> m_coreSchedulerPtrs;

  /**
   * Storage for normal task context pointers.
   */
  std::vector<RTT::TaskContext *> m_normalTcPtrs;

  /**
   * Storage for all port connections.
   */
  std::vector<std::pair<std::pair<RTT::TaskContext *, std::string>, std::pair<RTT::TaskContext *, std::string>>> portsToBeConnected;

  /**
   * Stores the component that is executed last in the PTG.
   * This information is used to determine the master of the core scheduler,
   * which notifies the other cs if a global iteration of the PTG has ended / a new one starts.
   */
  std::string m_lastComponentInPTG;
};

} // namespace cosima