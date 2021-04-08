/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2018 by Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>
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

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>
#include <map>
#include <mutex>

#include <boost/lexical_cast.hpp>

// header for introspection
#include "../introspection/rtt-introspection-base.hpp"

#include "barrier-condition.hpp"
#include "barrier-data.hpp"

namespace cosima
{

class CoreScheduler : public cogimon::RTTIntrospectionBase
{
public:
  /**
   * Constructor.
   */
  CoreScheduler(std::string const &name);

  ///////////////////////////////////////////
  // Internal mirrors of the default Orocos
  // life cycle from the introspection base.
  ///////////////////////////////////////////
  bool configureHookInternal();
  bool startHookInternal();
  void updateHookInternal();
  void stopHookInternal();
  void cleanupHookInternal();
  ///////////////////////////////////////////

  /**
   * Override to intercept event port triggers.
   */
  bool dataOnPortHook(RTT::base::PortInterface *port);

  /**
   * Returns the time provided by the Orocos API in seconds.
   */
  double getOrocosTime();

  /**
   * Register multiple new barrier condition for targetTCName.
   */
  bool registerBarrierConditionBatch(std::string targetTCName, std::vector<std::string> barrierTCNames);

  /**
   * Register a new barrier condition for targetTCName.
   */
  bool registerBarrierCondition(std::string targetTCName, std::string barrierTCName);

  /**
   * Clear all registered barrier conditions for targetTCName.
   */
  void clearRegisteredBarrierConditions(std::string targetTCName);

  /**
   * Generate the ports and shared data based on the registered barrier conditions.
   */
  bool generatePortsAndData();

  /**
   * Create signal port for a (included) task context.
   */
  std::string createSignalPort(std::string const &tcName);

  /**
   * Debug external trigger mechanism.
   */
  void triggerEventData(std::string portName, bool value);

  /**
   * Set the execution order of the peer task contexts.
   */
  void setExecutionOrder(std::vector<std::string> eo);

private:
  // RTT::InputPort<bool> in_A_port;
  // RTT::InputPort<bool> in_B_port;
  // RTT::FlowStatus in_A_flow;
  // RTT::FlowStatus in_B_flow;
  // RTT::OutputPort<int> out_exec;

  double m_startTime; /**< Start time. Details. */

  /**
   * List holding the TaskContexts.
   * TCs are represented by the registered peers.
   */
  std::vector<RTT::TaskContext *> m_tcList;

  /**
   * Stores the execution order of the peer TCs. This is used to restructure m_tcList.
   */
  std::vector<std::string> m_executionOrderOfTCs;

  ///////////////////
  /// EXAMPLE:
  /// (A ---lo--> B)
  ///    (C -in-> B)
  ///////////////////

  // /**
  //  * Example event input port that acts as trigger to wait for C so B can be executed.
  //  */
  // RTT::InputPort<BarrierData> ev_trigger_C_B;
  // RTT::FlowStatus ev_trigger_C_B_flow;
  // BarrierData ev_trigger_C_B_data;
  // /**
  //  * Example local trigger to wait for A (to finish) so B can be executed.
  //  */
  // BarrierData lo_trigger_A_B_data;

  // TODO how to generate and dynamically manage the internal state?
  // Use a hashmap to encode the name and ~data~: "ev_trigger_C_B_data" : ~ev_trigger_C_B_data~.
  // Get pointer to data from the hashmap and store them in a condition object,
  // so we do not have to do the look up all the time.
  // Then we also can have for each step/state a condition/barrier to wait for.
  // And this we are going to yield in the update hook!
  // Further we need to make sure that the CoreScheduler with the first component of the chain,
  // is triggered by frequency and triggers all the other CoreScheduler data-flow-basedat the beginning
  // of every iteration.
  // Input can then be some kind of cvs/json format to specify the components, order, and their barrier-conditions.

  /**
   * Map access to all registered barrier conditions.
   */
  std::map<std::string, std::shared_ptr<BarrierCondition>> m_barrierConditions;

  /**
   * With this we encode the state information.
   */
  std::shared_ptr<BarrierCondition> m_activeBarrierCondition;

  /**
   * Holds the index to the active task context in m_tcList.
   */
  int m_activeTaskContextIndex;

  /**
   * Hold a pointer to the active task context.
   */
  RTT::TaskContext *m_activeTaskContextPtr;

  /**
   * This map allows (hopefully) fast access to the data pointer based on the associated port interface.
   */
  std::map<RTT::base::PortInterface *, std::shared_ptr<BarrierData>> m_mapPortToDataPtr;

  /**
   * Prints debug information including registered port, barrier conditions, and so on.
   */
  void printDebugInformation();

  /**
   * Container for generated input ports. Because I had some problems without such an container before.
   * Perhaps there is a smarter solution to this.
   */
  std::vector<std::shared_ptr<RTT::InputPort<bool>>> genPortEvInputPtrs;

  /**
   * Container for generated output ports. Because I had some problems without such an container before.
   * Perhaps there is a smarter solution to this.
   */
  std::map<std::string, std::shared_ptr<RTT::OutputPort<bool>>> genPortOutputSignalPtrs;

  /**
   * Set the activity of the passed task context to a new SlaveActivity.
   */
  void setSlaveActivityFor(RTT::TaskContext *new_block);

  /**
   * Condition variable to decide if we yield after every component execution,
   * or only after we found a barrier and execute unconditioned execution sequentially in the same updateHook().
   */
  bool m_doNextIterationWithoutTrigger;

  /**
   * Operation for setting m_doNextIterationWithoutTrigger.
   */
  void alwaysYieldAfterEachComponentExecution(bool alwaysYield);

  /**
   * Global trigger port, can only be used by the master core scheduler,
   * which contains the component that is last executed in a PTG.
   */
  std::shared_ptr<RTT::OutputPort<bool>> globalTriggerPort;

  /**
   * Operation to create globalTriggerPort.
   */
  std::string createGlobalSignalPort();

  /**
   * Create global event port that when called triggers the dataOnPort mechanism.
   * This is used to receive anotification from the core scheduler master.
   */
  std::string createGlobalEventPort();

  /**
   * Storage for global event receive ports that can only be used by non master core scheduler.
   */
  std::shared_ptr<RTT::InputPort<bool>> globalEventPort;

  RTT::OutputPort<bool> debugPort;

  std::mutex mutex;
};

} // namespace cosima
