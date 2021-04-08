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
#include <atomic>

namespace cosima
{

class BarrierData
{
public:
  /**
   * Constructor.
   */
  BarrierData();

  /**
   * Constructor with parameters.
   */
  BarrierData(std::string dataName, bool dataState);

  /**
   * Set the name for the data.
   */
  void setDataName(std::string dataName);

  /**
   * Set the state for the data.
   */
  void setDataState(bool dataState);

  /**
   * Get the name for the data.
   */
  const std::string getDataName();

  /**
   * Get the state for the data.
   */
  bool getDataState();

private:
  /**
   * Name of the data.
   */
  std::string m_dataName;

  /**
   * State of the data.
   */
  std::atomic<bool> m_dataState;
};

} // namespace cosima