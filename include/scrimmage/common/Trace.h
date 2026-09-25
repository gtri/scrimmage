/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @section DESCRIPTION
 *
 * Opt-in comparison trace for scrimmage-rs (branch scrimmage-rs-instrumentation).
 * Set SCRIMMAGE_TRACE to a file path to write one JSON object per line.
 * When unset, every function returns immediately and nothing is written, so
 * simulation behavior and standard logs are unchanged. The trace only reads
 * simulator state; it never changes it.
 */

#ifndef INCLUDE_SCRIMMAGE_COMMON_TRACE_H_
#define INCLUDE_SCRIMMAGE_COMMON_TRACE_H_

#include <scrimmage/fwd_decl.h>

#include <list>
#include <string>

namespace scrimmage {
namespace trace {

bool enabled();

/// A message became available to a subscriber (immediate network delivery).
void delivery(
    double t,
    const std::string& network,
    const std::string& topic,
    const EntityPluginPtr& from,
    const EntityPluginPtr& to,
    const MessageBasePtr& msg);

/// A message was queued for delayed delivery at msg->time.
void scheduled_delivery(
    double t,
    const std::string& network,
    const std::string& topic,
    const EntityPluginPtr& from,
    const EntityPluginPtr& to,
    const MessageBasePtr& msg);

/// Each entity's belief state, at the same point as the frame log.
void beliefs(double t, const std::list<EntityPtr>& ents);

/// Autonomy and controller output variables after all controller substeps,
/// i.e. the values the motion model reads this step.
void outputs(double t, const std::list<EntityPtr>& ents);

}  // namespace trace
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_COMMON_TRACE_H_
