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
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Ethan M Boos <ethan.boos@gtri.gatech.edu>
 * @date 9 April 2026
 * @version 0.1.0
 *
 * @section DESCRIPTION
 * Ogre3D's scene graph is not thread-safe. This module provides
 * a command queue for safely passing render requests from the
 * simulation thread to the render thread.
 */

#ifndef INCLUDE_SCRIMMAGE_VIEWER_OGRE_RENDERCOMMAND_H_
#define INCLUDE_SCRIMMAGE_VIEWER_OGRE_RENDERCOMMAND_H_

#include <memory>
#include <mutex>
#include <queue>
#include <variant>
#include "scrimmage/proto/Contact.pb.h"
#include "scrimmage/proto/Frame.pb.h"
#include "scrimmage/proto/Shape.pb.h"
#include "scrimmage/proto/Visual.pb.h"

namespace scrimmage {
namespace viewer {

/*
 *
 */
enum class RenderCommandType {
    UPDATE_FRAME,
    UPDATE_CONTACT_VISUAL,
    ADD_SHAPES,
    REMOVE_SHAPE,
    UPDATE_CAMERA,
    SET_TIME,
    SHUTDOWN
};

/*
 *
 */
struct FrameData {
    std::shared_ptr<scrimmage_proto::Frame> frame;
};

/*
 *
 */
struct ContactVisualData {
    std::shared_ptr<scrimmage_proto::ContactVisual> visual;
};

/*
 *
 */
struct ShapeData {
    scrimmage_proto::Shapes shapes;
};

/*
 *
 */
struct CameraData {
    double pos_x, pos_y, pos_z;
    double focal_x, focal_y, focal_z;
};

/*
 *
 */
struct TimeData {
    double time;
};

/*
 *
 */
struct EmptyData {};

/*
 *
 */
struct RenderCommand {
    RenderCommandType type;
    std::variant<FrameData, ContactVisualData, ShapeData,
                 CameraData, TimeData, EmptyData> data;
};

/*
 *
 *
 * Simulation thread pushes commands, render thread processes them.
 */
class RenderCommandQueue {
 public:
    RenderCommandQueue() = default;
    ~RenderCommandQueue() = default;

    /*
     *
     */
    void push(RenderCommand cmd) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(std::move(cmd));
    }

    /*
     *
     * @return True if a command was popped, false if queue was empty.
     */
    bool pop(RenderCommand& cmd) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        cmd = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    /*
     *
     * @return Vector of all commands that were in the queue.
     */
    std::vector<RenderCommand> drainAll() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<RenderCommand> commands;
        commands.reserve(queue_.size());
        while (!queue_.empty()) {
            commands.push_back(std::move(queue_.front()));
            queue_.pop();
        }
        return commands;
    }

    /*
     *
     */
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    /*
     *
     */
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    /*
     *
     */
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty()) {
            queue_.pop();
        }
    }

 private:
    mutable std::mutex mutex_;
    std::queue<RenderCommand> queue_;
};

}  // namespace viewer
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_VIEWER_OGRE_RENDERCOMMAND_H_
