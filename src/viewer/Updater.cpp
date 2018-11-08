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
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkDataSetMapper.h>
#include <vtkPNGReader.h>
#include <vtkOBJReader.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTriangle.h>
#include <vtkArrowSource.h>
#include <vtkVectorText.h>
#include <vtkCamera.h>
#include <vtkJPEGReader.h>
#include <vtkPolyDataReader.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkPolyDataNormals.h>
#include <vtkTextureMapToPlane.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolygon.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkCallbackCommand.h>
#include <vtkCellArray.h>
#include <vtkSphereSource.h>
#include <vtkCubeSource.h>
#include <vtkPyramid.h>
#include <vtkUnstructuredGrid.h>
#include <vtkConeSource.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPlaneSource.h>
#include <vtkRegularPolygonSource.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

#include <scrimmage/common/FileSearch.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/Quaternion.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/network/Interface.h>
#include <scrimmage/parse/ConfigParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/viewer/OriginAxes.h>
#include <scrimmage/viewer/Updater.h>
#include <scrimmage/viewer/Grid.h>

#include <iostream>

#include <boost/filesystem.hpp>

using std::cout;
using std::endl;
namespace fs = boost::filesystem;

#define BILLION 1000000000L

namespace sp = scrimmage_proto;
namespace sc = scrimmage;

namespace scrimmage {

double fps = 0;
void fpsCallbackFunction(vtkObject* caller, long unsigned int vtkNotUsed(eventId), // NOLINT
                         void* vtkNotUsed(clientData), void* vtkNotUsed(callData)) {
    vtkRenderer* renderer = static_cast<vtkRenderer*>(caller);
    double timeInSeconds = renderer->GetLastRenderTimeInSeconds();
    fps = 1.0/timeInSeconds;
}

Updater::Updater() :
        frame_time_(-1.0), update_count(0), inc_follow_(true),
        dec_follow_(false), view_mode_(ViewMode::FOLLOW), view_mode_prev_(ViewMode::FREE),
        follow_offset_(50), follow_vec_(-50, 10, 10), show_helpmenu_(false) {
    prev_time.tv_nsec = 0;
    prev_time.tv_sec = 0;
    max_update_rate_ = 1.0;

    reset_scale();
    scale_required_ = false;

    follow_id_ = 0;

    enable_trails_ = false;

    gui_msg_.set_inc_warp(false);
    gui_msg_.set_dec_warp(false);
    gui_msg_.set_toggle_pause(false);
    gui_msg_.set_single_step(false);

    send_shutdown_msg_ = true;
}

void Updater::init(const std::string &log_dir, double dt) {
    // Create a default grid:
    grid_ = std::make_shared<Grid>();
    grid_->create(20, 1, renderer_);

    // Create a default origin:
    origin_axes_ = std::make_shared<OriginAxes>();
    origin_axes_->create(1, renderer_);

    renderer_->SetNearClippingPlaneTolerance(0.00001);

    enable_fps();
    log_dir_ = log_dir;
    dt_ = dt;
}

void Updater::Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId), // NOLINT
                      void * vtkNotUsed(callData)) {
    update();

    vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::SafeDownCast(caller);
    iren->GetRenderWindow()->Render();

    incoming_interface_->gui_msg_mutex.lock();
    auto &gui_msg_list = incoming_interface_->gui_msg();
    if (!gui_msg_list.empty()) {
        auto &msg = gui_msg_list.front();
        if (std::abs(msg.time() - frame_time_) < 1e-7 && fs::exists(log_dir_)) {

            vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
                vtkSmartPointer<vtkWindowToImageFilter>::New();
            windowToImageFilter->SetInput(rwi_->GetRenderWindow());
            windowToImageFilter->Update();

            vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();

            const std::string fname =
                log_dir_ + "/screenshot_" + std::to_string(frame_time_) + ".png";
            if (!fs::exists(fname)) {
                writer->SetFileName(fname.c_str());
                writer->SetInputConnection(windowToImageFilter->GetOutputPort());
                writer->Write();
            }
            single_step();
            gui_msg_list.pop_front();
        }
    }
    incoming_interface_->gui_msg_mutex.unlock();
}

bool Updater::update() {
    // Make sure we don't update the contacts faster than the max rate
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    uint64_t diff = BILLION * (now.tv_sec - prev_time.tv_sec) + now.tv_nsec - prev_time.tv_nsec;
    if (diff < (1.0 / max_update_rate_*BILLION)) {
        return true;
    }
    prev_time = now;

    ///////////////////////////////////////////////////////////////////////
    // Network / Shared Memory Messages
    ///////////////////////////////////////////////////////////////////////

    // Do we have any updates to the terrain info
    if (incoming_interface_->utm_terrain_update()) {
        incoming_interface_->utm_terrain_mutex.lock();
        auto &utms = incoming_interface_->utm_terrain();

        // We only care about the last message
        auto &utm = utms.back();

        // Process utm terrain data
        update_utm_terrain(utm);

        utms.clear();
        incoming_interface_->utm_terrain_mutex.unlock();
    }

    // Do we have any updates to the contact_visuals
    if (incoming_interface_->contact_visual_update()) {
        incoming_interface_->contact_visual_mutex.lock();
        auto &cv = incoming_interface_->contact_visual();
        auto it = cv.begin();
        while (it != cv.end()) {
            (*it)->set_update_required(true);
            contact_visuals_[(*it)->id()] = *it;
            cv.erase(it++);
        }
        incoming_interface_->contact_visual_mutex.unlock();
    }

    // Do we have any updates to the frames?
    if (incoming_interface_->frames_update(frame_time_)) {
        incoming_interface_->frames_mutex.lock();
        auto &frames = incoming_interface_->frames();

        // Check to see if we have to remove any contacts.
        // Need to check every frame so we don't miss a discrete removal
        // Maybe we make this it's own message one day.
        for (auto it : frames) {
            for (int i = 0; i < it->contact_size(); i++) {
                if (!it->contact(i).active()) {
                    auto it_ac = actor_contacts_.find(it->contact(i).id().id());
                    if (it_ac != actor_contacts_.end()) {
                        it_ac->second->remove = true;
                    }
                }
            }
        }

        // We only care about the last frame for display purposes
        auto &frame = frames.back();
        update_contacts(frame);
        frames.erase(frames.begin(), std::next(frames.end(), -1));

        // We want the shapes' ttl counter to be linked to newly received
        // frames. Update the shapes on a newly received frame.
        update_shapes();

        incoming_interface_->frames().clear();

        incoming_interface_->frames_mutex.unlock();
    }

    // Do we have any updates to the sim info?
    if (incoming_interface_->sim_info_update()) {
        incoming_interface_->sim_info_mutex.lock();
        auto &info_list = incoming_interface_->sim_info();

        // Check for shutting_down message from simcontrol
        for (auto it : info_list) {
            if (it.shutting_down()) {
                send_shutdown_msg_ = false;
                rwi_->GetRenderWindow()->Finalize();
                rwi_->TerminateApp();
            }
        }

        // We only care about the last message for actual data
        sim_info_ = info_list.back();
        info_list.clear();
        incoming_interface_->sim_info_mutex.unlock();
    }

    // Do we have any new shapes?
    if (incoming_interface_->shapes_update()) {
        incoming_interface_->shapes_mutex.lock();
        auto &shapes_list = incoming_interface_->shapes();

        auto it = shapes_list.begin();
        while (it != shapes_list.end()) {
            draw_shapes(*it);

            // Update ttl for shapes that aren't the last in the list
            if (++it != shapes_list.end()) {
                update_shapes();
            }
            --it;
            shapes_list.erase(it++);
        }
        incoming_interface_->shapes_mutex.unlock();
    }

    // Update scale
    if (scale_required_) {
        update_scale();
        scale_required_ = false;
    }

    ///////////////////////////////////////////////////////////////////////
    // Update camera and GUI elements
    ///////////////////////////////////////////////////////////////////////
    update_camera();
    update_text_display();

    return true;
}

bool Updater::update_scale() {
    for (auto &kv : actor_contacts_) {
        // Get the desired scale amount, which is a factor of the current
        // visualization scale (scale_) and the scale amount associated with
        // the contact visual information
        double desired_scale_amount = 1.0;
        auto it = contact_visuals_.find(kv.first);
        if (it != contact_visuals_.end()) {
            desired_scale_amount = it->second->scale() * scale_;
        }

        // Get the 4x4 matrix that defines the scale, rotation, and translation
        vtkMatrix4x4 *m;
        m = kv.second->actor->GetMatrix();

        // Compute the current scale factor for the object. The top-left 3x3
        // part of the matrix contains the rotation and scale. The length of
        // each row of the 3x3 matrix is the current scaling factor.
        double sq_sum = 0;
        for (int c = 0; c < 3; c++) {
            sq_sum += pow(m->GetElement(0, c), 2);
        }
        double curr_scale_factor = sqrt(sq_sum);

        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                // For each element in the top-left 3x3 matrix, normalize the
                // scaling factor to 1 and then apply the desired scaling
                // amount.
                double elem = m->GetElement(r, c);
                elem = elem / curr_scale_factor * desired_scale_amount;
                m->SetElement(r, c, elem);
            }
        }

        // update the label scale
        kv.second->label->SetScale(label_scale_, label_scale_, label_scale_);
    }
    return true;
}

bool Updater::update_shapes() {
    // Remove past frames shapes that have reached time-to-live, if they
    // are not persistent
    for (auto it = shapes_.begin(); it != shapes_.end();) {
        scrimmage_proto::Shape &s = std::get<0>(it->second);
        s.set_ttl(s.ttl()-1);
        if (!s.persistent() && s.ttl() <= 0) {
            renderer_->RemoveActor(std::get<1>(it->second));
            it = shapes_.erase(it);
        } else {
            ++it;
        }
    }
    return true;
}

bool Updater::draw_shapes(scrimmage_proto::Shapes &shapes) {
    // Display new shapes
    for (int i = 0; i < shapes.shape_size(); i++) {
        sp::Shape shape = shapes.shape(i);

        bool new_shape = false;
        vtkSmartPointer<vtkActor> actor;
        vtkSmartPointer<vtkPolyDataAlgorithm> source;
        vtkSmartPointer<vtkPolyDataMapper> mapper;

        // Does this shape ID exist already?
        auto it = shapes_.find(shape.hash());
        if (it != shapes_.end()) {
            new_shape = false;
            actor = std::get<1>(it->second);
            source = std::get<2>(it->second);
        } else {
            new_shape = true;
            mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            actor = vtkSmartPointer<vtkActor>::New();
        }

        bool shape_status = false;
        switch (shape.oneof_type_case()) {
        case sp::Shape::kTriangle:
            shape_status = draw_triangle(new_shape, shape.triangle(), actor, source, mapper);
            break;
        case sp::Shape::kPlane:
            shape_status = draw_plane(new_shape, shape.plane(), actor, source, mapper);
            break;
        case sp::Shape::kArrow:
            shape_status = draw_arrow(new_shape, shape.arrow(), actor, source, mapper);
            break;
        case sp::Shape::kCone:
            shape_status = draw_cone(new_shape, shape.cone(), actor, source, mapper);
            break;
        case sp::Shape::kLine:
            shape_status = draw_line(new_shape, shape.line(), actor, source, mapper);
            break;
        case sp::Shape::kPolygon:
            shape_status = draw_polygon(new_shape, shape.polygon(), actor, source, mapper);
            break;
        case sp::Shape::kPolydata:
            shape_status = draw_polydata(new_shape, shape.polydata(), actor, source, mapper);
            break;
        case sp::Shape::kCuboid:
            shape_status = draw_cube(new_shape, shape.cuboid(), actor, source, mapper);
            break;
        case sp::Shape::kPointcloud:
            shape_status = draw_pointcloud(new_shape, shape, actor, source, mapper);
            break;
        case sp::Shape::kCircle:
            shape_status = draw_circle(new_shape, shape.circle(), actor, source, mapper);
            break;
        case sp::Shape::kEllipse:
            shape_status = draw_ellipse(new_shape, shape.ellipse(), actor, source, mapper);
            break;
        case sp::Shape::kSphere:
            shape_status = draw_sphere(new_shape, shape.sphere(), actor, source, mapper);
            break;
        case sp::Shape::kText:
            shape_status = draw_text(new_shape, shape.text(), actor, source, mapper);
            break;
        case sp::Shape::kMesh:
            shape_status = draw_mesh(new_shape, shape.mesh(), actor, source, mapper);
            break;
        default:
            break;
        }

        // Only add the actor if it was correctly constructed
        if (shape_status) {
            double opacity = shape.opacity();
            if (opacity < 0.00001) {
                actor->GetProperty()->SetOpacity(1.0);
            } else {
                actor->GetProperty()->SetOpacity(shape.opacity());
            }
            actor->GetProperty()->SetColor(shape.color().r()/255.0,
                                           shape.color().g()/255.0,
                                           shape.color().b()/255.0);

            // Since protobufs default value for int is 0, if the ttl is 0
            // at this point, set ttl to 1. We are creating new shapes
            // here, so it doesn't make sense for a shape to have a ttl
            // less than 1 here.
            if (shape.ttl() <= 0) {
                shape.set_ttl(1);
            }

            if (new_shape) {
                renderer_->AddActor(actor);
                shapes_[shape.hash()] = std::make_tuple(shape, actor, source);
            } else {
                std::get<0>(it->second) = shape;
            }
        }
    }
    return true;
}

void Updater::set_view_mode(ViewMode view_mode) {
    view_mode_ = view_mode;
    switch (view_mode_) {
        case ViewMode::FOLLOW:
            view_mode_actor_->SetInput("View: Follow");
            break;
        case ViewMode::FREE:
            view_mode_actor_->SetInput("View: Free");
            break;
        case ViewMode::OFFSET:
            view_mode_actor_->SetInput("View: Offset");
            break;
        case ViewMode::FPV:
            view_mode_actor_->SetInput("View: FPV");
            break;
        default:
            break;
    }
}

bool Updater::update_camera() {
    // Free mode if contacts don't exist
    if (actor_contacts_.empty()) {
        set_view_mode(ViewMode::FREE);
        return true;
    }

    // Handle changing follow ids
    if (inc_follow_) {
        follow_id_++;
    } else if (dec_follow_) {
        follow_id_--;
    }

    // Find min/max ids
    // this is a map so the front will be the min and the back is the max
    int min = actor_contacts_.begin()->first;
    int max = actor_contacts_.rbegin()->first;

    auto it = actor_contacts_.find(follow_id_);
    while (it == actor_contacts_.end()) {
        if (follow_id_ > max) {
            follow_id_ = actor_contacts_.begin()->second->contact.id().id();
        } else if (follow_id_ < min) {
            follow_id_ = actor_contacts_.rbegin()->second->contact.id().id();
        } else if (inc_follow_) {
            follow_id_++;
        } else if (dec_follow_) {
            follow_id_--;
        } else {
            // The ID might have been removed, increment to search for next
            // available
            follow_id_++;
        }
        it = actor_contacts_.find(follow_id_);
    }
    inc_follow_ = dec_follow_ = false;

    double x_pos = it->second->contact.state().position().x();
    double y_pos = it->second->contact.state().position().y();
    double z_pos = it->second->contact.state().position().z();

    double x_pos_fp = x_pos;
    double y_pos_fp = y_pos;
    double z_pos_fp = z_pos;

    double camera_pos[3] {0, 0, 0};
    if (view_mode_ == ViewMode::FREE) {
        if (reset_camera_) {
            x_pos_fp = camera_reset_params_.focal_x;
            y_pos_fp = camera_reset_params_.focal_y;
            z_pos_fp = camera_reset_params_.focal_z;

            // there appears to be some type of gimbal lock in VTK
            // so the y-position is slightly off of 0
            camera_pos[0] = camera_reset_params_.pos_x;
            camera_pos[1] = camera_reset_params_.pos_y;
            camera_pos[2] = camera_reset_params_.pos_z;

            renderer_->GetActiveCamera()->SetViewUp(0, 1, 0);

        } else {
            return true;
        }
    } else {
        if (view_mode_ == ViewMode::OFFSET) {
            camera_pos[0] = x_pos + 0.0;
            camera_pos[1] = y_pos - 6.0;
            camera_pos[2] = z_pos + 2.0;
        } else if (view_mode_ == ViewMode::FOLLOW) {

            if (view_mode_prev_ == ViewMode::FOLLOW) {
                double currentPos[3];
                renderer_->GetActiveCamera()->GetPosition(currentPos);
                double currentFp[3];
                renderer_->GetActiveCamera()->GetFocalPoint(currentFp);
                for (int i = 0; i < 3; i++)
                    follow_vec_[i] = currentPos[i]-currentFp[i];
                follow_offset_ = follow_vec_.norm();
            }

            Eigen::Vector3d rel_cam_pos = follow_vec_.normalized() * follow_offset_;
            Eigen::Vector3d unit_vector = rel_cam_pos / rel_cam_pos.norm();

            sp::Quaternion sp_quat = it->second->contact.state().orientation();
            sc::Quaternion quat(sp_quat.w(), sp_quat.x(), sp_quat.y(), sp_quat.z());

            Eigen::Vector3d pos = Eigen::Vector3d(x_pos, y_pos, z_pos) +
                unit_vector * rel_cam_pos.norm();

            camera_pos[0] = pos[0];
            camera_pos[1] = pos[1];
            camera_pos[2] = pos[2];

            // Compute the camera's "up" vector
            Eigen::Vector3d z_axis(0, 0, 1);
            renderer_->GetActiveCamera()->SetViewUp(z_axis(0), z_axis(1), z_axis(2));

        } else if (view_mode_ == ViewMode::FPV) {
            sp::Quaternion sp_quat = it->second->contact.state().orientation();
            sc::Quaternion quat(sp_quat.w(), sp_quat.x(), sp_quat.y(), sp_quat.z());

            // Compute camera position
            Eigen::Vector3d base_offset(1, 0, 0);
            Eigen::Vector3d pos = Eigen::Vector3d(x_pos, y_pos, z_pos) +
                quat * base_offset * 5;

            // Compute camera focal point
            Eigen::Vector3d cam_dir = quat.rotate(base_offset.normalized());
            Eigen::Vector3d fp = Eigen::Vector3d(x_pos, y_pos, z_pos) + cam_dir * 100;

            x_pos_fp = fp(0);
            y_pos_fp = fp(1);
            z_pos_fp = fp(2);

            camera_pos[0] = pos[0];
            camera_pos[1] = pos[1];
            camera_pos[2] = pos[2];

            // Compute the camera's "up" vector
            Eigen::Vector3d z_axis(0, 0, 1);
            Eigen::Vector3d up = quat * z_axis;
            renderer_->GetActiveCamera()->SetViewUp(up(0), up(1), up(2));
        }
    }
    view_mode_prev_ = view_mode_;
    reset_camera_ = false;
    renderer_->GetActiveCamera()->SetPosition(camera_pos);
    renderer_->GetActiveCamera()->SetFocalPoint(x_pos_fp, y_pos_fp, z_pos_fp);
    renderer_->ResetCameraClippingRange(); // fixes missing terrain/entity issue
    return true;
}

bool Updater::update_text_display() {
    // Update FPS
    if (show_fps_) {
        std::stringstream stream_fps;
        stream_fps << std::fixed << std::setprecision(2) << fps;
        std::string fps_str = "FPS: " + stream_fps.str();
        fps_actor_->SetInput(fps_str.c_str());
    } else {
        fps_actor_->SetInput(" ");
    }

    // Update the time (text) display
    const int num_digits = std::abs(log10(dt_));
    std::stringstream ss;
    ss << std::setprecision(num_digits) << std::fixed << frame_time_ << " s";
    time_actor_->SetInput(ss.str().c_str());

    // Update the time warp
    std::stringstream stream_warp;
    stream_warp << std::fixed << std::setprecision(2) << sim_info_.desired_warp();
    std::string time_warp_str = stream_warp.str() + " X";
    warp_actor_->SetInput(time_warp_str.c_str());

    // Display information about the aircraft we are following:
    if (view_mode_ == ViewMode::FREE) {
        heading_actor_->SetInput(" ");
        alt_actor_->SetInput(" ");

    } else {
        auto it = actor_contacts_.find(follow_id_);
        if (it != actor_contacts_.end()) {
            sp::Quaternion sp_quat = it->second->contact.state().orientation();
            sc::Quaternion quat(sp_quat.w(), sp_quat.x(), sp_quat.y(), sp_quat.z());
            std::string heading_str = "H: " + std::to_string(sc::Angles::rad2deg(quat.yaw()));
            heading_actor_->SetInput(heading_str.c_str());

            std::string alt_str = "Alt: " + std::to_string(it->second->contact.state().position().z());
            alt_actor_->SetInput(alt_str.c_str());
        }
    }

    return true;
}


void Updater::next_mode() {
    switch (view_mode_) {
        case ViewMode::FOLLOW:
            set_view_mode(ViewMode::FREE);
            break;
        case ViewMode::FREE:
            set_view_mode(ViewMode::OFFSET);
            break;
        case ViewMode::OFFSET:
            set_view_mode(ViewMode::FPV);
            break;
        case ViewMode::FPV:
            set_view_mode(ViewMode::FOLLOW);
            break;
        default:
            set_view_mode(ViewMode::FOLLOW);
            break;
    }
}

bool Updater::update_utm_terrain(std::shared_ptr<scrimmage_proto::UTMTerrain> &utm) {
    // Reset the grid
    grid_->remove();
    if (utm->enable_grid()) {
        grid_->create(utm->grid_size(), utm->grid_spacing(), renderer_);
    }

    // Reset / Show the origin?
    origin_axes_->remove();
    if (utm->show_origin()) {
        origin_axes_->create(utm->origin_length(), renderer_);
    }

    // Set the background color:
    renderer_->SetBackground(utm->background().r() / 255.0,
                             utm->background().g() / 255.0,
                             utm->background().b() / 255.0);

    // Exit if the terrain is disabled in this messasge
    if (!utm->enable_terrain()) return true;

    // If the terrain data already exists in our map, use that instead:
    auto it = terrain_map_.find(utm->terrain_name());
    if (it != terrain_map_.end()) {
        utm = it->second;
    } else {
        // Search for the appropriate files on the local system
        ConfigParse terrain_parse;
        find_terrain_files(utm->terrain_name(),
                           terrain_parse, utm);
        terrain_map_[utm->terrain_name()] = utm;
    }

    if (utm->enable_terrain()) {
        // Read and create texture...
        vtkSmartPointer<vtkJPEGReader> terrain_jPEGReader =
            vtkSmartPointer<vtkJPEGReader>::New();
        terrain_jPEGReader->SetFileName(utm->texture_file().c_str());
        terrain_jPEGReader->Update();

        // Apply the texture
        vtkSmartPointer<vtkTexture> terrain_texture =
            vtkSmartPointer<vtkTexture>::New();
        terrain_texture->SetInputConnection(terrain_jPEGReader->GetOutputPort());
        terrain_texture->InterpolateOn();

        // Read the terrain polydata
        vtkSmartPointer<vtkPolyDataReader> terrain_reader1 =
            vtkSmartPointer<vtkPolyDataReader>::New();
        terrain_reader1->SetFileName(utm->poly_data_file().c_str());
        terrain_reader1->Update();

        vtkSmartPointer<vtkPolyData> polydata;
        polydata = terrain_reader1->GetOutput();

        double bad_z_thresh = -90000;
        double testpoint[3];
        double z_sum = 0;
        int z_count = 0;
        for (vtkIdType n = 0; n < polydata->GetNumberOfPoints(); n++) {
            polydata->GetPoint(n, testpoint);
            if (testpoint[2] > bad_z_thresh) {
                z_sum += testpoint[2];
                z_count++;
            }
        }

        double z_avg = z_sum / z_count;
        for (vtkIdType n = 0; n < polydata->GetNumberOfPoints(); n++) {
            polydata->GetPoint(n, testpoint);
            if (testpoint[2] < bad_z_thresh) {
                testpoint[2] = z_avg;
                polydata->GetPoints()->SetPoint(n, testpoint);
                z_sum += testpoint[2];
            }
        }

        // Setup colors
        vtkSmartPointer<vtkUnsignedCharArray> colors =
            vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3);
        colors->SetName("Colors");
        for (int i = 0; i < polydata->GetNumberOfPoints(); ++i) {
            unsigned char tempColor[3] = {255, 255, 255};
#if VTK_MAJOR_VERSION <= 6
            colors->InsertNextTupleValue(tempColor);
#else
            colors->InsertNextTypedTuple(tempColor);
#endif
        }

        terrain_reader1->Update();
        polydata->GetPointData()->SetScalars(colors);

        vtkSmartPointer<vtkTransform> translation =
            vtkSmartPointer<vtkTransform>::New();
        translation->Translate(-utm->x_translate(), -utm->y_translate(),
                               -utm->z_translate());

        vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
            vtkSmartPointer<vtkTransformPolyDataFilter>::New();
        transformFilter->SetInputConnection(terrain_reader1->GetOutputPort());

        transformFilter->SetTransform(translation);
        transformFilter->Update();

        // Smooth poly data
        vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter =
            vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
        smoothFilter->SetInputConnection(transformFilter->GetOutputPort());
        smoothFilter->SetNumberOfIterations(15);
        smoothFilter->SetRelaxationFactor(0.1);
        smoothFilter->FeatureEdgeSmoothingOff();
        smoothFilter->BoundarySmoothingOn();
        smoothFilter->Update();

        // Update normals on newly smoothed polydata
        vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
        normalGenerator->SetInputConnection(smoothFilter->GetOutputPort());
        normalGenerator->ComputePointNormalsOn();
        normalGenerator->ComputeCellNormalsOn();
        normalGenerator->Update();

        vtkSmartPointer<vtkTextureMapToPlane> texturePlane =
            vtkSmartPointer<vtkTextureMapToPlane>::New();
        texturePlane->SetInputConnection(normalGenerator->GetOutputPort());

        vtkSmartPointer<vtkPolyDataMapper> terrain_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
        terrain_mapper->SetInputConnection(texturePlane->GetOutputPort());

        // Remove the old terrain actor if it exists already
        if (terrain_actor_ != NULL) {
            renderer_->RemoveActor(terrain_actor_);
        }
        terrain_actor_ = vtkSmartPointer<vtkActor>::New();
        terrain_actor_->SetMapper(terrain_mapper);
        terrain_actor_->SetTexture(terrain_texture);
        terrain_actor_->GetProperty()->SetColor(0, 0, 0);

        renderer_->AddActor(terrain_actor_);
    }
    return true;
}

void Updater::set_max_update_rate(double max_update_rate)
{ max_update_rate_ = max_update_rate; }

void Updater::set_renderer(vtkSmartPointer<vtkRenderer> &renderer) {
    renderer_ = renderer;

    // After the renderer is set, we can setup the text display
    create_text_display();
    set_view_mode(ViewMode::FOLLOW);
}

void Updater::set_rwi(vtkSmartPointer<vtkRenderWindowInteractor> &rwi)
{ rwi_ = rwi; }

void Updater::set_incoming_interface(InterfacePtr &incoming_interface)
{ incoming_interface_ = incoming_interface; }

void Updater::set_outgoing_interface(InterfacePtr &outgoing_interface)
{ outgoing_interface_ = outgoing_interface; }

bool Updater::update_contacts(std::shared_ptr<scrimmage_proto::Frame> &frame) {
    frame_time_ = frame->time();

    // Add new contacts to contact map
    for (int i = 0; i < frame->contact_size(); i++) {

        const sp::Contact cnt = frame->contact(i);
        int id = cnt.id().id();

        if (actor_contacts_.count(id) == 0 && cnt.active()) {
            // Initialize everything as a sphere until it can be matched
            // with information in the contact_visuals_ map
            vtkSmartPointer<vtkSphereSource> sphereSource =
                vtkSmartPointer<vtkSphereSource>::New();
            sphereSource->SetCenter(0, 0, 0);
            sphereSource->SetRadius(1);

            // Create a mapper
            vtkSmartPointer<vtkDataSetMapper> mapper =
                vtkSmartPointer<vtkDataSetMapper>::New();
            mapper->SetInputConnection(sphereSource->GetOutputPort());

            // Create an actor
            vtkSmartPointer<vtkActor> actor =
                vtkSmartPointer<vtkActor>::New();
            actor->SetMapper(mapper);

            actor->GetProperty()->SetColor(161, 161, 161);
            actor->GetProperty()->SetOpacity(0.25);

            renderer_->AddActor(actor);

            // Add the object label
            vtkSmartPointer<vtkVectorText> textSource =
                vtkSmartPointer<vtkVectorText>::New();
            textSource->SetText(std::to_string(id).c_str());

            // Create a mapper for the label
            vtkSmartPointer<vtkPolyDataMapper> label_mapper =
                vtkSmartPointer<vtkPolyDataMapper>::New();
            label_mapper->SetInputConnection(textSource->GetOutputPort());

            // Create a subclass of vtkActor: a vtkFollower that remains facing the camera
            vtkSmartPointer<vtkFollower> label =
                vtkSmartPointer<vtkFollower>::New();
            label->SetMapper(label_mapper);
            label->GetProperty()->SetColor(1, 1, 1); // white
            label->SetScale(label_scale_, label_scale_, label_scale_);

            // Add the actor to the scene
            // renderer_->AddActor(actor);
            renderer_->AddActor(label);

            // Save a reference to the actor for modifying later
            std::shared_ptr<ActorContact> actor_contact = std::make_shared<ActorContact>();
            actor_contact->actor = actor;

            sc::set(actor_contact->color, 161, 161, 161);
            actor_contact->contact = cnt;
            actor_contact->label = label;
            actor_contact->exists = true;
            actor_contact->remove = false;
            actor_contacts_[id] = actor_contact;
        }
    }

    // Update contacts in contact map
    for (int i = 0; i < frame->contact_size(); i++) {
        const sp::Contact cnt = frame->contact(i);
        int id = cnt.id().id();

        if (actor_contacts_.count(id) > 0) {
            // Update an existing contact
            std::shared_ptr<ActorContact> ac = actor_contacts_[id];
            ac->exists = true;
            ac->contact = cnt;

            // Update the visuals for the actor, if neccessary
            auto it = contact_visuals_.find(id);
            if (it != contact_visuals_.end() && it->second->update_required()) {
                update_contact_visual(ac, it->second);
                it->second->set_update_required(false);
            }

            double x_pos = cnt.state().position().x();
            double y_pos = cnt.state().position().y();
            double z_pos = cnt.state().position().z();

            ac->actor->SetPosition(x_pos, y_pos, z_pos);

            sp::Quaternion sp_quat = cnt.state().orientation();
            sc::Quaternion quat(sp_quat.w(), sp_quat.x(), sp_quat.y(), sp_quat.z());
            Eigen::Matrix3d mat3 = quat.toRotationMatrix();

            double desired_scale_amount = 1.0;
            auto it_cv = contact_visuals_.find(id);
            if (it_cv != contact_visuals_.end()) {
                desired_scale_amount = it_cv->second->scale() * scale_;
            }

            vtkMatrix4x4 *m;
            m = ac->actor->GetMatrix();

            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    m->SetElement(r, c, desired_scale_amount*mat3(r, c));
                }
            }

            // Tells each label to "face" the camera
            ac->label->SetCamera(renderer_->GetActiveCamera());
            ac->label->SetPosition(x_pos, y_pos, z_pos + 0.1);

            update_trail(ac, x_pos, y_pos, z_pos);
        }
    }

    // Set opacity of stale actors
    for (auto &kv : actor_contacts_) {
        if (!kv.second->exists) {
            kv.second->actor->GetProperty()->SetOpacity(0.10);
            kv.second->label->GetProperty()->SetOpacity(0.10);
        }
        kv.second->exists = false;
    }

    // Remove actors that had an inactive tag in their message
    auto it = actor_contacts_.begin();
    while (it != actor_contacts_.end()) {
        if (it->second->remove) {
            // Remove actor and label from viewer
            renderer_->RemoveActor(it->second->actor);
            renderer_->RemoveActor(it->second->label);

            // Remove the trail points
            for (std::list<vtkSmartPointer<vtkActor> >::iterator it_trail = it->second->trail.begin();
                 it_trail != it->second->trail.end(); ++it_trail) {
                renderer_->RemoveActor(*it_trail);
            }
            actor_contacts_.erase(it++); // remove map entry
        } else {
            it++;
        }
    }

    return true;
}

void Updater::update_contact_visual(std::shared_ptr<ActorContact> &actor_contact,
                                    std::shared_ptr<scrimmage_proto::ContactVisual> &cv) {
    // Only update the meshes if the model name has changed
    if (actor_contact->model_name != cv->name()) {
        actor_contact->model_name = cv->name();

        // Remove the old actor:
        renderer_->RemoveActor(actor_contact->actor);

        // Create a new actor and mapper
        vtkSmartPointer<vtkDataSetMapper> mapper =
            vtkSmartPointer<vtkDataSetMapper>::New();

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();

        // Search for the appropriate files on the local system
        ConfigParse cv_parse;
        FileSearch file_search;
        bool mesh_found, texture_found;

        // Setup the overrides based on the data in the message
        std::map<std::string, std::string> overrides;
        overrides["visual_scale"] = std::to_string(cv->scale());
        overrides["visual_rpy"] = std::to_string(cv->rotate(0)) + " " +
            std::to_string(cv->rotate(1)) + " " +
            std::to_string(cv->rotate(2));

        find_model_properties(cv->name(), cv_parse, file_search, overrides,
                              cv, mesh_found, texture_found);

        if (actor_contact->contact.type() == sp::MESH) {
            if (texture_found) {
                vtkSmartPointer<vtkPNGReader> pngReader =
                    vtkSmartPointer<vtkPNGReader>::New();
                pngReader->SetFileName(cv->texture_file().c_str());
                pngReader->Update();

                vtkSmartPointer<vtkTexture> colorTexture =
                    vtkSmartPointer<vtkTexture>::New();
                colorTexture->SetInputConnection(pngReader->GetOutputPort());
                colorTexture->InterpolateOn();
                actor->SetTexture(colorTexture);
            }

            vtkSmartPointer<vtkOBJReader> reader =
                vtkSmartPointer<vtkOBJReader>::New();
            reader->SetFileName(cv->model_file().c_str());
            reader->Update();

            vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

            // transform->RotateWXYZ(cv->rotate(0), cv->rotate(1),
            //                      cv->rotate(2), cv->rotate(3));
            transform->RotateX(cv->rotate(0));
            transform->RotateY(cv->rotate(1));
            transform->RotateZ(cv->rotate(2));

            vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
                vtkSmartPointer<vtkTransformPolyDataFilter>::New();

            transformFilter->SetTransform(transform);
            transformFilter->SetInputConnection(reader->GetOutputPort());
            transformFilter->Update();

            mapper->SetInputConnection(transformFilter->GetOutputPort());

            // Need to scale contact to current GUI scale:
            double scale_data[3] = {0, 0, 0};
            for (int i = 0; i < 3; i++) {
                scale_data[i] = cv->scale() * scale_;
            }
            actor->SetScale(scale_data[0], scale_data[1],
                            scale_data[2]);
        } else if (actor_contact->contact.type() == sp::AIRCRAFT) {
            vtkSmartPointer<vtkPoints> points =
                vtkSmartPointer<vtkPoints>::New();

            float size = 2.0;
            float offset = -size/2;
            float p0[3] = {offset, 0, size /4};
            float p1[3] = {offset, 0, size /4};
            float p2[3] = {offset, -2 * size / 3, 0.0};
            float p3[3] = {offset, 2 * size / 3, 0};
            float p4[3] = {size*2+offset, 0.0, 0.0};

            points->InsertNextPoint(p0);
            points->InsertNextPoint(p1);
            points->InsertNextPoint(p2);
            points->InsertNextPoint(p3);
            points->InsertNextPoint(p4);

            vtkSmartPointer<vtkPyramid> pyramid =
                vtkSmartPointer<vtkPyramid>::New();
            pyramid->GetPointIds()->SetId(0, 0);
            pyramid->GetPointIds()->SetId(1, 1);
            pyramid->GetPointIds()->SetId(2, 2);
            pyramid->GetPointIds()->SetId(3, 3);
            pyramid->GetPointIds()->SetId(4, 4);

            vtkSmartPointer<vtkCellArray> cells =
                vtkSmartPointer<vtkCellArray>::New();
            cells->InsertNextCell(pyramid);

            vtkSmartPointer<vtkUnstructuredGrid> ug =
                vtkSmartPointer<vtkUnstructuredGrid>::New();
            ug->SetPoints(points);
            ug->InsertNextCell(pyramid->GetCellType(), pyramid->GetPointIds());
#if VTK_MAJOR_VERSION <= 5
            mapper->SetInput(ug);
#else
            mapper->SetInputData(ug);
#endif

        } else if (actor_contact->contact.type() == sp::SPHERE) {
            vtkSmartPointer<vtkSphereSource> sphereSource =
                vtkSmartPointer<vtkSphereSource>::New();
            sphereSource->SetCenter(0, 0, 0);
            sphereSource->SetRadius(1);

            mapper->SetInputConnection(sphereSource->GetOutputPort());

        } else {
            vtkSmartPointer<vtkSphereSource> sphereSource =
                vtkSmartPointer<vtkSphereSource>::New();
            sphereSource->SetCenter(0, 0, 0);
            sphereSource->SetRadius(1);

            mapper->SetInputConnection(sphereSource->GetOutputPort());
        }

        actor->SetMapper(mapper);
        renderer_->AddActor(actor);
        actor_contact->actor = actor;
    }

    actor_contact->actor->GetProperty()->SetOpacity(cv->opacity());
    actor_contact->label->GetProperty()->SetOpacity(cv->opacity());

    if (cv->visual_mode() == sp::ContactVisual::COLOR) {
        actor_contact->actor->GetProperty()->SetColor(cv->color().r()/255.0,
                                                      cv->color().g()/255.0,
                                                      cv->color().b()/255.0);
    }

    sc::set(actor_contact->color, cv->color());
}

void Updater::update_trail(std::shared_ptr<ActorContact> &actor_contact,
                           double &x_pos, double &y_pos, double &z_pos) {
    if (enable_trails_) {
        /////////////////////
        // Create the geometry of a point (the coordinate)
        vtkSmartPointer<vtkPoints> points =
            vtkSmartPointer<vtkPoints>::New();
        const float p[3] =
            {static_cast<float>(x_pos),
             static_cast<float>(y_pos),
             static_cast<float>(z_pos)};

        // Create the topology of the point (a vertex)
        vtkSmartPointer<vtkCellArray> vertices =
            vtkSmartPointer<vtkCellArray>::New();
        vtkIdType pid[1];
        pid[0] = points->InsertNextPoint(p);
        vertices->InsertNextCell(1, pid);

        // Create a polydata object
        vtkSmartPointer<vtkPolyData> point =
            vtkSmartPointer<vtkPolyData>::New();

        // Set the points and vertices we created as the geometry and topology of the polydata
        point->SetPoints(points);
        point->SetVerts(vertices);

        // Visualize
        vtkSmartPointer<vtkPolyDataMapper> points_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();

#if VTK_MAJOR_VERSION < 6
        points_mapper->SetInput(point);
#else
        points_mapper->SetInputData(point);
#endif

        vtkSmartPointer<vtkActor> points_actor =
            vtkSmartPointer<vtkActor>::New();
        points_actor->SetMapper(points_mapper);
        points_actor->GetProperty()->SetPointSize(5);

        points_actor->GetProperty()->SetColor(actor_contact->color.r()/255.0,
                                              actor_contact->color.g()/255.0,
                                              actor_contact->color.b()/255.0);

        renderer_->AddActor(points_actor);

        // Save the points actor, so that it can be removed
        // later.
        actor_contact->trail.push_back(points_actor);
        if (actor_contact->trail.size() > 20) {
            renderer_->RemoveActor(actor_contact->trail.front());
            actor_contact->trail.pop_front();
        }
    } else {
        // Remove the trails
        for (vtkSmartPointer<vtkActor> actor : actor_contact->trail) {
            renderer_->RemoveActor(actor);
        }
        actor_contact->trail.clear();
    }
}

void Updater::process_custom_key(std::string &key) {
    gui_msg_.set_custom_key(key);
    outgoing_interface_->send_gui_msg(gui_msg_);
    gui_msg_.set_custom_key("");
}

void Updater::inc_follow() {
    inc_follow_ = true;
}

void Updater::dec_follow() {
    dec_follow_ = true;
}

void Updater::toggle_trails() {
    enable_trails_ = !enable_trails_;
}

void Updater::inc_warp() {
    gui_msg_.set_inc_warp(true);
    outgoing_interface_->send_gui_msg(gui_msg_);
    gui_msg_.set_inc_warp(false);
}

void Updater::dec_warp() {
    gui_msg_.set_dec_warp(true);
    outgoing_interface_->send_gui_msg(gui_msg_);
    gui_msg_.set_dec_warp(false);
}

void Updater::toggle_pause() {
    gui_msg_.set_toggle_pause(true);
    outgoing_interface_->send_gui_msg(gui_msg_);
    gui_msg_.set_toggle_pause(false);
}

void Updater::single_step() {
    gui_msg_.set_single_step(true);
    outgoing_interface_->send_gui_msg(gui_msg_);
    gui_msg_.set_single_step(false);
}

void Updater::request_cached() {
    gui_msg_.set_request_cached(true);
    outgoing_interface_->send_gui_msg(gui_msg_);
    gui_msg_.set_request_cached(false);
}

void Updater::world_point_clicked(const double &x, const double &y,
                                  const double &z) {
    scrimmage_proto::WorldPointClicked msg;
    sc::set(msg.mutable_point(), x, y, z);
    msg.set_name("WorldPointClicked");
    outgoing_interface_->send_world_point_clicked_msg(msg);
}

void Updater::toggle_helpmenu() {
    std::stringstream stream_helpkeys, stream_helpvalues;
        show_helpmenu_ = !show_helpmenu_;
    if (show_helpmenu_) {
        stream_helpkeys
            << "q\n"
            << "b\n"
            << "space\n"
            << "a\n"
            << "A\n"
            << "right/left arrows\n"
            << "[\n"
            << "]\n"
            << "+\n"
            << "-\n"
            << "N\n"
            << "n\n"
            << "r\n"
            << "scroll\n"
            << "z\n"
            << "SHIFT + z\n"
            << "w\n"
            << "s\n"
            << "CTRL + left click\n"
            << "SHIFT + left click\n"
            << ";\n";
        helpkeys_actor_->SetInput(stream_helpkeys.str().c_str());
        stream_helpvalues
            << ": quit\n"
            << ": pause/unpause\n"
            << ": step sim (paused only)\n"
            << ": cycle camera views\n"
            << ": free camera view\n"
            << ": change aircraft\n"
            << ": decrease warp speed\n"
            << ": increase warp speed\n"
            << ": increase visual scale\n"
            << ": decrease visual scale\n"
            << ": increase entity label scale\n"
            << ": decrease entity label scale\n"
            << ": reset scale/camera\n"
            << ": zoom\n"
            << ": zoom out\n"
            << ": zoom in\n"
            << ": wireframe\n"
            << ": solid\n"
            << ": rotate world\n"
            << ": pan camera\n"
            << ": enter custom keypress\n";
        helpvalues_actor_->SetInput(stream_helpvalues.str().c_str());
    } else {
        stream_helpkeys << " ";
        stream_helpvalues << " ";
        helpkeys_actor_->SetInput(stream_helpkeys.str().c_str());
        helpvalues_actor_->SetInput(stream_helpvalues.str().c_str());
    }
}

void Updater::shutting_down() {
    gui_msg_.set_shutting_down(true);
    if (send_shutdown_msg_) {
        outgoing_interface_->send_gui_msg(gui_msg_);
    }
}

void Updater::inc_scale() {
    scale_ *= 2.0;
    scale_required_ = true;
}

void Updater::dec_scale() {
    scale_ *= 0.5;
    scale_required_ = true;
}

void Updater::inc_label_scale() {
    label_scale_ *= 2.0;
    scale_required_ = true;
}

void Updater::dec_label_scale() {
    label_scale_ *= 0.5;
    scale_required_ = true;
}

void Updater::inc_follow_offset() {follow_offset_ *= 1.1;}

void Updater::dec_follow_offset() {follow_offset_ /= 1.1;}

void Updater::reset_scale() {
    scale_ = 1.0;
    scale_required_ = true;
}

void Updater::set_reset_camera() {
    reset_camera_ = true;
    set_view_mode(ViewMode::FREE);
}

void Updater::set_follow_id(int follow_id) {
    follow_id_ = follow_id;
}

void Updater::set_show_fps(bool show_fps) {
    show_fps_ = false;
}

void Updater::set_camera_reset_params(double pos_x, double pos_y, double pos_z,
                                      double focal_x, double focal_y, double focal_z) {
    camera_reset_params_.pos_x = pos_x;
    camera_reset_params_.pos_y = pos_y;
    camera_reset_params_.pos_z = pos_z;
    camera_reset_params_.focal_x = focal_x;
    camera_reset_params_.focal_y = focal_y;
    camera_reset_params_.focal_z = focal_z;
}

void Updater::reset_view() {
    reset_scale();
    set_view_mode(ViewMode::FREE);
}

void Updater::create_text_display() {
    int text_y_spacing = 30;
    int text_y = 10;
    int text_x = 10;

    // Add the time (text) display
    time_actor_ = vtkSmartPointer<vtkTextActor>::New();
    time_actor_->SetInput("000.000 s");
    time_actor_->SetPosition(text_x, text_y);
    time_actor_->GetTextProperty()->SetFontSize(24);
    time_actor_->GetTextProperty()->SetColor(1.0, 1.0, 1.0);
    renderer_->AddActor2D(time_actor_);
    text_y += text_y_spacing;

    // Add the view mode (text) display
    view_mode_actor_ = vtkSmartPointer<vtkTextActor>::New();
    view_mode_actor_->SetInput("View: Follow");
    view_mode_actor_->SetPosition(text_x, text_y);
    view_mode_actor_->GetTextProperty()->SetFontSize(24);
    view_mode_actor_->GetTextProperty()->SetColor(1.0, 1.0, 1.0);
    renderer_->AddActor2D(view_mode_actor_);
    text_y += text_y_spacing;

    // Add the heading (text) display
    heading_actor_ = vtkSmartPointer<vtkTextActor>::New();
    heading_actor_->SetInput("H: 360.00");
    heading_actor_->SetPosition(text_x, text_y);
    heading_actor_->GetTextProperty()->SetFontSize(24);
    heading_actor_->GetTextProperty()->SetColor(1.0, 1.0, 1.0);
    renderer_->AddActor2D(heading_actor_);
    text_y += text_y_spacing;

    // Add the alt (text) display
    alt_actor_ = vtkSmartPointer<vtkTextActor>::New();
    alt_actor_->SetInput("Alt: 360.00");
    alt_actor_->SetPosition(text_x, text_y);
    alt_actor_->GetTextProperty()->SetFontSize(24);
    alt_actor_->GetTextProperty()->SetColor(1.0, 1.0, 1.0);
    renderer_->AddActor2D(alt_actor_);

    // Add the help menu
    // NOTE: this requires two vtkTextActor's because you can't
    // get the text to align by ':' otherwise
    helpkeys_actor_ = vtkSmartPointer<vtkTextActor>::New();
    helpkeys_actor_->SetInput(" ");
    helpkeys_actor_->SetPosition(text_x, 200);
    helpkeys_actor_->GetTextProperty()->SetFontSize(14);
    helpkeys_actor_->GetTextProperty()->SetColor(1.0, 1.0, 1.0);
    renderer_->AddActor2D(helpkeys_actor_);
    // Add helpmenu values (descriptions)
    helpvalues_actor_ = vtkSmartPointer<vtkTextActor>::New();
    helpvalues_actor_->SetInput(" ");
    helpvalues_actor_->SetPosition(text_x + 120, 200);
    helpvalues_actor_->GetTextProperty()->SetFontSize(14);
    helpvalues_actor_->GetTextProperty()->SetColor(1.0, 1.0, 1.0);
    renderer_->AddActor2D(helpvalues_actor_);

    text_x = 300;

    // Add the warp (text) display
    warp_actor_ = vtkSmartPointer<vtkTextActor>::New();
    warp_actor_->SetInput("50.00 X");
    warp_actor_->SetPosition(text_x, 10);
    warp_actor_->GetTextProperty()->SetFontSize(24);
    warp_actor_->GetTextProperty()->SetColor(1.0, 1.0, 1.0);
    renderer_->AddActor2D(warp_actor_);

    text_x += 100;

    // Add the fps (text) display
    fps_actor_ = vtkSmartPointer<vtkTextActor>::New();
    fps_actor_->SetInput("FPS: 60.0");
    fps_actor_->SetPosition(text_x, 10);
    fps_actor_->GetTextProperty()->SetFontSize(24);
    fps_actor_->GetTextProperty()->SetColor(1.0, 1.0, 1.0);
    renderer_->AddActor2D(fps_actor_);
}

void Updater::enable_fps() {
    // Set FPS callback:
    vtkSmartPointer<vtkCallbackCommand> callback =
        vtkSmartPointer<vtkCallbackCommand>::New();

    callback->SetCallback(fpsCallbackFunction);
    renderer_->AddObserver(vtkCommand::EndEvent, callback);
}

void Updater::quat_2_transform(const sc::Quaternion &q,
                               vtkSmartPointer<vtkTransform> transform) {
    transform->RotateX(sc::Angles::rad2deg(q.roll()));
    transform->RotateY(sc::Angles::rad2deg(q.pitch()));
    transform->RotateZ(sc::Angles::rad2deg(q.yaw()));
}

bool Updater::draw_triangle(const bool &new_shape,
                            const scrimmage_proto::Triangle &t,
                            vtkSmartPointer<vtkActor> &actor,
                            vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                            vtkSmartPointer<vtkPolyDataMapper> &mapper) {
    vtkSmartPointer<vtkPoints> points =
        vtkSmartPointer<vtkPoints>::New();

    points->InsertNextPoint(t.point0().x(),
                            t.point0().y(),
                            t.point0().z());

    points->InsertNextPoint(t.point1().x(),
                            t.point1().y(),
                            t.point1().z());

    points->InsertNextPoint(t.point2().x(),
                            t.point2().y(),
                            t.point2().z());

    vtkSmartPointer<vtkTriangle> triangle =
        vtkSmartPointer<vtkTriangle>::New();
    triangle->GetPointIds()->SetId(0, 0);
    triangle->GetPointIds()->SetId(1, 1);
    triangle->GetPointIds()->SetId(2, 2);

    vtkSmartPointer<vtkCellArray> triangles =
        vtkSmartPointer<vtkCellArray>::New();
    triangles->InsertNextCell(triangle);

    vtkSmartPointer<vtkPolyData> trianglePolyData;
    if (new_shape) {
        trianglePolyData = vtkSmartPointer<vtkPolyData>::New();
        // source = trianglePolyData;
    } else {
        trianglePolyData = vtkPolyData::SafeDownCast(source);
    }

    // Add the geometry and topology to the polydata
    trianglePolyData->SetPoints(points);
    trianglePolyData->SetPolys(triangles);
#if VTK_MAJOR_VERSION < 6
    mapper->SetInput(trianglePolyData);
#else
    mapper->SetInputData(trianglePolyData);
#endif
    actor->SetMapper(mapper);

    return true;
}

bool Updater::draw_arrow(const bool &new_shape,
                         const scrimmage_proto::Arrow &a,
                         vtkSmartPointer<vtkActor> &actor,
                         vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                         vtkSmartPointer<vtkPolyDataMapper> &mapper) {
    if (new_shape) {
        // Create an arrow.
        vtkSmartPointer<vtkArrowSource> arrowSource =
            vtkSmartPointer<vtkArrowSource>::New();

        double startPoint[3] = {a.tail().x(), a.tail().y(), a.tail().z()};
        double endPoint[3] = {a.head().x(), a.head().y(), a.head().z()};

        // Compute a basis
        double normalizedX[3] = {0.0, 0.0, 0.0};
        double normalizedY[3] = {0.0, 0.0, 0.0};
        double normalizedZ[3] = {0.0, 0.0, 0.0};

        // The X axis is a vector from start to end
        vtkMath::Subtract(endPoint, startPoint, normalizedX);
        double length = vtkMath::Norm(normalizedX);
        vtkMath::Normalize(normalizedX);

        // The Z axis is an arbitrary vector cross X
        double arbitrary[3] = {1.0, 2.0, 3.0};
        vtkMath::Cross(normalizedX, arbitrary, normalizedZ);
        vtkMath::Normalize(normalizedZ);

        // The Y axis is Z cross X
        vtkMath::Cross(normalizedZ, normalizedX, normalizedY);
        vtkSmartPointer<vtkMatrix4x4> matrix =
            vtkSmartPointer<vtkMatrix4x4>::New();

        // Create the direction cosine matrix
        matrix->Identity();
        for (unsigned int i = 0; i < 3; i++) {
            matrix->SetElement(i, 0, normalizedX[i]);
            matrix->SetElement(i, 1, normalizedY[i]);
            matrix->SetElement(i, 2, normalizedZ[i]);
        }

        // Apply the transforms
        vtkSmartPointer<vtkTransform> transform =
            vtkSmartPointer<vtkTransform>::New();
        transform->Translate(startPoint);
        transform->Concatenate(matrix);
        transform->Scale(length, length, length);

        // Transform the polydata
        vtkSmartPointer<vtkTransformPolyDataFilter> transformPD =
            vtkSmartPointer<vtkTransformPolyDataFilter>::New();
        transformPD->SetTransform(transform);
        transformPD->SetInputConnection(arrowSource->GetOutputPort());

#ifdef USER_MATRIX
        mapper->SetInputConnection(arrowSource->GetOutputPort());
        actor->SetUserMatrix(transform->GetMatrix());
#else
        mapper->SetInputConnection(transformPD->GetOutputPort());
#endif
        actor->SetMapper(mapper);
    }

    return true;
}

bool Updater::draw_cone(const bool &new_shape,
                        const scrimmage_proto::Cone &c,
                        vtkSmartPointer<vtkActor> &actor,
                        vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                        vtkSmartPointer<vtkPolyDataMapper> &mapper) {
    vtkSmartPointer<vtkConeSource> coneSource;
    if (new_shape) {
        coneSource = vtkSmartPointer<vtkConeSource>::New();
        source = coneSource;
        mapper->SetInputConnection(coneSource->GetOutputPort());
        actor->SetMapper(mapper);
    } else {
        coneSource = vtkConeSource::SafeDownCast(source);
    }

    coneSource->SetRadius(c.base_radius());
    coneSource->SetHeight(c.height());
    coneSource->SetResolution(32);

    Eigen::Vector3d cone_dir = sc::eigen(c.direction());
    Eigen::Vector3d apex_shift = sc::eigen(c.apex()) + cone_dir.normalized()*c.height()/2.0;
    Eigen::Vector3d dir_shift = -cone_dir;
    coneSource->SetCenter(apex_shift(0), apex_shift(1), apex_shift(2));
    coneSource->SetDirection(dir_shift(0), dir_shift(1), dir_shift(2));
    coneSource->Update();

    return true;
}

bool Updater::draw_line(const bool &new_shape,
                        const scrimmage_proto::Line &l,
                        vtkSmartPointer<vtkActor> &actor,
                        vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                        vtkSmartPointer<vtkPolyDataMapper> &mapper) {
    vtkSmartPointer<vtkLineSource> lineSource;
    if (new_shape) {
        lineSource = vtkSmartPointer<vtkLineSource>::New();
        source = lineSource;
        mapper->SetInputConnection(lineSource->GetOutputPort());
        actor->SetMapper(mapper);
        actor->GetProperty()->SetLineWidth(2);
    } else {
        lineSource = vtkLineSource::SafeDownCast(source);
    }
    lineSource->SetPoint1(l.start().x(), l.start().y(),
                          l.start().z());
    lineSource->SetPoint2(l.end().x(), l.end().y(),
                          l.end().z());
    lineSource->Update();
    return true;
}

bool Updater::draw_polygon(const bool &new_shape,
                           const scrimmage_proto::Polygon &p,
                           vtkSmartPointer<vtkActor> &actor,
                           vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                           vtkSmartPointer<vtkPolyDataMapper> &mapper) {
    if (new_shape) {
        // Setup points
        vtkSmartPointer<vtkPoints> points =
            vtkSmartPointer<vtkPoints>::New();

        // Create the polygon
        vtkSmartPointer<vtkPolygon> polygon =
            vtkSmartPointer<vtkPolygon>::New();

        polygon->GetPointIds()->SetNumberOfIds(p.point_size());

        for (int i = 0; i < p.point_size(); i++) {
            points->InsertNextPoint(p.point(i).x(), p.point(i).y(),
                                    p.point(i).z());
            polygon->GetPointIds()->SetId(i, i);
        }

        // Add the polygon to a list of polygons
        vtkSmartPointer<vtkCellArray> polygons =
            vtkSmartPointer<vtkCellArray>::New();
        polygons->InsertNextCell(polygon);

        // Create a PolyData
        vtkSmartPointer<vtkPolyData> polygonPolyData =
            vtkSmartPointer<vtkPolyData>::New();
        polygonPolyData->SetPoints(points);
        polygonPolyData->SetPolys(polygons);
#if VTK_MAJOR_VERSION < 6
        mapper->SetInput(polygonPolyData);
#else
        mapper->SetInputData(polygonPolyData);
#endif

        actor->SetMapper(mapper);
        actor->GetProperty()->SetLineWidth(1);
    }
    return true;
}

bool Updater::draw_polydata(const bool &new_shape,
                            const scrimmage_proto::Polydata &p,
                            vtkSmartPointer<vtkActor> &actor,
                            vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                            vtkSmartPointer<vtkPolyDataMapper> &mapper) {
    if (new_shape) {
        vtkSmartPointer<vtkPoints> points =
            vtkSmartPointer<vtkPoints>::New();

        for (int i = 0; i < p.point_size(); i++) {
            points->InsertNextPoint(p.point(i).x(), p.point(i).y(),
                                    p.point(i).z());
        }

        vtkSmartPointer<vtkPolyData> polyData =
            vtkSmartPointer<vtkPolyData>::New();
        polyData->SetPoints(points);

        vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
            vtkSmartPointer<vtkVertexGlyphFilter>::New();

#if VTK_MAJOR_VERSION < 6
        glyphFilter->SetInput(polyData);
#else
        glyphFilter->SetInputData(polyData);
#endif
        glyphFilter->Update();

        mapper->SetInputConnection(glyphFilter->GetOutputPort());
        actor->SetMapper(mapper);
        actor->GetProperty()->SetLineWidth(1);
    }

    return true;
}

bool Updater::draw_pointcloud(const bool &new_shape,
                              const scrimmage_proto::Shape &shape,
                              vtkSmartPointer<vtkActor> &actor,
                              vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                              vtkSmartPointer<vtkPolyDataMapper> &mapper) {
    if (new_shape) {
        const scrimmage_proto::PointCloud &pc = shape.pointcloud();

        vtkSmartPointer<vtkPoints> points =
            vtkSmartPointer<vtkPoints>::New();

        for (int i = 0; i < pc.point_size(); i++) {
            points->InsertNextPoint(pc.point(i).x(), pc.point(i).y(),
                                    pc.point(i).z());
        }

        vtkSmartPointer<vtkUnsignedCharArray> color_array =
            vtkSmartPointer<vtkUnsignedCharArray>::New();
        color_array->SetName("Colors");
        color_array->SetNumberOfComponents(3);

        if (pc.point_size() != pc.color_size()) {
            // use same color if the colors and points aren't the same size
            unsigned char c[3] = {(unsigned char)(shape.color().r()),
                                  (unsigned char)(shape.color().g()),
                                  (unsigned char)(shape.color().b())};
            for (int i = 0; i < pc.point_size(); i++) {
#if VTK_MAJOR_VERSION <= 6
                color_array->InsertNextTupleValue(c);
#else
                color_array->InsertNextTypedTuple(c);
#endif
            }
        } else {
            // Use the color vector if it's the same size as the points
            for (int i = 0; i < pc.color_size(); i++) {
                unsigned char c[3] = {(unsigned char)(pc.color(i).r()),
                                      (unsigned char)(pc.color(i).g()),
                                      (unsigned char)(pc.color(i).b())};
#if VTK_MAJOR_VERSION <= 6
                color_array->InsertNextTupleValue(c);
#else
                color_array->InsertNextTypedTuple(c);
#endif
            }
        }

        vtkSmartPointer<vtkPolyData> pointsPolydata =
            vtkSmartPointer<vtkPolyData>::New();

        pointsPolydata->SetPoints(points);

        vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
            vtkSmartPointer<vtkVertexGlyphFilter>::New();
#if VTK_MAJOR_VERSION < 6
        vertexFilter->SetInput(pointsPolydata);
#else
        vertexFilter->SetInputData(pointsPolydata);
#endif

        vertexFilter->Update();

        vtkSmartPointer<vtkPolyData> poly_data =
            vtkSmartPointer<vtkPolyData>::New();
        poly_data->ShallowCopy(vertexFilter->GetOutput());
        poly_data->GetPointData()->SetScalars(color_array);
#if VTK_MAJOR_VERSION < 6
        mapper->SetInput(poly_data);
#else
        mapper->SetInputData(poly_data);
#endif
        actor->SetMapper(mapper);
        actor->GetProperty()->SetPointSize(pc.size());
    }

    return true;
}

bool Updater::draw_plane(const bool &new_shape,
                         const scrimmage_proto::Plane &p,
                         vtkSmartPointer<vtkActor> &actor,
                         vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                         vtkSmartPointer<vtkPolyDataMapper> &mapper) {
  // sanity checks
  if (std::abs(p.x_length()) < std::numeric_limits<double>::epsilon()
      || std::abs(p.y_length()) < std::numeric_limits<double>::epsilon()) {
    std::cout << "Cannot draw plane: bad dimensions (" << p.x_length() << ", " << p.y_length() << ")\n";
    return false;
  }
  // Load texture
  std::string texture_file, texture_name;
  texture_name = p.texture();
  bool texture_found = false;
  if (!texture_name.empty()) {
    ConfigParse c_parse;
    FileSearch file_search;
    std::map<std::string, std::string> overrides;
    if (c_parse.parse(overrides, p.texture(), "SCRIMMAGE_DATA_PATH", file_search)) {
      texture_file = c_parse.directory() + "/" + c_parse.params()["texture"];
      texture_found = fs::exists(texture_file) && fs::is_regular_file(texture_file);
    }
  }

  vtkSmartPointer<vtkPlaneSource> planeSource;
  vtkSmartPointer<vtkJPEGReader> jPEGReader;
  vtkSmartPointer<vtkTexture> texture;
  vtkSmartPointer<vtkTextureMapToPlane> texturePlane;
  jPEGReader = vtkSmartPointer<vtkJPEGReader>::New();
  texture = vtkSmartPointer<vtkTexture>::New();
  texturePlane = vtkSmartPointer<vtkTextureMapToPlane>::New();
  if (new_shape) {
    planeSource = vtkSmartPointer<vtkPlaneSource>::New();
    source = planeSource;
    texture_found ?
      mapper->SetInputConnection(texturePlane->GetOutputPort()) :
      mapper->SetInputConnection(planeSource->GetOutputPort());
    actor->SetMapper(mapper);
  } else {
    planeSource = vtkPlaneSource::SafeDownCast(source);
  }

  // update properties
  Eigen::Vector3d center, point1, point2;
  scrimmage::Quaternion quat(
      p.quat().w(), p.quat().x(), p.quat().y(), p.quat().z());
  center << p.center().x(), p.center().y(), p.center().z();
  auto x_hat = quat.rotate(Eigen::Vector3d::UnitX());
  auto y_hat = quat.rotate(Eigen::Vector3d::UnitY());

  // vtk's "center" is actually the bottom left corner
  center -= x_hat * (p.x_length() / 2.0) + y_hat * (p.y_length() / 2.0);

  // calculate point1 and point2, which define the plane axes
  point1 = center + x_hat * p.x_length();
  point2 = center + y_hat * p.y_length();
  planeSource->SetOrigin(center[0], center[1], center[2]);
  planeSource->SetPoint1(point1[0], point1[1], point1[2]);
  planeSource->SetPoint2(point2[0], point2[1], point2[2]);

  if (texture_found) {
    actor->SetTexture(texture);
    jPEGReader->SetFileName(texture_file.c_str());
    texture->SetInputConnection(jPEGReader->GetOutputPort());
    texturePlane->SetInputConnection(planeSource->GetOutputPort());
  } else {
    std::cout << "plane texture not found: " << texture_file << std::endl;
  }

  // set ambient lighting for the plane
  if (!p.diffuse_lighting()) {
    actor->GetProperty()->SetAmbient(1);
    actor->GetProperty()->SetDiffuse(0);
  }

  return true;
}

bool Updater::draw_cube(const bool &new_shape,
                        const scrimmage_proto::Cuboid &c,
                        vtkSmartPointer<vtkActor> &actor,
                        vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                        vtkSmartPointer<vtkPolyDataMapper> &mapper) {
    vtkSmartPointer<vtkCubeSource> cubeSource;
    if (new_shape) {
        cubeSource = vtkSmartPointer<vtkCubeSource>::New();
        source = cubeSource;
        mapper->SetInputConnection(cubeSource->GetOutputPort());
        actor->SetMapper(mapper);
    } else {
        cubeSource = vtkCubeSource::SafeDownCast(source);
    }

    cubeSource->SetCenter(0, 0, 0); // Place cube at center of actor
    cubeSource->SetXLength(c.x_length());
    cubeSource->SetYLength(c.y_length());
    cubeSource->SetZLength(c.z_length());

    // Set the actor's position
    actor->SetPosition(c.center().x(), c.center().y(), c.center().z());

    // Rotate the actor
    Quaternion quat = proto_2_quat(c.quat());
    actor->RotateZ(sc::Angles::rad2deg(quat.yaw()));

    return true;
}

bool Updater::draw_sphere(const bool &new_shape,
                          const scrimmage_proto::Sphere &s,
                          vtkSmartPointer<vtkActor> &actor,
                          vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                          vtkSmartPointer<vtkPolyDataMapper> &mapper) {
    vtkSmartPointer<vtkSphereSource> sphereSource;
    if (new_shape) {
        sphereSource = vtkSmartPointer<vtkSphereSource>::New();
        sphereSource->SetCenter(0, 0, 0); // actor is moved later
        sphereSource->SetThetaResolution(100);
        sphereSource->SetPhiResolution(100);
        source = sphereSource;
        mapper->SetInputConnection(sphereSource->GetOutputPort());
        actor->SetMapper(mapper);
    } else {
        sphereSource = vtkSphereSource::SafeDownCast(source);
    }
    sphereSource->SetRadius(s.radius());
    actor->SetPosition(s.center().x(), s.center().y(), s.center().z());
    return true;
}

bool Updater::draw_circle(const bool &new_shape,
                          const scrimmage_proto::Circle &c,
                          vtkSmartPointer<vtkActor> &actor,
                          vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                          vtkSmartPointer<vtkPolyDataMapper> &mapper) {
    vtkSmartPointer<vtkRegularPolygonSource> polygonSource;
    if (new_shape) {
        polygonSource = vtkSmartPointer<vtkRegularPolygonSource>::New();
        source = polygonSource;
        // polygonSource->GeneratePolygonOff(); // Uncomment this line to generate only the outline of the circle
        polygonSource->SetNumberOfSides(30);
        mapper->SetInputConnection(polygonSource->GetOutputPort());
        actor->SetMapper(mapper);
    } else {
        polygonSource = vtkRegularPolygonSource::SafeDownCast(source);
    }

    polygonSource->SetRadius(c.radius());
    actor->SetPosition(c.center().x(), c.center().y(), c.center().z());
    return true;
}

bool Updater::draw_ellipse(const bool &new_shape,
                           const scrimmage_proto::Ellipse &elp,
                           vtkSmartPointer<vtkActor> &actor,
                           vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                           vtkSmartPointer<vtkPolyDataMapper> &mapper) {
    vtkSmartPointer<vtkRegularPolygonSource> polygonSource;
    if (new_shape) {
        polygonSource = vtkSmartPointer<vtkRegularPolygonSource>::New();
        source = polygonSource;
        // polygonSource->GeneratePolygonOff(); // Uncomment this line to generate only the outline of the ellipse
        polygonSource->SetNumberOfSides(30);

        mapper->SetInputConnection(polygonSource->GetOutputPort());
        actor->SetMapper(mapper);
    } else {
        polygonSource = vtkRegularPolygonSource::SafeDownCast(source);
    }

    Quaternion quat = proto_2_quat(elp.quat());

    actor->SetPosition(elp.center().x(), elp.center().y(), elp.center().z());
    actor->SetOrientation(sc::Angles::rad2deg(quat.roll()),
                          sc::Angles::rad2deg(quat.pitch()),
                          sc::Angles::rad2deg(quat.yaw()));
    actor->SetScale(elp.x_radius(), elp.y_radius(), 0);

    return true;
}

bool Updater::draw_text(const bool &new_shape,
                        const scrimmage_proto::Text &t,
                        vtkSmartPointer<vtkActor> &actor,
                        vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                        vtkSmartPointer<vtkPolyDataMapper> &mapper) {
    vtkSmartPointer<vtkVectorText> textSource;
    if (new_shape) {
        textSource = vtkSmartPointer<vtkVectorText>::New();
        source = textSource;
        mapper->SetInputConnection(textSource->GetOutputPort());

        // Create a subclass of vtkActor: a vtkFollower that remains facing the
        // camera
        actor = vtkSmartPointer<vtkFollower>::New();
        actor->SetMapper(mapper);
        // We created this actor object in this function, so we can use a
        // static_cast. Need to extract the raw pointer from vtk's smartpointer
        // system first.
        static_cast<vtkFollower*>(&*actor)->SetCamera(renderer_->GetActiveCamera());
    } else {
        textSource = vtkVectorText::SafeDownCast(source);
    }

    if (textSource) {
        if (t.scale() > 0) {
            actor->SetScale(t.scale(), t.scale(), t.scale());
        }

        textSource->SetText(t.text().c_str());
        actor->SetPosition(t.center().x(), t.center().y(),
                           t.center().z());
        return true;
    } else {
        return false;
    }
}

void Updater::get_model_texture(std::string name,
                                std::string& model_file, bool& model_found,
                                std::string& texture_file, bool& texture_found,
                                double& base_scale, Quaternion& base_rot) {
    ConfigParse c_parse;
    FileSearch file_search;
    std::map<std::string, std::string> overrides;

    model_found = false;
    texture_found = false;

    base_scale = 1.0;
    base_rot = Quaternion(0.0, 0.0, 0.0);

    if (c_parse.parse(overrides, name, "SCRIMMAGE_DATA_PATH", file_search)) {
        model_file = c_parse.directory() + "/" + c_parse.params()["model"];
        texture_file = c_parse.directory() + "/" + c_parse.params()["texture"];

        model_found = fs::exists(model_file) && fs::is_regular_file(model_file);
        texture_found = fs::exists(texture_file) && fs::is_regular_file(texture_file);

        auto sc_iter = c_parse.params().find("visual_scale");
        if (sc_iter != c_parse.params().end()) {
            base_scale = std::stod(sc_iter->second);
        }

        auto rpy_iter = c_parse.params().find("visual_rpy");
        if (rpy_iter != c_parse.params().end()) {
            std::vector<double> tf_rpy = {0.0, 0.0, 0.0};
            str2container(rpy_iter->second, " ", tf_rpy, 3);
            for (auto& e : tf_rpy) {
                e = sc::Angles::deg2rad(e);
            }
            base_rot = Quaternion(tf_rpy[0], tf_rpy[1], tf_rpy[2]);
        }
    }
}

bool Updater::draw_mesh(const bool &new_shape,
                        const scrimmage_proto::Mesh &m,
                        vtkSmartPointer<vtkActor> &actor,
                        vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                        vtkSmartPointer<vtkPolyDataMapper> &mapper) {
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter;

    if (new_shape) {
        // get transformFilter hooked up to texture and model
        std::string texture_file = "";
        bool texture_file_found = false;
        std::string model_file = "";
        bool model_file_found = false;

        double base_scale = 1.0;
        Quaternion base_rot(0.0, 0.0, 0.0);

        // get model, texture, and params for a given model name
        get_model_texture(m.name(),
                          model_file, model_file_found,
                          texture_file, texture_file_found,
                          base_scale, base_rot);

        if (!model_file_found) {
            std::cout << "Updater: Couldn't find model for "
                      << m.name() << std::endl;
            return false;
        }

        // use texture if we find it
        if (texture_file_found) {
            auto pngReader = vtkSmartPointer<vtkPNGReader>::New();
            pngReader->SetFileName(texture_file.c_str());
            pngReader->Update();

            auto colorTexture = vtkSmartPointer<vtkTexture>::New();
            colorTexture->SetInputConnection(pngReader->GetOutputPort());
            colorTexture->InterpolateOn();
            actor->SetTexture(colorTexture);
        }

        auto reader = vtkSmartPointer<vtkOBJReader>::New();
        reader->SetFileName(model_file.c_str());
        reader->Update();

        // add transform from model coordinates to normal aircraft rpy and scale
        // based on the rpy and scale in the model config file
        transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
        auto transform = vtkSmartPointer<vtkTransform>::New();
        quat_2_transform(base_rot, transform);
        transform->Scale(base_scale, base_scale, base_scale);
        transformFilter->SetTransform(transform);

        // connect transform filter
        transformFilter->SetInputConnection(reader->GetOutputPort());
        transformFilter->Update();
        source = transformFilter;

        mapper->SetInputConnection(transformFilter->GetOutputPort());
        actor->SetMapper(mapper);
    } else {
        transformFilter = vtkTransformPolyDataFilter::SafeDownCast(source);
    }

    Quaternion quat = proto_2_quat(m.quat());
    actor->SetPosition(m.center().x(), m.center().y(), m.center().z());
    actor->SetOrientation(sc::Angles::rad2deg(quat.roll()),
                          sc::Angles::rad2deg(quat.pitch()),
                          sc::Angles::rad2deg(quat.yaw()));
    auto scale = m.scale() * scale_;
    actor->SetScale(scale, scale, scale);

    return true;
}
} // namespace scrimmage
