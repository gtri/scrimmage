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

#ifndef INCLUDE_SCRIMMAGE_VIEWER_UPDATER_H_
#define INCLUDE_SCRIMMAGE_VIEWER_UPDATER_H_

#include <scrimmage/viewer/Grid.h>
#include <scrimmage/viewer/OriginAxes.h>
#include <scrimmage/network/Interface.h>
#include <scrimmage/proto/Contact.pb.h>
#include <scrimmage/proto/Frame.pb.h>
#include <scrimmage/proto/Color.pb.h>
#include <scrimmage/proto/Visual.pb.h>

#include <time.h>

#include <vtkCommand.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkActor.h>
#include <vtkFollower.h>
#include <vtkTextActor.h>

#include <memory>
#include <limits>
#include <list>
#include <map>
#include <utility>
#include <string>

namespace scrimmage {

class ActorContact {
 public:
    vtkSmartPointer<vtkActor> actor;
    vtkSmartPointer<vtkFollower> label;
    std::list<vtkSmartPointer<vtkActor> > trail;
    scrimmage_proto::Color color;
    scrimmage_proto::Contact contact;
    std::string model_name = "";
    bool exists = true;
    bool remove = false;
};

class Updater : public vtkCommand {
 public:
    enum class ViewMode {
        FOLLOW = 0,
        FREE,
        OFFSET
    };

    static Updater *New();

    Updater();

    virtual void Execute(vtkObject *caller, uint64_t eventId, void * vtkNotUsed(callData));

    void enable_fps();

    bool update();
    bool update_contacts(std::shared_ptr<scrimmage_proto::Frame> &frame);
    bool draw_shapes(scrimmage_proto::Shapes &shapes);
    bool update_scale();
    bool update_camera();
    bool update_text_display();
    bool update_shapes();

    bool update_utm_terrain(std::shared_ptr<scrimmage_proto::UTMTerrain> &utm);

    void set_max_update_rate(double max_update_rate);

    void set_renderer(vtkSmartPointer<vtkRenderer> &renderer);

    void set_rwi(vtkSmartPointer<vtkRenderWindowInteractor> &rwi);

    void set_incoming_interface(InterfacePtr &incoming_interface);

    void set_outgoing_interface(InterfacePtr &outgoing_interface);

    void init();

    void next_mode();

    void inc_follow();
    void dec_follow();

    void toggle_trails();

    void inc_warp();

    void dec_warp();

    void toggle_pause();

    void single_step();

    void request_cached();

    void shutting_down();

    void inc_scale();

    void dec_scale();

    void inc_follow_offset();
    void dec_follow_offset();

    void reset_scale();

    void reset_view();

    void create_text_display();

    void update_trail(std::shared_ptr<ActorContact> &actor_contact,
                      double &x_pos, double &y_pos, double &z_pos);

    void update_contact_visual(std::shared_ptr<ActorContact> &actor_contact,
                               std::shared_ptr<scrimmage_proto::ContactVisual> &cv);

    bool draw_triangle(const scrimmage_proto::Shape &s,
                       vtkSmartPointer<vtkActor> &actor,
                       vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_arrow(const scrimmage_proto::Shape &s,
                    vtkSmartPointer<vtkActor> &actor,
                    vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_cone(const scrimmage_proto::Shape &s,
                   vtkSmartPointer<vtkActor> &actor,
                   vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_line(const scrimmage_proto::Shape &s,
                   vtkSmartPointer<vtkActor> &actor,
                   vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_polygon(const scrimmage_proto::Shape &s,
                      vtkSmartPointer<vtkActor> &actor,
                      vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_polydata(const scrimmage_proto::Shape &s,
                       vtkSmartPointer<vtkActor> &actor,
                       vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_plane(const scrimmage_proto::Shape &s,
                    vtkSmartPointer<vtkActor> &actor,
                    vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_pointcloud(const scrimmage_proto::Shape &s,
                         vtkSmartPointer<vtkActor> &actor,
                         vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_sphere(const scrimmage_proto::Shape &s,
                     vtkSmartPointer<vtkActor> &actor,
                     vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_cube(const scrimmage_proto::Shape &s,
                   vtkSmartPointer<vtkActor> &actor,
                   vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_circle(const scrimmage_proto::Shape &s,
                     vtkSmartPointer<vtkActor> &actor,
                     vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_text(const scrimmage_proto::Shape &s,
                   vtkSmartPointer<vtkActor> &actor,
                   vtkSmartPointer<vtkPolyDataMapper> &mapper);

 protected:
    vtkSmartPointer<vtkRenderWindowInteractor> rwi_;
    vtkSmartPointer<vtkRenderer> renderer_;

    double frame_time_;

    int update_count;
    struct timespec prev_time;
    double max_update_rate_;
    std::shared_ptr<Grid> grid_;
    std::shared_ptr<OriginAxes> origin_axes_;

    InterfacePtr incoming_interface_;
    InterfacePtr outgoing_interface_;

    std::map<int, std::shared_ptr<ActorContact> > actor_contacts_;

    int follow_id_;
    bool inc_follow_;
    bool dec_follow_;

    double scale_;
    bool scale_required_;

    ViewMode view_mode_;
    bool enable_trails_;

    scrimmage_proto::GUIMsg gui_msg_;

    scrimmage_proto::SimInfo sim_info_;

    vtkSmartPointer<vtkTextActor> time_actor_;
    vtkSmartPointer<vtkTextActor> warp_actor_;
    vtkSmartPointer<vtkTextActor> heading_actor_;
    vtkSmartPointer<vtkTextActor> alt_actor_;
    vtkSmartPointer<vtkTextActor> fps_actor_;

    std::map<int, std::shared_ptr<scrimmage_proto::ContactVisual> > contact_visuals_;

    std::list<std::pair<scrimmage_proto::Shape, vtkSmartPointer<vtkActor>>> shapes_;

    std::map<std::string, std::shared_ptr<scrimmage_proto::UTMTerrain> > terrain_map_;
    std::map<std::string, std::shared_ptr<scrimmage_proto::ContactVisual>> contact_visual_map_;

    bool send_shutdown_msg_;

    vtkSmartPointer<vtkActor> terrain_actor_;

    double follow_offset_;
};

} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_VIEWER_UPDATER_H_
