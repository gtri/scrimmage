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

#include <scrimmage/math/Quaternion.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/Contact.pb.h>
#include <scrimmage/proto/Frame.pb.h>
#include <scrimmage/proto/Color.pb.h>
#include <scrimmage/proto/Visual.pb.h>
#include <scrimmage/proto/GUIControl.pb.h>

#include <time.h>

#include <vtkCommand.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkActor.h>
#include <vtkFollower.h>
#include <vtkTextActor.h>
#include <vtkPolyDataAlgorithm.h>

#include <tuple>
#include <memory>
#include <limits>
#include <list>
#include <map>
#include <unordered_map>
#include <utility>
#include <string>

namespace scrimmage {

class Grid;
class OriginAxes;
class Interface;
using InterfacePtr = std::shared_ptr<Interface>;

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

struct CameraResetParams {
 public:
    double pos_x = 0;
    double pos_y = 0;
    double pos_z = 200;
    double focal_x = 0;
    double focal_y = 0;
    double focal_z = 0;
};

class Updater : public vtkCommand {
 public:
    // codechecker_intentional [cplusplus.NewDeleteLeaks]
    vtkTypeMacro(Updater, vtkCommand);

    enum class ViewMode {FOLLOW = 0, FREE, OFFSET, FPV};

    static Updater *New() {
        return new Updater;
    }

    Updater();

    void Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId), // NOLINT
                 void * vtkNotUsed(callData));

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

    void init(const std::string &log_dir, double dt);

    void next_mode();

    void process_custom_key(std::string &key);

    void inc_follow();
    void dec_follow();

    void toggle_trails();

    void inc_warp();

    void dec_warp();

    void toggle_helpmenu();

    void toggle_pause();

    void single_step();

    void request_cached();

    void shutting_down();

    void inc_scale();
    void dec_scale();
    void inc_label_scale();
    void dec_label_scale();

    void inc_follow_offset();
    void dec_follow_offset();

    void world_point_clicked(const double &x, const double &y,
                             const double &z);

    void reset_scale();

    void set_reset_camera();
    void set_camera_reset_params(double pos_x, double pos_y, double pos_z,
                                 double focal_x, double focal_y, double focal_z);

    void set_view_mode(ViewMode view_mode);
    void set_show_fps(bool show_fps);
    void set_follow_id(int follow_id);

    void reset_view();

    void create_text_display();

    void update_trail(std::shared_ptr<ActorContact> &actor_contact,
                      double &x_pos, double &y_pos, double &z_pos);

    void update_contact_visual(std::shared_ptr<ActorContact> &actor_contact,
                               std::shared_ptr<scrimmage_proto::ContactVisual> &cv);

    void quat_2_transform(const Quaternion &quat,
                          vtkSmartPointer<vtkTransform> transform);

    bool draw_triangle(const bool &new_shape,
                       const scrimmage_proto::Triangle &t,
                       vtkSmartPointer<vtkActor> &actor,
                       vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                       vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_arrow(const bool &new_shape,
                    const scrimmage_proto::Arrow &a,
                    vtkSmartPointer<vtkActor> &actor,
                    vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                    vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_cone(const bool &new_shape,
                   const scrimmage_proto::Cone &c,
                   vtkSmartPointer<vtkActor> &actor,
                   vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                   vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_line(const bool &new_shape,
                   const scrimmage_proto::Line &l,
                   vtkSmartPointer<vtkActor> &actor,
                   vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                   vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_polygon(const bool &new_shape,
                      const scrimmage_proto::Polygon &p,
                      vtkSmartPointer<vtkActor> &actor,
                      vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                      vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_polydata(const bool &new_shape,
                       const scrimmage_proto::Polydata &p,
                       vtkSmartPointer<vtkActor> &actor,
                       vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                       vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_plane(const bool &new_shape,
                    const scrimmage_proto::Plane &p,
                    vtkSmartPointer<vtkActor> &actor,
                    vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                    vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_pointcloud(const bool &new_shape,
                         const scrimmage_proto::Shape &shape,
                         vtkSmartPointer<vtkActor> &actor,
                         vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                         vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_sphere(const bool &new_shape,
                     const scrimmage_proto::Sphere &s,
                     vtkSmartPointer<vtkActor> &actor,
                     vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                     vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_cube(const bool &new_shape,
                   const scrimmage_proto::Cuboid &c,
                   vtkSmartPointer<vtkActor> &actor,
                   vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                   vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_circle(const bool &new_shape,
                     const scrimmage_proto::Circle &c,
                     vtkSmartPointer<vtkActor> &actor,
                     vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                     vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_ellipse(const bool &new_shape,
                      const scrimmage_proto::Ellipse &elp,
                      vtkSmartPointer<vtkActor> &actor,
                      vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                      vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_text(const bool &new_shape,
                   const scrimmage_proto::Text &t,
                   vtkSmartPointer<vtkActor> &actor,
                   vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                   vtkSmartPointer<vtkPolyDataMapper> &mapper);
    bool draw_mesh(const bool &new_shape,
                   const scrimmage_proto::Mesh &m,
                   vtkSmartPointer<vtkActor> &actor,
                   vtkSmartPointer<vtkPolyDataAlgorithm> &source,
                   vtkSmartPointer<vtkPolyDataMapper> &mapper);

 protected:
    void get_model_texture(std::string name,
                           std::string& model_file, bool& model_found,
                           std::string& texture_file, bool& texture_found,
                           double& base_scale, Quaternion& base_rot);

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
    ViewMode view_mode_prev_;
    bool enable_trails_;

    scrimmage_proto::GUIMsg gui_msg_;

    scrimmage_proto::SimInfo sim_info_;

    vtkSmartPointer<vtkTextActor> time_actor_;
    vtkSmartPointer<vtkTextActor> warp_actor_;
    vtkSmartPointer<vtkTextActor> heading_actor_;
    vtkSmartPointer<vtkTextActor> view_mode_actor_;
    vtkSmartPointer<vtkTextActor> alt_actor_;
    vtkSmartPointer<vtkTextActor> fps_actor_;
    vtkSmartPointer<vtkTextActor> helpkeys_actor_;
    vtkSmartPointer<vtkTextActor> helpvalues_actor_;

    std::map<int, std::shared_ptr<scrimmage_proto::ContactVisual> > contact_visuals_;

    std::unordered_map<uint64_t, std::tuple<scrimmage_proto::Shape,
        vtkSmartPointer<vtkActor>, vtkSmartPointer<vtkPolyDataAlgorithm>>> shapes_;

    std::map<std::string, std::shared_ptr<scrimmage_proto::UTMTerrain> > terrain_map_;
    std::map<std::string, std::shared_ptr<scrimmage_proto::ContactVisual>> contact_visual_map_;

    bool send_shutdown_msg_;

    vtkSmartPointer<vtkActor> terrain_actor_;

    double follow_offset_;
    Eigen::Vector3d follow_vec_;

    bool reset_camera_ = false;
    CameraResetParams camera_reset_params_;
    bool show_fps_ = false;
    std::string log_dir_;
    double dt_ = 0.1;

    bool show_helpmenu_;
    double label_scale_ = 0.3;
};

} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_VIEWER_UPDATER_H_
