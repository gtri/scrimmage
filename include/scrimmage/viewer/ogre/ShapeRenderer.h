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
 */

#ifndef INCLUDE_SCRIMMAGE_VIEWER_OGRE_SHAPERENDERER_H_
#define INCLUDE_SCRIMMAGE_VIEWER_OGRE_SHAPERENDERER_H_

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreManualObject.h>
#include <map>
#include <memory>
#include <string>
#include "scrimmage/proto/Shape.pb.h"
#include "scrimmage/viewer/ogre/MaterialPool.h"

namespace scrimmage {
namespace viewer {

/*
 *
 */
struct RenderedShape {
    Ogre::ManualObject* manualObject = nullptr;
    Ogre::SceneNode* sceneNode = nullptr;
    int shapeCase = 0;  // stores oneof_type_case() value
    bool persistent = false;
    double ttl = 0.0;  // Time to live (for non-persistent shapes)
};

/*
 *
 *
 * This class replaces VTK's procedural shape sources (vtkSphereSource,
 * vtkCubeSource, etc.) with Ogre3D ManualObject or prefab equivalents.
 */
class ShapeRenderer {
 public:
    ShapeRenderer(Ogre::SceneManager* sceneMgr, MaterialPool* materialPool);
    ~ShapeRenderer();

    /*
     *
     */
    void init();

    /*
     *
     */
    void processShapes(const scrimmage_proto::Shapes& shapes);

    /*
     *
     * @param dt Delta time since last update.
     */
    void update(double dt);

    /*
     *
     */
    void clear();

    /*
     *
     */
    Ogre::SceneNode* getShapesNode() const { return shapes_node_; }

 private:
    // Shape drawing methods
    void drawSphere(const std::string& name, const scrimmage_proto::Sphere& s,
                    const scrimmage_proto::Color& color, double opacity);
    void drawCube(const std::string& name, const scrimmage_proto::Cuboid& c,
                  const scrimmage_proto::Color& color, double opacity);
    void drawLine(const std::string& name, const scrimmage_proto::Line& l,
                  const scrimmage_proto::Color& color, double opacity);
    void drawArrow(const std::string& name, const scrimmage_proto::Arrow& a,
                   const scrimmage_proto::Color& color, double opacity);
    void drawCircle(const std::string& name, const scrimmage_proto::Circle& c,
                    const scrimmage_proto::Color& color, double opacity);
    void drawCone(const std::string& name, const scrimmage_proto::Cone& c,
                  const scrimmage_proto::Color& color, double opacity);
    void drawPlane(const std::string& name, const scrimmage_proto::Plane& p,
                   const scrimmage_proto::Color& color, double opacity);
    void drawPolygon(const std::string& name, const scrimmage_proto::Polygon& p,
                     const scrimmage_proto::Color& color, double opacity);
    void drawPolyline(const std::string& name, const scrimmage_proto::Polyline& pl,
                      const scrimmage_proto::Color& color, double opacity);
    void drawTriangle(const std::string& name, const scrimmage_proto::Triangle& t,
                      const scrimmage_proto::Color& color, double opacity);
    void drawEllipse(const std::string& name, const scrimmage_proto::Ellipse& e,
                     const scrimmage_proto::Color& color, double opacity);
    void drawArc(const std::string& name, const scrimmage_proto::Arc& a,
                 const scrimmage_proto::Color& color, double opacity);
    void drawSpline(const std::string& name, const scrimmage_proto::Spline& s,
                    const scrimmage_proto::Color& color, double opacity);
    void drawPointCloud(const std::string& name, const scrimmage_proto::Shape& shape,
                        const scrimmage_proto::Color& color, double opacity);
    void drawText(const std::string& name, const scrimmage_proto::Text& t,
                  const scrimmage_proto::Color& color, double opacity);
    void drawMesh(const std::string& name, const scrimmage_proto::Mesh& m,
                  const scrimmage_proto::Color& color, double opacity);

    // Utility methods
    void removeShape(const std::string& name);
    Ogre::ManualObject* createManualObject(const std::string& name);

    // Sphere vertex generation helper
    void generateSphereVertices(Ogre::ManualObject* obj, float radius,
                                const Ogre::ColourValue& color,
                                int rings = 16, int segments = 16);

    Ogre::SceneManager* scene_mgr_;
    MaterialPool* material_pool_;
    Ogre::SceneNode* shapes_node_;

    std::map<std::string, RenderedShape> shapes_;
    int shape_counter_ = 0;
};

}  // namespace viewer
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_VIEWER_OGRE_SHAPERENDERER_H_
