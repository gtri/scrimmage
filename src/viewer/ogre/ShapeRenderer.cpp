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
 * @brief Shape renderer implementation.
 */

#include "scrimmage/viewer/ogre/ShapeRenderer.h"
#include "scrimmage/viewer/ogre/CoordinateConverter.h"
#include <OGRE/OgreEntity.h>
#include <cmath>

namespace scrimmage {
namespace viewer {

ShapeRenderer::ShapeRenderer(Ogre::SceneManager* sceneMgr, MaterialPool* materialPool)
    : scene_mgr_(sceneMgr), material_pool_(materialPool), shapes_node_(nullptr) {
}

ShapeRenderer::~ShapeRenderer() {
    clear();
}

void ShapeRenderer::init() {
    shapes_node_ = scene_mgr_->getRootSceneNode()->createChildSceneNode("ShapesNode");
}

Ogre::ManualObject* ShapeRenderer::createManualObject(const std::string& name) {
    return scene_mgr_->createManualObject(name);
}

void ShapeRenderer::processShapes(const scrimmage_proto::Shapes& shapes) {
    for (const auto& shape : shapes.shape()) {
        std::string name = shape.hash_set() ? std::to_string(shape.hash())
                                             : "shape_" + std::to_string(shape_counter_++);

        // Check for existing shape with same name
        auto it = shapes_.find(name);
        bool is_new = (it == shapes_.end());

        // If shape exists and needs update, remove old first
        if (!is_new) {
            removeShape(name);
        }

        const auto& color = shape.color();
        double opacity = shape.opacity();

        // Draw based on shape type
        switch (shape.oneof_type_case()) {
            case scrimmage_proto::Shape::kSphere:
                drawSphere(name, shape.sphere(), color, opacity);
                break;
            case scrimmage_proto::Shape::kCuboid:
                drawCube(name, shape.cuboid(), color, opacity);
                break;
            case scrimmage_proto::Shape::kLine:
                drawLine(name, shape.line(), color, opacity);
                break;
            case scrimmage_proto::Shape::kArrow:
                drawArrow(name, shape.arrow(), color, opacity);
                break;
            case scrimmage_proto::Shape::kCircle:
                drawCircle(name, shape.circle(), color, opacity);
                break;
            case scrimmage_proto::Shape::kCone:
                drawCone(name, shape.cone(), color, opacity);
                break;
            case scrimmage_proto::Shape::kPlane:
                drawPlane(name, shape.plane(), color, opacity);
                break;
            case scrimmage_proto::Shape::kPolygon:
                drawPolygon(name, shape.polygon(), color, opacity);
                break;
            case scrimmage_proto::Shape::kPolyline:
                drawPolyline(name, shape.polyline(), color, opacity);
                break;
            case scrimmage_proto::Shape::kTriangle:
                drawTriangle(name, shape.triangle(), color, opacity);
                break;
            case scrimmage_proto::Shape::kEllipse:
                drawEllipse(name, shape.ellipse(), color, opacity);
                break;
            case scrimmage_proto::Shape::kArc:
                drawArc(name, shape.arc(), color, opacity);
                break;
            case scrimmage_proto::Shape::kSpline:
                drawSpline(name, shape.spline(), color, opacity);
                break;
            case scrimmage_proto::Shape::kPointcloud:
                drawPointCloud(name, shape, color, opacity);
                break;
            case scrimmage_proto::Shape::kText:
                drawText(name, shape.text(), color, opacity);
                break;
            case scrimmage_proto::Shape::kMesh:
                drawMesh(name, shape.mesh(), color, opacity);
                break;
            default:
                break;
        }

        // Track shape
        if (shapes_.find(name) != shapes_.end()) {
            shapes_[name].persistent = shape.persistent();
            shapes_[name].ttl = shape.ttl();
            shapes_[name].shapeCase = static_cast<int>(shape.oneof_type_case());
        }
    }
}

void ShapeRenderer::update(double dt) {
    // Update TTL and remove expired shapes
    std::vector<std::string> to_remove;
    for (auto& [name, shape] : shapes_) {
        if (!shape.persistent) {
            shape.ttl -= dt;
            if (shape.ttl <= 0) {
                to_remove.push_back(name);
            }
        }
    }
    for (const auto& name : to_remove) {
        removeShape(name);
    }
}

void ShapeRenderer::removeShape(const std::string& name) {
    auto it = shapes_.find(name);
    if (it == shapes_.end()) return;

    if (it->second.manualObject) {
        if (it->second.sceneNode) {
            it->second.sceneNode->detachObject(it->second.manualObject);
            scene_mgr_->destroySceneNode(it->second.sceneNode);
        }
        scene_mgr_->destroyManualObject(it->second.manualObject);
    }
    shapes_.erase(it);
}

void ShapeRenderer::clear() {
    for (auto& [name, shape] : shapes_) {
        if (shape.manualObject) {
            if (shape.sceneNode) {
                shape.sceneNode->detachObject(shape.manualObject);
            }
            scene_mgr_->destroyManualObject(shape.manualObject);
        }
        if (shape.sceneNode) {
            scene_mgr_->destroySceneNode(shape.sceneNode);
        }
    }
    shapes_.clear();
}

void ShapeRenderer::generateSphereVertices(Ogre::ManualObject* obj, float radius,
                                            const Ogre::ColourValue& color,
                                            int rings, int segments) {
    // Generate sphere vertices using spherical coordinates
    for (int ring = 0; ring <= rings; ++ring) {
        float phi = M_PI * static_cast<float>(ring) / rings;
        for (int seg = 0; seg <= segments; ++seg) {
            float theta = 2.0f * M_PI * static_cast<float>(seg) / segments;

            float x = radius * std::sin(phi) * std::cos(theta);
            float y = radius * std::cos(phi);
            float z = radius * std::sin(phi) * std::sin(theta);

            obj->position(x, y, z);
            obj->normal(std::sin(phi) * std::cos(theta),
                        std::cos(phi),
                        std::sin(phi) * std::sin(theta));
            obj->colour(color);
        }
    }

    // Generate indices
    for (int ring = 0; ring < rings; ++ring) {
        for (int seg = 0; seg < segments; ++seg) {
            int curr = ring * (segments + 1) + seg;
            int next = curr + segments + 1;

            obj->index(curr);
            obj->index(next);
            obj->index(curr + 1);

            obj->index(curr + 1);
            obj->index(next);
            obj->index(next + 1);
        }
    }
}

void ShapeRenderer::drawSphere(const std::string& name, const scrimmage_proto::Sphere& s,
                                const scrimmage_proto::Color& color, double opacity) {
    std::string matName = material_pool_->getMaterial(color);
    Ogre::ColourValue ogreColor = MaterialPool::toOgreColor(color);
    ogreColor.a = static_cast<float>(opacity);

    Ogre::ManualObject* obj = createManualObject(name);
    obj->begin(matName, Ogre::RenderOperation::OT_TRIANGLE_LIST);
    generateSphereVertices(obj, static_cast<float>(s.radius()), ogreColor);
    obj->end();

    Ogre::SceneNode* node = shapes_node_->createChildSceneNode(name + "_node");
    node->setPosition(CoordinateConverter::toOgre(s.center()));
    node->attachObject(obj);

    shapes_[name] = {obj, node, scrimmage_proto::Shape::kSphere, false, 0.0};
}

void ShapeRenderer::drawCube(const std::string& name, const scrimmage_proto::Cuboid& c,
                              const scrimmage_proto::Color& color, double opacity) {
    std::string matName = material_pool_->getMaterial(color);
    Ogre::ColourValue ogreColor = MaterialPool::toOgreColor(color);
    ogreColor.a = static_cast<float>(opacity);

    float hx = static_cast<float>(c.x_length()) / 2.0f;
    float hy = static_cast<float>(c.y_length()) / 2.0f;
    float hz = static_cast<float>(c.z_length()) / 2.0f;

    Ogre::ManualObject* obj = createManualObject(name);
    obj->begin(matName, Ogre::RenderOperation::OT_TRIANGLE_LIST);

    // Front face
    obj->position(-hx, -hz, hy); obj->normal(0, 0, 1); obj->colour(ogreColor);
    obj->position(hx, -hz, hy);  obj->normal(0, 0, 1); obj->colour(ogreColor);
    obj->position(hx, hz, hy);   obj->normal(0, 0, 1); obj->colour(ogreColor);
    obj->position(-hx, hz, hy);  obj->normal(0, 0, 1); obj->colour(ogreColor);

    // Back face
    obj->position(hx, -hz, -hy);  obj->normal(0, 0, -1); obj->colour(ogreColor);
    obj->position(-hx, -hz, -hy); obj->normal(0, 0, -1); obj->colour(ogreColor);
    obj->position(-hx, hz, -hy);  obj->normal(0, 0, -1); obj->colour(ogreColor);
    obj->position(hx, hz, -hy);   obj->normal(0, 0, -1); obj->colour(ogreColor);

    // Top face
    obj->position(-hx, hz, hy);  obj->normal(0, 1, 0); obj->colour(ogreColor);
    obj->position(hx, hz, hy);   obj->normal(0, 1, 0); obj->colour(ogreColor);
    obj->position(hx, hz, -hy);  obj->normal(0, 1, 0); obj->colour(ogreColor);
    obj->position(-hx, hz, -hy); obj->normal(0, 1, 0); obj->colour(ogreColor);

    // Bottom face
    obj->position(-hx, -hz, -hy); obj->normal(0, -1, 0); obj->colour(ogreColor);
    obj->position(hx, -hz, -hy);  obj->normal(0, -1, 0); obj->colour(ogreColor);
    obj->position(hx, -hz, hy);   obj->normal(0, -1, 0); obj->colour(ogreColor);
    obj->position(-hx, -hz, hy);  obj->normal(0, -1, 0); obj->colour(ogreColor);

    // Right face
    obj->position(hx, -hz, hy);  obj->normal(1, 0, 0); obj->colour(ogreColor);
    obj->position(hx, -hz, -hy); obj->normal(1, 0, 0); obj->colour(ogreColor);
    obj->position(hx, hz, -hy);  obj->normal(1, 0, 0); obj->colour(ogreColor);
    obj->position(hx, hz, hy);   obj->normal(1, 0, 0); obj->colour(ogreColor);

    // Left face
    obj->position(-hx, -hz, -hy); obj->normal(-1, 0, 0); obj->colour(ogreColor);
    obj->position(-hx, -hz, hy);  obj->normal(-1, 0, 0); obj->colour(ogreColor);
    obj->position(-hx, hz, hy);   obj->normal(-1, 0, 0); obj->colour(ogreColor);
    obj->position(-hx, hz, -hy);  obj->normal(-1, 0, 0); obj->colour(ogreColor);

    // Indices for 6 faces (4 vertices each, 2 triangles per face)
    for (int face = 0; face < 6; ++face) {
        int base = face * 4;
        obj->index(base); obj->index(base + 1); obj->index(base + 2);
        obj->index(base); obj->index(base + 2); obj->index(base + 3);
    }

    obj->end();

    Ogre::SceneNode* node = shapes_node_->createChildSceneNode(name + "_node");
    node->setPosition(CoordinateConverter::toOgre(c.center()));
    if (c.has_quat()) {
        node->setOrientation(CoordinateConverter::toOgre(
            scrimmage::Quaternion(c.quat().w(), c.quat().x(), c.quat().y(), c.quat().z())));
    }
    node->attachObject(obj);

    shapes_[name] = {obj, node, scrimmage_proto::Shape::kCuboid, false, 0.0};
}

void ShapeRenderer::drawLine(const std::string& name, const scrimmage_proto::Line& l,
                              const scrimmage_proto::Color& color, double opacity) {
    std::string matName = material_pool_->getMaterial(color);
    Ogre::ColourValue ogreColor = MaterialPool::toOgreColor(color);
    ogreColor.a = static_cast<float>(opacity);

    Ogre::ManualObject* obj = createManualObject(name);
    obj->begin(matName, Ogre::RenderOperation::OT_LINE_LIST);
    obj->position(CoordinateConverter::toOgre(l.start()));
    obj->colour(ogreColor);
    obj->position(CoordinateConverter::toOgre(l.end()));
    obj->colour(ogreColor);
    obj->end();

    Ogre::SceneNode* node = shapes_node_->createChildSceneNode(name + "_node");
    node->attachObject(obj);

    shapes_[name] = {obj, node, scrimmage_proto::Shape::kLine, false, 0.0};
}

void ShapeRenderer::drawArrow(const std::string& name, const scrimmage_proto::Arrow& a,
                               const scrimmage_proto::Color& color, double opacity) {
    std::string matName = material_pool_->getMaterial(color);
    Ogre::ColourValue ogreColor = MaterialPool::toOgreColor(color);
    ogreColor.a = static_cast<float>(opacity);

    Ogre::Vector3 start = CoordinateConverter::toOgre(a.tail());
    Ogre::Vector3 end = CoordinateConverter::toOgre(a.head());
    Ogre::Vector3 dir = (end - start).normalisedCopy();
    float length = (end - start).length();
    float headLength = length * 0.2f;
    float headRadius = length * 0.05f;  // Proportional to length

    Ogre::ManualObject* obj = createManualObject(name);

    // Shaft as a line
    obj->begin(matName, Ogre::RenderOperation::OT_LINE_LIST);
    obj->position(start);
    obj->colour(ogreColor);
    obj->position(end - dir * headLength);
    obj->colour(ogreColor);
    obj->end();

    // Simple arrowhead as triangle fan
    obj->begin(matName, Ogre::RenderOperation::OT_TRIANGLE_FAN);
    obj->position(end);  // Tip
    obj->colour(ogreColor);

    // Generate base circle
    Ogre::Vector3 up = (std::abs(dir.y) < 0.9f) ? Ogre::Vector3::UNIT_Y : Ogre::Vector3::UNIT_X;
    Ogre::Vector3 right = dir.crossProduct(up).normalisedCopy();
    up = right.crossProduct(dir);

    Ogre::Vector3 headBase = end - dir * headLength;
    const int segments = 8;
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        Ogre::Vector3 pos = headBase + right * std::cos(angle) * headRadius +
                           up * std::sin(angle) * headRadius;
        obj->position(pos);
        obj->colour(ogreColor);
    }
    obj->end();

    Ogre::SceneNode* node = shapes_node_->createChildSceneNode(name + "_node");
    node->attachObject(obj);

    shapes_[name] = {obj, node, scrimmage_proto::Shape::kArrow, false, 0.0};
}

void ShapeRenderer::drawCircle(const std::string& name, const scrimmage_proto::Circle& c,
                                const scrimmage_proto::Color& color, double opacity) {
    std::string matName = material_pool_->getMaterial(color);
    Ogre::ColourValue ogreColor = MaterialPool::toOgreColor(color);
    ogreColor.a = static_cast<float>(opacity);

    float radius = static_cast<float>(c.radius());
    const int segments = 32;

    Ogre::ManualObject* obj = createManualObject(name);
    obj->begin(matName, Ogre::RenderOperation::OT_LINE_STRIP);

    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        float x = radius * std::cos(angle);
        float z = radius * std::sin(angle);
        obj->position(x, 0, z);
        obj->colour(ogreColor);
    }
    obj->end();

    Ogre::SceneNode* node = shapes_node_->createChildSceneNode(name + "_node");
    node->setPosition(CoordinateConverter::toOgre(c.center()));
    node->attachObject(obj);

    shapes_[name] = {obj, node, scrimmage_proto::Shape::kCircle, false, 0.0};
}

void ShapeRenderer::drawCone(const std::string& name, const scrimmage_proto::Cone& c,
                              const scrimmage_proto::Color& color, double opacity) {
    std::string matName = material_pool_->getMaterial(color);
    Ogre::ColourValue ogreColor = MaterialPool::toOgreColor(color);
    ogreColor.a = static_cast<float>(opacity);

    float radius = static_cast<float>(c.base_radius());
    float height = static_cast<float>(c.height());
    const int segments = 16;

    Ogre::ManualObject* obj = createManualObject(name);
    obj->begin(matName, Ogre::RenderOperation::OT_TRIANGLE_FAN);

    // Apex
    obj->position(0, height, 0);
    obj->normal(0, 1, 0);
    obj->colour(ogreColor);

    // Base circle
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        float x = radius * std::cos(angle);
        float z = radius * std::sin(angle);

        // Calculate normal
        Ogre::Vector3 tangent(-std::sin(angle), 0, std::cos(angle));
        Ogre::Vector3 up(x, height, z);
        up.normalise();
        Ogre::Vector3 normal = tangent.crossProduct(up);
        normal.normalise();

        obj->position(x, 0, z);
        obj->normal(normal);
        obj->colour(ogreColor);
    }
    obj->end();

    // Base cap
    obj->begin(matName, Ogre::RenderOperation::OT_TRIANGLE_FAN);
    obj->position(0, 0, 0);
    obj->normal(0, -1, 0);
    obj->colour(ogreColor);

    for (int i = segments; i >= 0; --i) {
        float angle = 2.0f * M_PI * i / segments;
        obj->position(radius * std::cos(angle), 0, radius * std::sin(angle));
        obj->normal(0, -1, 0);
        obj->colour(ogreColor);
    }
    obj->end();

    Ogre::SceneNode* node = shapes_node_->createChildSceneNode(name + "_node");
    node->setPosition(CoordinateConverter::toOgre(c.apex()));
    node->attachObject(obj);

    shapes_[name] = {obj, node, scrimmage_proto::Shape::kCone, false, 0.0};
}

void ShapeRenderer::drawPlane(const std::string& name, const scrimmage_proto::Plane& p,
                               const scrimmage_proto::Color& color, double opacity) {
    std::string matName = material_pool_->getMaterial(color);
    Ogre::ColourValue ogreColor = MaterialPool::toOgreColor(color);
    ogreColor.a = static_cast<float>(opacity);

    float hx = static_cast<float>(p.x_length()) / 2.0f;
    float hz = static_cast<float>(p.y_length()) / 2.0f;

    Ogre::ManualObject* obj = createManualObject(name);
    obj->begin(matName, Ogre::RenderOperation::OT_TRIANGLE_LIST);

    obj->position(-hx, 0, -hz); obj->normal(0, 1, 0); obj->colour(ogreColor);
    obj->position(hx, 0, -hz);  obj->normal(0, 1, 0); obj->colour(ogreColor);
    obj->position(hx, 0, hz);   obj->normal(0, 1, 0); obj->colour(ogreColor);
    obj->position(-hx, 0, hz);  obj->normal(0, 1, 0); obj->colour(ogreColor);

    obj->index(0); obj->index(1); obj->index(2);
    obj->index(0); obj->index(2); obj->index(3);

    obj->end();

    Ogre::SceneNode* node = shapes_node_->createChildSceneNode(name + "_node");
    node->setPosition(CoordinateConverter::toOgre(p.center()));
    node->attachObject(obj);

    shapes_[name] = {obj, node, scrimmage_proto::Shape::kPlane, false, 0.0};
}

void ShapeRenderer::drawPolygon(const std::string& name, const scrimmage_proto::Polygon& p,
                                 const scrimmage_proto::Color& color, double opacity) {
    if (p.point_size() < 3) return;

    std::string matName = material_pool_->getMaterial(color);
    Ogre::ColourValue ogreColor = MaterialPool::toOgreColor(color);
    ogreColor.a = static_cast<float>(opacity);

    Ogre::ManualObject* obj = createManualObject(name);
    obj->begin(matName, Ogre::RenderOperation::OT_TRIANGLE_FAN);

    for (int i = 0; i < p.point_size(); ++i) {
        obj->position(CoordinateConverter::toOgre(p.point(i)));
        obj->normal(0, 1, 0);  // Assuming up-facing polygon
        obj->colour(ogreColor);
    }
    obj->end();

    Ogre::SceneNode* node = shapes_node_->createChildSceneNode(name + "_node");
    node->attachObject(obj);

    shapes_[name] = {obj, node, scrimmage_proto::Shape::kPolygon, false, 0.0};
}

void ShapeRenderer::drawPolyline(const std::string& name, const scrimmage_proto::Polyline& pl,
                                  const scrimmage_proto::Color& color, double opacity) {
    if (pl.point_size() < 2) return;

    std::string matName = material_pool_->getMaterial(color);
    Ogre::ColourValue ogreColor = MaterialPool::toOgreColor(color);
    ogreColor.a = static_cast<float>(opacity);

    Ogre::ManualObject* obj = createManualObject(name);
    obj->begin(matName, Ogre::RenderOperation::OT_LINE_STRIP);

    for (int i = 0; i < pl.point_size(); ++i) {
        obj->position(CoordinateConverter::toOgre(pl.point(i)));
        obj->colour(ogreColor);
    }
    obj->end();

    Ogre::SceneNode* node = shapes_node_->createChildSceneNode(name + "_node");
    node->attachObject(obj);

    shapes_[name] = {obj, node, scrimmage_proto::Shape::kPolyline, false, 0.0};
}

void ShapeRenderer::drawTriangle(const std::string& name, const scrimmage_proto::Triangle& t,
                                  const scrimmage_proto::Color& color, double opacity) {
    std::string matName = material_pool_->getMaterial(color);
    Ogre::ColourValue ogreColor = MaterialPool::toOgreColor(color);
    ogreColor.a = static_cast<float>(opacity);

    Ogre::Vector3 p0 = CoordinateConverter::toOgre(t.point0());
    Ogre::Vector3 p1 = CoordinateConverter::toOgre(t.point1());
    Ogre::Vector3 p2 = CoordinateConverter::toOgre(t.point2());

    Ogre::Vector3 normal = (p1 - p0).crossProduct(p2 - p0);
    normal.normalise();

    Ogre::ManualObject* obj = createManualObject(name);
    obj->begin(matName, Ogre::RenderOperation::OT_TRIANGLE_LIST);

    obj->position(p0); obj->normal(normal); obj->colour(ogreColor);
    obj->position(p1); obj->normal(normal); obj->colour(ogreColor);
    obj->position(p2); obj->normal(normal); obj->colour(ogreColor);

    obj->index(0); obj->index(1); obj->index(2);
    obj->end();

    Ogre::SceneNode* node = shapes_node_->createChildSceneNode(name + "_node");
    node->attachObject(obj);

    shapes_[name] = {obj, node, scrimmage_proto::Shape::kTriangle, false, 0.0};
}

void ShapeRenderer::drawEllipse(const std::string& name, const scrimmage_proto::Ellipse& e,
                                 const scrimmage_proto::Color& color, double opacity) {
    std::string matName = material_pool_->getMaterial(color);
    Ogre::ColourValue ogreColor = MaterialPool::toOgreColor(color);
    ogreColor.a = static_cast<float>(opacity);

    float a = static_cast<float>(e.x_radius());
    float b = static_cast<float>(e.y_radius());
    const int segments = 32;

    Ogre::ManualObject* obj = createManualObject(name);
    obj->begin(matName, Ogre::RenderOperation::OT_LINE_STRIP);

    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        float x = a * std::cos(angle);
        float z = b * std::sin(angle);
        obj->position(x, 0, z);
        obj->colour(ogreColor);
    }
    obj->end();

    Ogre::SceneNode* node = shapes_node_->createChildSceneNode(name + "_node");
    node->setPosition(CoordinateConverter::toOgre(e.center()));
    node->attachObject(obj);

    shapes_[name] = {obj, node, scrimmage_proto::Shape::kEllipse, false, 0.0};
}

void ShapeRenderer::drawArc(const std::string& name, const scrimmage_proto::Arc& a,
                             const scrimmage_proto::Color& color, double opacity) {
    std::string matName = material_pool_->getMaterial(color);
    Ogre::ColourValue ogreColor = MaterialPool::toOgreColor(color);
    ogreColor.a = static_cast<float>(opacity);

    // Arc contains a circle (with center/radius) and angle
    float radius = static_cast<float>(a.circle().radius());
    float startAngle = static_cast<float>(a.angle()) / 2.0f;
    float endAngle = -startAngle;
    const int segments = 16;

    Ogre::ManualObject* obj = createManualObject(name);
    obj->begin(matName, Ogre::RenderOperation::OT_LINE_STRIP);

    for (int i = 0; i <= segments; ++i) {
        float t = static_cast<float>(i) / segments;
        float angle = startAngle + (endAngle - startAngle) * t;
        float x = radius * std::cos(angle);
        float z = radius * std::sin(angle);
        obj->position(x, 0, z);
        obj->colour(ogreColor);
    }
    obj->end();

    Ogre::SceneNode* node = shapes_node_->createChildSceneNode(name + "_node");
    node->setPosition(CoordinateConverter::toOgre(a.circle().center()));
    node->attachObject(obj);

    shapes_[name] = {obj, node, scrimmage_proto::Shape::kArc, false, 0.0};
}

void ShapeRenderer::drawSpline(const std::string& name, const scrimmage_proto::Spline& s,
                                const scrimmage_proto::Color& color, double opacity) {
    // For splines, just draw as polyline through control points for now
    if (s.point_size() < 2) return;

    std::string matName = material_pool_->getMaterial(color);
    Ogre::ColourValue ogreColor = MaterialPool::toOgreColor(color);
    ogreColor.a = static_cast<float>(opacity);

    Ogre::ManualObject* obj = createManualObject(name);
    obj->begin(matName, Ogre::RenderOperation::OT_LINE_STRIP);

    for (int i = 0; i < s.point_size(); ++i) {
        obj->position(CoordinateConverter::toOgre(s.point(i)));
        obj->colour(ogreColor);
    }
    obj->end();

    Ogre::SceneNode* node = shapes_node_->createChildSceneNode(name + "_node");
    node->attachObject(obj);

    shapes_[name] = {obj, node, scrimmage_proto::Shape::kSpline, false, 0.0};
}

void ShapeRenderer::drawPointCloud(const std::string& name, const scrimmage_proto::Shape& shape,
                                    const scrimmage_proto::Color& color, double opacity) {
    const auto& pc = shape.pointcloud();
    if (pc.point_size() == 0) return;

    std::string matName = material_pool_->getMaterial(color);
    Ogre::ColourValue ogreColor = MaterialPool::toOgreColor(color);
    ogreColor.a = static_cast<float>(opacity);

    Ogre::ManualObject* obj = createManualObject(name);
    obj->begin(matName, Ogre::RenderOperation::OT_POINT_LIST);

    for (int i = 0; i < pc.point_size(); ++i) {
        obj->position(CoordinateConverter::toOgre(pc.point(i)));
        if (i < pc.color_size()) {
            obj->colour(MaterialPool::toOgreColor(pc.color(i)));
        } else {
            obj->colour(ogreColor);
        }
    }
    obj->end();

    Ogre::SceneNode* node = shapes_node_->createChildSceneNode(name + "_node");
    node->attachObject(obj);

    shapes_[name] = {obj, node, scrimmage_proto::Shape::kPointcloud, false, 0.0};
}

void ShapeRenderer::drawText(const std::string& name, const scrimmage_proto::Text& t,
                              const scrimmage_proto::Color& color, double opacity) {
    // Text rendering in Ogre3D requires either:
    // 1. Overlay system (2D text)
    // 2. MovableText (3D billboard text)
    // For now, we'll skip text rendering - it requires additional setup
    // TODO: Implement MovableText or use Ogre::OverlaySystem
}

void ShapeRenderer::drawMesh(const std::string& name, const scrimmage_proto::Mesh& m,
                              const scrimmage_proto::Color& color, double opacity) {
    // Mesh loading from files requires pre-converted .mesh files
    // For now, create a simple placeholder
    // TODO: Implement mesh loading with resource manager
}

}  // namespace viewer
}  // namespace scrimmage
