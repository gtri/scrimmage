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
 * Manages materials for debug shapes and entities with dynamic colors/opacity.
 */

#ifndef INCLUDE_SCRIMMAGE_VIEWER_OGRE_MATERIALPOOL_H_
#define INCLUDE_SCRIMMAGE_VIEWER_OGRE_MATERIALPOOL_H_

#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreMaterialManager.h>
#include <string>
#include <unordered_map>
#include "scrimmage/proto/Color.pb.h"

namespace scrimmage {
namespace viewer {

/*
 *
 *
 * VTK sets colors directly on actors. Ogre uses materials.
 * This class creates and caches materials for common color/opacity combinations.
 */
class MaterialPool {
 public:
    MaterialPool();
    ~MaterialPool() = default;

    /*
     *
     */
    void init();

    /*
     *
     * @param color The protobuf Color message.
     * @return The material name.
     */
    std::string getMaterial(const scrimmage_proto::Color& color);

    /*
     *
     * @param r Red component [0, 255].
     * @param g Green component [0, 255].
     * @param b Blue component [0, 255].
     * @param a Alpha component [0, 1].
     * @return The material name.
     */
    std::string getMaterial(int r, int g, int b, float a = 1.0f);

    /*
     *
     * @param color The Ogre::ColourValue.
     * @return The material name.
     */
    std::string getMaterial(const Ogre::ColourValue& color);

    /*
     *
     * Note: Protobuf Color has no alpha - defaults to 1.0
     */
    static Ogre::ColourValue toOgreColor(const scrimmage_proto::Color& color,
                                          float alpha = 1.0f);

    /*
     *
     */
    std::string getBaseMaterial() const { return base_material_name_; }

    /*
     *
     */
    std::string getWireframeMaterial(const scrimmage_proto::Color& color);

 private:
    /*
     *
     */
    std::string generateMaterialName(int r, int g, int b, int a);

    /*
     *
     */
    void createMaterial(const std::string& name, const Ogre::ColourValue& color,
                        bool wireframe = false);

    std::unordered_map<std::string, Ogre::MaterialPtr> material_cache_;
    std::string base_material_name_ = "SCRIMMAGE/BaseWhite";
    bool initialized_ = false;
};

}  // namespace viewer
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_VIEWER_OGRE_MATERIALPOOL_H_
