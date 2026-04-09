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
 * @brief Dynamic material pool for Ogre3D.
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

/**
 * @brief Manages a pool of Ogre materials for dynamic color/opacity combinations.
 *
 * VTK sets colors directly on actors. Ogre uses materials.
 * This class creates and caches materials for common color/opacity combinations.
 */
class MaterialPool {
 public:
    MaterialPool();
    ~MaterialPool() = default;

    /**
     * @brief Initialize the material pool with base materials.
     */
    void init();

    /**
     * @brief Get or create a material for the given color.
     * @param color The protobuf Color message.
     * @return The material name.
     */
    std::string getMaterial(const scrimmage_proto::Color& color);

    /**
     * @brief Get or create a material for the given color.
     * @param r Red component [0, 255].
     * @param g Green component [0, 255].
     * @param b Blue component [0, 255].
     * @param a Alpha component [0, 1].
     * @return The material name.
     */
    std::string getMaterial(int r, int g, int b, float a = 1.0f);

    /**
     * @brief Get or create a material for the given Ogre color.
     * @param color The Ogre::ColourValue.
     * @return The material name.
     */
    std::string getMaterial(const Ogre::ColourValue& color);

    /**
     * @brief Convert protobuf Color to Ogre::ColourValue.
     * Note: Protobuf Color has no alpha - defaults to 1.0
     */
    static Ogre::ColourValue toOgreColor(const scrimmage_proto::Color& color,
                                          float alpha = 1.0f);

    /**
     * @brief Get the base white material for manual objects.
     */
    std::string getBaseMaterial() const { return base_material_name_; }

    /**
     * @brief Get a wireframe material.
     */
    std::string getWireframeMaterial(const scrimmage_proto::Color& color);

 private:
    /**
     * @brief Generate a unique material name from color components.
     */
    std::string generateMaterialName(int r, int g, int b, int a);

    /**
     * @brief Create a new material with the specified color.
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
