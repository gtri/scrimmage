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
 * @brief Dynamic material pool for Ogre3D implementation.
 */

#include "scrimmage/viewer/ogre/MaterialPool.h"
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgrePass.h>
#include <OGRE/OgreResourceGroupManager.h>
#include <sstream>
#include <iomanip>

namespace scrimmage {
namespace viewer {

MaterialPool::MaterialPool() {}

void MaterialPool::init() {
    if (initialized_) return;

    // Create base white material that can be cloned for others
    Ogre::MaterialPtr baseMat = Ogre::MaterialManager::getSingleton().create(
        base_material_name_,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    Ogre::Pass* pass = baseMat->getTechnique(0)->getPass(0);
    pass->setLightingEnabled(true);
    pass->setDiffuse(Ogre::ColourValue::White);
    pass->setAmbient(Ogre::ColourValue(0.3f, 0.3f, 0.3f));
    pass->setSpecular(Ogre::ColourValue(0.3f, 0.3f, 0.3f));
    pass->setShininess(32.0f);

    material_cache_[base_material_name_] = baseMat;

    // Pre-create common debug colors
    createMaterial("SCRIMMAGE/Red", Ogre::ColourValue(1.0f, 0.0f, 0.0f));
    createMaterial("SCRIMMAGE/Green", Ogre::ColourValue(0.0f, 1.0f, 0.0f));
    createMaterial("SCRIMMAGE/Blue", Ogre::ColourValue(0.0f, 0.0f, 1.0f));
    createMaterial("SCRIMMAGE/Yellow", Ogre::ColourValue(1.0f, 1.0f, 0.0f));
    createMaterial("SCRIMMAGE/Cyan", Ogre::ColourValue(0.0f, 1.0f, 1.0f));
    createMaterial("SCRIMMAGE/Magenta", Ogre::ColourValue(1.0f, 0.0f, 1.0f));
    createMaterial("SCRIMMAGE/White", Ogre::ColourValue(1.0f, 1.0f, 1.0f));
    createMaterial("SCRIMMAGE/Black", Ogre::ColourValue(0.0f, 0.0f, 0.0f));
    createMaterial("SCRIMMAGE/Orange", Ogre::ColourValue(1.0f, 0.5f, 0.0f));
    createMaterial("SCRIMMAGE/Purple", Ogre::ColourValue(0.5f, 0.0f, 0.5f));
    createMaterial("SCRIMMAGE/Gray", Ogre::ColourValue(0.5f, 0.5f, 0.5f));

    // Create terrain material
    Ogre::MaterialPtr terrainMat = Ogre::MaterialManager::getSingleton().create(
        "SCRIMMAGE/Terrain",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    Ogre::Pass* terrainPass = terrainMat->getTechnique(0)->getPass(0);
    terrainPass->setLightingEnabled(true);
    terrainPass->setDiffuse(Ogre::ColourValue(0.4f, 0.6f, 0.3f));
    terrainPass->setAmbient(Ogre::ColourValue(0.2f, 0.3f, 0.15f));

    initialized_ = true;
}

Ogre::ColourValue MaterialPool::toOgreColor(const scrimmage_proto::Color& color,
                                             float alpha) {
    return Ogre::ColourValue(
        color.r() / 255.0f,
        color.g() / 255.0f,
        color.b() / 255.0f,
        alpha
    );
}

std::string MaterialPool::generateMaterialName(int r, int g, int b, int a) {
    std::ostringstream oss;
    oss << "SCRIMMAGE/Dynamic_"
        << std::setfill('0') << std::setw(3) << r << "_"
        << std::setfill('0') << std::setw(3) << g << "_"
        << std::setfill('0') << std::setw(3) << b << "_"
        << std::setfill('0') << std::setw(3) << a;
    return oss.str();
}

void MaterialPool::createMaterial(const std::string& name,
                                  const Ogre::ColourValue& color,
                                  bool wireframe) {
    if (material_cache_.find(name) != material_cache_.end()) {
        return;  // Already exists
    }

    Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
        name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    Ogre::Pass* pass = mat->getTechnique(0)->getPass(0);
    pass->setLightingEnabled(true);
    pass->setDiffuse(color);
    pass->setAmbient(color * 0.3f);
    pass->setSpecular(Ogre::ColourValue(0.3f, 0.3f, 0.3f));
    pass->setShininess(32.0f);

    // Handle transparency
    if (color.a < 1.0f) {
        pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        pass->setDepthWriteEnabled(false);
    }

    if (wireframe) {
        pass->setPolygonMode(Ogre::PM_WIREFRAME);
    }

    material_cache_[name] = mat;
}

std::string MaterialPool::getMaterial(const scrimmage_proto::Color& color) {
    // Color proto doesn't have alpha, use default of 1.0
    return getMaterial(color.r(), color.g(), color.b(), 1.0f);
}

std::string MaterialPool::getMaterial(int r, int g, int b, float a) {
    // Quantize alpha to reduce material count (10 levels)
    int alpha_quantized = static_cast<int>(a * 10.0f) * 10;
    if (alpha_quantized > 100) alpha_quantized = 100;

    std::string name = generateMaterialName(r, g, b, alpha_quantized);

    if (material_cache_.find(name) == material_cache_.end()) {
        Ogre::ColourValue color(r / 255.0f, g / 255.0f, b / 255.0f, a);
        createMaterial(name, color);
    }

    return name;
}

std::string MaterialPool::getMaterial(const Ogre::ColourValue& color) {
    int r = static_cast<int>(color.r * 255);
    int g = static_cast<int>(color.g * 255);
    int b = static_cast<int>(color.b * 255);
    return getMaterial(r, g, b, color.a);
}

std::string MaterialPool::getWireframeMaterial(const scrimmage_proto::Color& color) {
    // Color proto doesn't have alpha, use default of 100 (1.0 * 100)
    std::string baseName = generateMaterialName(color.r(), color.g(), color.b(), 100);
    std::string name = baseName + "_wireframe";

    if (material_cache_.find(name) == material_cache_.end()) {
        createMaterial(name, toOgreColor(color), true);
    }

    return name;
}

}  // namespace viewer
}  // namespace scrimmage
