////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Plugin.hpp"

#include "../../../src/cs-core/InputManager.hpp"
#include "../../../src/cs-core/Settings.hpp"
#include "../../../src/cs-core/SolarSystem.hpp"
#include "../../../src/cs-utils/logger.hpp"
#include "../../../src/cs-utils/utils.hpp"
#include "ProxyEllipsoid.hpp"
#include "logger.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////

EXPORT_FN cs::core::PluginBase* create() {
  return new csp::flowvis::Plugin;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

EXPORT_FN void destroy(cs::core::PluginBase* pluginBase) {
  delete pluginBase; // NOLINT(cppcoreguidelines-owning-memory)
}

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace csp::flowvis {

////////////////////////////////////////////////////////////////////////////////////////////////////

void from_json(nlohmann::json const& j, Plugin::Settings& o) {
  cs::core::Settings::deserialize(j, "anchor", o.mAnchor);
  cs::core::Settings::deserialize(j, "tifDirectory", o.mTifDirectory);
  cs::core::Settings::deserialize(j, "startDate", o.mStartDate);
  cs::core::Settings::deserialize(j, "endDate", o.mEndDate);
  cs::core::Settings::deserialize(j, "east", o.mEast);
  cs::core::Settings::deserialize(j, "west", o.mWest);
  cs::core::Settings::deserialize(j, "north", o.mNorth);
  cs::core::Settings::deserialize(j, "south", o.mSouth);
  cs::core::Settings::deserialize(j, "numTimeSteps", o.mNumTimeSteps);
}

void to_json(nlohmann::json& j, Plugin::Settings const& o) {
  cs::core::Settings::serialize(j, "anchor", o.mAnchor);
  cs::core::Settings::serialize(j, "tifDirectory", o.mTifDirectory);
  cs::core::Settings::serialize(j, "startDate", o.mStartDate);
  cs::core::Settings::serialize(j, "endDate", o.mEndDate);
  cs::core::Settings::serialize(j, "east", o.mEast);
  cs::core::Settings::serialize(j, "west", o.mWest);
  cs::core::Settings::serialize(j, "north", o.mNorth);
  cs::core::Settings::serialize(j, "south", o.mSouth);
  cs::core::Settings::serialize(j, "numTimeSteps", o.mNumTimeSteps);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Plugin::init() {

  logger().info("Loading plugin...");

  mOnLoadConnection = mAllSettings->onLoad().connect([this]() { onLoad(); });
  mOnSaveConnection = mAllSettings->onSave().connect(
      [this]() { mAllSettings->mPlugins["csp-flow-vis"] = mPluginSettings; });

  // Load settings.
  onLoad();

  logger().info("Loading done.");
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Plugin::deInit() {
  logger().info("Unloading plugin...");

  mAllSettings->onLoad().disconnect(mOnLoadConnection);
  mAllSettings->onSave().disconnect(mOnSaveConnection);

  mSolarSystem->unregisterAnchor(mProxyEllipsoid);

  logger().info("Unloading done.");
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Plugin::onLoad() {
  // Read settings from JSON.
  from_json(mAllSettings->mPlugins.at("csp-flow-vis"), mPluginSettings);

  auto anchor = mAllSettings->mAnchors.find(mPluginSettings.mAnchor);

  if (anchor == mAllSettings->mAnchors.end()) {
    throw std::runtime_error(
        "There is no Anchor \"" + mPluginSettings.mAnchor + "\" defined in the settings.");
  }

  auto [tStartExistence, tEndExistence] = anchor->second.getExistence();

  mProxyEllipsoid = std::make_shared<ProxyEllipsoid>(mAllSettings, mSolarSystem,
      anchor->second.mCenter, anchor->second.mFrame, tStartExistence, tEndExistence, mPluginSettings.mNumTimeSteps);

  mProxyEllipsoid->setTifDirectory(mPluginSettings.mTifDirectory);
  mProxyEllipsoid->setStartDate(mPluginSettings.mStartDate);
  mProxyEllipsoid->setEndDate(mPluginSettings.mEndDate);
  mProxyEllipsoid->setBounds(glm::vec4(mPluginSettings.mNorth, mPluginSettings.mEast,
      mPluginSettings.mSouth, mPluginSettings.mWest));
  mProxyEllipsoid->setSun(mSolarSystem->getSun());

  mSolarSystem->registerAnchor(mProxyEllipsoid);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::flowvis
