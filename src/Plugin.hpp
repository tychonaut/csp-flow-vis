////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CSP_FLOWVIS_PLUGIN_HPP
#define CSP_FLOWVIS_PLUGIN_HPP

#include "../../../src/cs-core/PluginBase.hpp"

#include <string>

namespace csp::flowvis {

class ProxyEllipsoid;

class Plugin : public cs::core::PluginBase {
 public:
  struct Settings {
    std::string mAnchor;
    std::string mTifDirectory;
    std::string mStartDate;
    std::string mEndDate;
    float       mEast;
    float       mWest;
    float       mNorth;
    float       mSouth;
    int         mNumTimeSteps;
  };

  void init() override;
  void deInit() override;

 private:
  void onLoad();

  Settings                        mPluginSettings;
  std::shared_ptr<ProxyEllipsoid> mProxyEllipsoid;

  int mOnLoadConnection = -1;
  int mOnSaveConnection = -1;
};

} // namespace csp::flowvis

#endif // CSP_FLOWVIS_PLUGIN_HPP
