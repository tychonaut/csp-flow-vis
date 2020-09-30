////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CSP_FLOWVIS_PROXY_ELLIPSOID_HPP
#define CSP_FLOWVIS_PROXY_ELLIPSOID_HPP

#include <VistaKernel/GraphicsManager/VistaOpenGLDraw.h>
#include <VistaKernel/GraphicsManager/VistaOpenGLNode.h>
#include <VistaOGLExt/VistaBufferObject.h>
#include <VistaOGLExt/VistaFramebufferObj.h>
#include <VistaOGLExt/VistaGLSLShader.h>
#include <VistaOGLExt/VistaTexture.h>
#include <VistaOGLExt/VistaVertexArrayObject.h>

#include "../../../src/cs-scene/CelestialObject.hpp"
#include "Plugin.hpp"

namespace cs::core {
class Settings;
class SolarSystem;
} // namespace cs::core


namespace cs::scene {
  class CelestialObserver;
} // namespace cs::scene


namespace csp::flowvis {


class FlowRenderer;


/// Dummy ellipsoid to show a texture that is valid in a certain coordinate range.
/// Holds also an instance of FlowRenderer (I have no time to figure out relationships between
/// the different vista GL instances ...)
/// The texture should be in equirectangular projection.
class ProxyEllipsoid : public cs::scene::CelestialObject, public IVistaOpenGLDraw {
 public:
  ProxyEllipsoid(std::shared_ptr<cs::core::Settings> programSettings,
      std::shared_ptr<csp::flowvis::Plugin::Settings> pluginSettings,
      std::shared_ptr<cs::core::GuiManager>           pGuiManager,
      std::shared_ptr<cs::core::SolarSystem> solarSystem, 
      std::string const& sCenterName,
      std::string const& sFrameName);

  void initEllipsoidGeometry();

  ProxyEllipsoid(ProxyEllipsoid const& other) = delete;
  ProxyEllipsoid(ProxyEllipsoid&& other)      = default;

  ProxyEllipsoid& operator=(ProxyEllipsoid const& other) = delete;
  ProxyEllipsoid& operator=(ProxyEllipsoid&& other) = default;

  ~ProxyEllipsoid() override;

  void update(double tTime, cs::scene::CelestialObserver const& oObs) override;


  void setStartDate(std::string const& startDate);
  void setEndDate(std::string const& endDate);
  void setBounds(glm::vec4 const& bounds);


  /// The sun object is used for lighting computation.
  void setSun(std::shared_ptr<const cs::scene::CelestialObject> const& sun);

  /// Interface implementation of IVistaOpenGLDraw.
  bool Do() override;
  void setShaderUniforms();
  void initShader();
  bool GetBoundingBox(VistaBoundingBox& bb) override;

 private:

  std::shared_ptr<cs::core::Settings>               mProgramSettings;
  std::shared_ptr<csp::flowvis::Plugin::Settings>   mPluginSettings;

  std::shared_ptr<cs::core::GuiManager>             mGuiManager;

  std::shared_ptr<cs::core::SolarSystem>            mSolarSystem;
  std::shared_ptr<const cs::scene::CelestialObject> mSun;

  std::unique_ptr<VistaOpenGLNode> mGLNode;

  
  //------------------------------------
  // Ellipsoid render stuff:

  VistaVertexArrayObject mSphereVAO;
  VistaBufferObject      mSphereVBO;
  VistaBufferObject      mSphereIBO;

  glm::dvec3 mRadii;
  glm::vec4  mBounds;

  VistaGLSLShader mShowParticleTexOnSphereShader;
  bool            mShowParticleTexOnSphereShaderDirty = true;
  //hacky shader source code definition
  static const char* SPHERE_VERT;
  static const char* SPHERE_FRAG;



  //-----------------------
  // some lighting variables (that should be obsolete; 
  // it was copypasted simplebodies-plugin)
  int mEnableLightingConnection = -1;
  int mEnableHDRConnection      = -1;






  //------------------------------------
  // Flow render stuff:
  std::unique_ptr<FlowRenderer> mFlowRenderer;


  // TODO delete 2D texture
  //std::vector<std::unique_ptr<VistaTexture> > mVelocity2DTextureArray;

  ////TODO outsource to FlowRenderer
  //double mCurrentTime;
  //// needed fordifference building, for FPS-independent animation speed
  //double                        mLastVisualRenderTime;
  //std::shared_ptr<VistaTexture> mVelocity3DTexture;

};

} // namespace csp::flowvis

#endif // CSP_FLOWVIS_PROXY_ELLIPSOID_HPP
