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

/// This is just a sphere with a texture, attached to the given SPICE frame. The texture should be
/// in equirectangular projection.
class ProxyEllipsoid : public cs::scene::CelestialObject, public IVistaOpenGLDraw {
 public:
  ProxyEllipsoid(std::shared_ptr<cs::core::Settings> programSettings,
      std::shared_ptr<csp::flowvis::Plugin::Settings> pluginSettings,
      std::shared_ptr<cs::core::SolarSystem> solarSystem, 
      std::string const& sCenterName,
      std::string const& sFrameName);

  ProxyEllipsoid(ProxyEllipsoid const& other) = delete;
  ProxyEllipsoid(ProxyEllipsoid&& other)      = default;

  ProxyEllipsoid& operator=(ProxyEllipsoid const& other) = delete;
  ProxyEllipsoid& operator=(ProxyEllipsoid&& other) = default;

  ~ProxyEllipsoid() override;

  void update(double tTime, cs::scene::CelestialObserver const& oObs) override;

  /// Configures the internal renderer according to the given values.
  void loadVelocityTifFiles(std::string const& tifDirectory);
  void setStartDate(std::string const& startDate);
  void setEndDate(std::string const& endDate);
  void setBounds(glm::vec4 const& bounds);


  /// The sun object is used for lighting computation.
  void setSun(std::shared_ptr<const cs::scene::CelestialObject> const& sun);

  /// Interface implementation of IVistaOpenGLDraw.
  bool Do() override;
  bool GetBoundingBox(VistaBoundingBox& bb) override;

 private:
  std::shared_ptr<cs::core::Settings>               mProgramSettings;
  std::shared_ptr<csp::flowvis::Plugin::Settings>   mPluginSettings;


  std::shared_ptr<cs::core::SolarSystem>            mSolarSystem;
  std::shared_ptr<const cs::scene::CelestialObject> mSun;

  std::unique_ptr<VistaOpenGLNode> mGLNode;

  
  //------------------------------------
  // Ellipsoid renders stuff:

  VistaVertexArrayObject mSphereVAO;
  VistaBufferObject      mSphereVBO;
  VistaBufferObject      mSphereIBO;

  glm::dvec3 mRadii;
  glm::vec4  mBounds;

  double mCurrentTime;
  // needed fordifference building, for FPS-independent animation speed
  double mLastVisualRenderTime;

  VistaGLSLShader mPixelDisplaceShader;
  bool            mPixelDisplaceShaderDirty = true;
  //some lighting variables (from copypasted simplebodies-plugin)
  int  mEnableLightingConnection = -1;
  int  mEnableHDRConnection      = -1;

  static const char* SPHERE_VERT;
  static const char* SPHERE_FRAG;


  //------------------------------------
  // Velocity vectors stuff:

  // TODO delete 2D texture
  std::vector<std::unique_ptr<VistaTexture>> mVelocity2DTextureArray;

  std::unique_ptr<VistaTexture> mVelocity3DTexture;


  //------------------------------------
  // Particle texture stuff:

  // render routines:
  //void initParticleTexture();
  //void reseedParticleTexture();
  //void updateParticleTexture();

  //fullscreen quad rendering:
  VistaVertexArrayObject    mQuadVAO;
  VistaBufferObject         mQuadVBO;

  //offscreen render target:
  VistaFramebufferObj       mFBO;
  GLuint                        currentRenderTargetIndex = 0;
  std::unique_ptr<VistaTexture> mParticlePingPongTexture[2];

  VistaGLSLShader           mSeedAndDisplaceParticlesShader;

  //static const char* PARTICLES_SEED_AND_DISPLACE_VERT;
  //static const char* PARTICLES_SEED_AND_DISPLACE_FRAG;


};

} // namespace csp::flowvis

#endif // CSP_FLOWVIS_PROXY_ELLIPSOID_HPP
