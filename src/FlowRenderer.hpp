////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CSP_FLOW_VIS_FLOW_RENDERER_HPP
#define CSP_FLOW_VIS_FLOW_RENDERER_HPP

#include <memory>
#include <string>

//#include <VistaKernel/GraphicsManager/VistaOpenGLDraw.h>
#include <VistaKernel/GraphicsManager/VistaOpenGLNode.h>
#include <VistaOGLExt/VistaBufferObject.h>
#include <VistaOGLExt/VistaFramebufferObj.h>
#include <VistaOGLExt/VistaGLSLShader.h>
#include <VistaOGLExt/VistaTexture.h>
#include <VistaOGLExt/VistaVertexArrayObject.h>

#include "Plugin.hpp"

namespace csp::flowvis {

/// Creates and updates a texture with random generated particles in a fragment shader,
/// 	then displaces the pixels of the image by 2D velocity vectors, 
///     which are given in a 3D texture, representing time slices of 2D areas (2+1D).
///     This representation allows interpolation of velocities in both space and time 
class FlowRenderer {
    //: public IVistaOpenGLDraw {
 public:
  //FlowRenderer() = default;
  //FlowRenderer(std::string vertexSource, std::string fragmentSource);
  explicit FlowRenderer(
      std::shared_ptr<cs::core::Settings>   programSettings,
      std::shared_ptr<Plugin::Settings>     pluginSettings,
      std::shared_ptr<cs::core::GuiManager> pGuiManager
      );

  FlowRenderer(FlowRenderer const& other) = delete;
  FlowRenderer(FlowRenderer&& other)      = delete;
  FlowRenderer& operator=(FlowRenderer const& other) = delete;
  FlowRenderer& operator=(FlowRenderer&& other) = delete;

  virtual ~FlowRenderer() = default;


  void update(double tTime);


  std::shared_ptr<VistaTexture> getVelocity3DTexture() const {
    return mVelocity3DTexture;
  }

  std::shared_ptr<VistaTexture> getCurrentRenderTarget() const {
    return mParticlePingPongTexture[mCurrentRenderTargetIndex];
  }
  
  std::shared_ptr<VistaTexture> getCurrentParticleTexToReadFrom() const {
    return mParticlePingPongTexture[(mCurrentRenderTargetIndex + 1) % 2];
  }

  //justs hacks during refactoring:
  double getCurrentTime() const {
    return mCurrentTime;
  }
  double getLastVisualRenderTime() const {
    return mLastVisualRenderTime;
  }
  void setLastVisualRenderTime(double v) {
    mLastVisualRenderTime = v;
  }


  // render routines:
 protected:  
  void initParticleTextures();

 public:
  void seedParticleTexture();
  void convectParticleTexture();

 protected:

  /// Configures the internal renderer according to the given values.
  void loadVelocityTifFiles(std::string const& tifDirectory);

  void togglePingPongTex();
  

  std::shared_ptr<cs::core::Settings>   mProgramSettings;
  std::shared_ptr<cs::core::GuiManager> mGuiManager;
  std::shared_ptr<Plugin::Settings>     mPluginSettings;


  //------------------------------------
  // Velocity vectors/"simulation" stuff:

  //handle to velocity data from plugin
  GLuint                       mImageWidth  = 0;
  GLuint                       mImageHeight = 0;
  std::shared_ptr<VistaTexture> mVelocity3DTexture;

  double                        mCurrentTime;
  // needed fordifference building, for FPS-independent animation speed
  double                        mLastVisualRenderTime;


  //------------------------------------
  // Particle texture stuff:

  // render routines:
  // void initParticleTexture();
  // void reseedParticleTexture();
  // void updateParticleTexture();

  // fullscreen quad rendering:
  VistaVertexArrayObject mQuadVAO;
  VistaBufferObject      mQuadVBO;

  // offscreen render target:
  VistaFramebufferObj           mFBO;
  GLuint                        mCurrentRenderTargetIndex = 0;
  std::shared_ptr<VistaTexture> mParticlePingPongTexture[2];

  std::shared_ptr<VistaTexture> mSeedTexture;
  // eventually, noise will be calculated one the GPU,
  // but as a quaick hack, let's to it on CPU an then upload
  std::vector<GLfloat> mSeedTextureHostData;

  VistaGLSLShader mSeedAndDisplaceParticlesShader;
  bool            mSeedAndDisplaceParticlesShaderDirty = true;

  // static const char* PARTICLES_SEED_AND_DISPLACE_VERT;
  // static const char* PARTICLES_SEED_AND_DISPLACE_FRAG;
};

} // namespace csp::flowvis

#endif // CSP_FLOW_VIS_FLOW_RENDERER_HPP
