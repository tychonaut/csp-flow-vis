////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CSP_FLOW_VIS_FLOW_RENDERER_HPP
#define CSP_FLOW_VIS_FLOW_RENDERER_HPP

#include <memory>
#include <string>

#include <VistaKernel/GraphicsManager/VistaOpenGLDraw.h>

#include <VistaOGLExt/VistaGLSLShader.h>
#include <VistaOGLExt/VistaVertexArrayObject.h>
#include <VistaOGLExt/VistaBufferObject.h>

#include "Plugin.hpp"

namespace csp::flowvis {

/// Creates and updates a texture with random generated particles in a fragment shader,
/// 	then displaces the pixels of the image by 2D velocity vectors, 
///     which are given in a 3D texture, representing time slices of 2D areas (2+1D).
///     This representation allows interpolation of velocities in both space and time 
class FlowRenderer : public IVistaOpenGLDraw {
 public:
  //FlowRenderer() = default;
  //FlowRenderer(std::string vertexSource, std::string fragmentSource);
  FlowRenderer(
      std::shared_ptr<cs::core::Settings>   programSettings,
      std::shared_ptr<Plugin::Settings>     pluginSettings,
      std::shared_ptr<cs::core::GuiManager> pGuiManager
      );

  FlowRenderer(FlowRenderer const& other) = delete;
  FlowRenderer(FlowRenderer&& other)      = delete;
  FlowRenderer& operator=(FlowRenderer const& other) = delete;
  FlowRenderer& operator=(FlowRenderer&& other) = delete;

  virtual ~FlowRenderer() = default;


  VistaVertexArrayObject mQuadVAO;
  VistaBufferObject      mQuadVBO;


 protected:


  std::shared_ptr<cs::core::Settings>   mProgramSettings;
  std::shared_ptr<cs::core::GuiManager> mGuiManager;
  std::shared_ptr<Plugin::Settings>     mPluginSettings;


  VistaGLSLShader mSeedParticlesShader;
  VistaGLSLShader mDisplaceParticlesShader;
};

} // namespace csp::flowvis

#endif // CSP_FLOW_VIS_FLOW_RENDERER_HPP
