////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "FlowRenderer.hpp"

#include "../../../src/cs-utils/utils.hpp"

#include <VistaOGLExt/VistaGLSLShader.h>
#include <VistaOGLExt/VistaShaderRegistry.h>

#include <utility>

namespace csp::flowvis {

////////////////////////////////////////////////////////////////////////////////////////////////////

//FlowRenderer::FlowRenderer(std::string vertexSource, std::string fragmentSource)
//    : mVertexSource(std::move(vertexSource))
//    , mFragmentSource(std::move(fragmentSource)) {
//}

////////////////////////////////////////////////////////////////////////////////////////////////////

//void FlowRenderer::bind() {
//  if (mShaderDirty) {
//    compile();
//    mShaderDirty = false;
//  }
//
//  mShader.Bind();
//}
//
//////////////////////////////////////////////////////////////////////////////////////////////////////
//
//void FlowRenderer::release() {
//  mShader.Release();
//}

////////////////////////////////////////////////////////////////////////////////////////////////////

/*
void FlowRenderer::compile() {
  VistaShaderRegistry& reg = VistaShaderRegistry::GetInstance();



  //cs::utils::replaceString(mVertexSource, "$VP_TERRAIN_SHADER_FUNCTIONS",
  //    reg.RetrieveShader("VistaPlanetTerrainShaderFunctions.vert"));
  //cs::utils::replaceString(mVertexSource, "$VP_TERRAIN_SHADER_UNIFORMS",
  //    reg.RetrieveShader("VistaPlanetTerrainShaderUniforms.glsl"));

  //cs::utils::replaceString(mFragmentSource, "$VP_TERRAIN_SHADER_FUNCTIONS",
  //    reg.RetrieveShader("VistaPlanetTerrainShaderFunctions.frag"));
  //cs::utils::replaceString(mFragmentSource, "$VP_TERRAIN_SHADER_UNIFORMS",
  //    reg.RetrieveShader("VistaPlanetTerrainShaderUniforms.glsl"));




  mShader = VistaGLSLShader();
  mShader.InitVertexShaderFromString(mVertexSource);
  mShader.InitFragmentShaderFromString(mFragmentSource);
  mShader.Link();
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////

FlowRenderer::FlowRenderer(std::shared_ptr<cs::core::Settings> programSettings,
    std::shared_ptr<Plugin::Settings>                          pluginSettings,
    std::shared_ptr<cs::core::GuiManager>                      pGuiManager)
    : mProgramSettings(programSettings), mPluginSettings(pluginSettings)
    , mGuiManager(pGuiManager)
{
  initParticleTexture();
}

void FlowRenderer::initParticleTexture() {
    //TODO
}

} // namespace csp::flowvis
