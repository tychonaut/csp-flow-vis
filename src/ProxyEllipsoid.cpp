////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ProxyEllipsoid.hpp"
#include "FlowRenderer.hpp"
#include "logger.hpp"

#include "../../../src/cs-core/Settings.hpp"
#include "../../../src/cs-core/SolarSystem.hpp"

#include "../../../src/cs-utils/FrameTimings.hpp"

#include "../../../src/cs-graphics/TextureLoader.hpp"
#include "../../../src/cs-utils/utils.hpp"
#include "../../../src/cs-utils/convert.hpp"


#include <VistaKernel/GraphicsManager/VistaSceneGraph.h>
#include <VistaKernel/GraphicsManager/VistaTransformNode.h>
#include <VistaKernel/VistaSystem.h>
#include <VistaKernelOpenSGExt/VistaOpenSGMaterialTools.h>
#include <VistaMath/VistaBoundingBox.h>
#include <VistaOGLExt/VistaOGLUtils.h>

#include <glm/gtc/type_ptr.hpp>
#include <tiffio.h>
#include <utility>

namespace csp::flowvis {

////////////////////////////////////////////////////////////////////////////////////////////////////

const uint32_t GRID_RESOLUTION_X = 200;
const uint32_t GRID_RESOLUTION_Y = 100;

////////////////////////////////////////////////////////////////////////////////////////////////////

const char* ProxyEllipsoid::SPHERE_VERT = R"(
uniform vec3 uSunDirection;
uniform vec3 uRadii;
uniform mat4 uMatModelView;
uniform mat4 uMatProjection;

// inputs
layout(location = 0) in vec2 iGridPos;

// outputs
out vec3 vNormal;
out vec3 vPosition;
out vec3 vCenter;
out vec2 vLngLat;

const float PI = 3.141592654;

vec3 geodeticSurfaceNormal(vec2 lngLat) {
  return vec3(cos(lngLat.y) * sin(lngLat.x), sin(lngLat.y),
      cos(lngLat.y) * cos(lngLat.x));
}

vec3 toCartesian(vec2 lonLat) {
  vec3 n = geodeticSurfaceNormal(lonLat);
  vec3 k = n * uRadii * uRadii;
  float gamma = sqrt(dot(k, n));
  return k / gamma;
}

void main()
{
    vLngLat.x = iGridPos.x * 2.0 * PI - PI;
    vLngLat.y = iGridPos.y * PI - PI/2;
    vPosition = toCartesian(vLngLat);
    vNormal    = (uMatModelView * vec4(geodeticSurfaceNormal(vLngLat), 0.0)).xyz;
    vPosition   = (uMatModelView * vec4(vPosition, 1.0)).xyz;
    vCenter     = (uMatModelView * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
    gl_Position =  uMatProjection * vec4(vPosition, 1);

    if (gl_Position.w > 0) {
      gl_Position /= gl_Position.w;
      if (gl_Position.z >= 1) {
        gl_Position.z = 0.999999;
      }
    }
}
)";

////////////////////////////////////////////////////////////////////////////////////////////////////

const char* ProxyEllipsoid::SPHERE_FRAG = R"(
uniform vec3 uSunDirection;
uniform vec4 uBounds;

// The least recently updated "random particle ping-pong image".
// (The other one is the current render target.)
uniform sampler2D uParticlesImage;

uniform float uAmbientBrightness;
uniform float uSunIlluminance;
uniform float uFarClip;

//{ TODO remove this and outsource this and its logic to FlowRenderer!
//interpolatable stack of 2D textures == 2D + time dimension == 2+1 D == 3D
uniform sampler3D uVelocity3DTexture;
uniform float     uNumTimeSteps;
// point in time to sample the stack of flow-textures;
uniform float     uRelativeTime;
// time interval to "Newton integrate" since last visual render frame:
uniform float     uDurationSinceLastFrame;
// non-physical, user-definable scale factor in order to make the flow speeds visually distinguishable
uniform float     uFlowSpeedScale;
//}


// inputs
in vec3 vNormal;
in vec3 vPosition;
in vec3 vCenter;
in vec2 vLngLat;

// outputs
layout(location = 0) out vec4 oColor;

vec3 SRGBtoLINEAR(vec3 srgbIn)
{
  vec3 bLess = step(vec3(0.04045),srgbIn);
  return mix( srgbIn/vec3(12.92), pow((srgbIn+vec3(0.055))/vec3(1.055),vec3(2.4)), bLess );
}
    
void main()
{
    if (vLngLat.x < uBounds.w || vLngLat.x > uBounds.y || vLngLat.y > uBounds.x || vLngLat.y < uBounds.z) {
      discard;
    }
    
    vec3 texcoords = vec3((vLngLat.x - uBounds.w) / (uBounds.y - uBounds.w),
                      1 - (vLngLat.y - uBounds.z) / (uBounds.x - uBounds.z),
                      //uNumTimeSteps * uRelativeTime);
                      uRelativeTime);
                      
    vec2 vel = texture(uVelocity3DTexture, texcoords).rg;
    float speed = length(vel.xy);

    float scaledTemperature = texture(uVelocity3DTexture, texcoords).b / 32.0;

    //debug draw
    //oColor.rg = (oColor.rg/4) + 0.25;

    // this is the real particle image:
    oColor.b = texture(uParticlesImage, texcoords.xy).r;

    // "particleness" makes overall brightness
    oColor.rgb = texture(uParticlesImage, texcoords.xy).rrr;

    // set red channel --> speediness
    oColor.r = speed;

    // scale green channel by temperature
    oColor.g *= scaledTemperature;
    
    //some transparency
    oColor.a = 0.375;


    #ifdef ENABLE_HDR
      oColor = SRGBtoLINEAR(oColor);
    #endif

    //oColor = oColor * uSunIlluminance;
    //#ifdef ENABLE_LIGHTING
    //  vec3 normal = normalize(vNormal);
    //  float light = max(dot(normal, uSunDirection), 0.0);
    //  oColor = mix(oColor*uAmbientBrightness, oColor, light);
    //#endif

    gl_FragDepth = (length(vPosition) / uFarClip) ;

    float multiplier = 1.0;
    gl_FragDepth = (length(vPosition) / uFarClip) * multiplier;

}
)";

////////////////////////////////////////////////////////////////////////////////////////////////////

ProxyEllipsoid::ProxyEllipsoid(std::shared_ptr<cs::core::Settings> programSettings,
    std::shared_ptr<csp::flowvis::Plugin::Settings>                pluginSettings,
    std::shared_ptr<cs::core::GuiManager>                          pGuiManager,
    std::shared_ptr<cs::core::SolarSystem> solarSystem, 
    std::string const& sCenterName,
    std::string const& sFrameName)
    : 
    cs::scene::CelestialObject(sCenterName, sFrameName)
    , mProgramSettings(std::move(programSettings))
    , mPluginSettings(std::move(pluginSettings))
    , mGuiManager(std::move(pGuiManager))
    , mSolarSystem(std::move(solarSystem))
    , mRadii(cs::core::SolarSystem::getRadii(sCenterName))
    , mBounds(glm::vec4(0.0f, 0.0f, 0.0f, 0.0f))
    //, mCurrentTime(cs::utils::convert::time::toSpice(mPluginSettings->mStartDate)) 
    //, mLastVisualRenderTime(cs::utils::convert::time::toSpice(mPluginSettings->mStartDate))

{
  initEllipsoidGeometry();

     
  setStartDate(mPluginSettings->mStartDate);
  setEndDate(mPluginSettings->mEndDate);
  
  setBounds(glm::vec4(mPluginSettings->mNorth, mPluginSettings->mEast,
      mPluginSettings->mSouth, mPluginSettings->mWest));
  setSun(mSolarSystem->getSun());


  mFlowRenderer = std::make_unique<FlowRenderer>(mProgramSettings, mPluginSettings, mGuiManager);

}

void ProxyEllipsoid::initEllipsoidGeometry() {

  pVisibleRadius = mRadii[0];

  // For rendering the sphere, we create a 2D-grid which is warped into a sphere in the vertex
  // shader. The vertex positions are directly used as texture coordinates.
  std::vector<float>    vertices(GRID_RESOLUTION_X * GRID_RESOLUTION_Y * 2);
  std::vector<unsigned> indices((GRID_RESOLUTION_X - 1) * (2 + 2 * GRID_RESOLUTION_Y));

  for (uint32_t x = 0; x < GRID_RESOLUTION_X; ++x) {
    for (uint32_t y = 0; y < GRID_RESOLUTION_Y; ++y) {
      vertices[(x * GRID_RESOLUTION_Y + y) * 2 + 0] = 1.F / (GRID_RESOLUTION_X - 1) * x;
      vertices[(x * GRID_RESOLUTION_Y + y) * 2 + 1] = 1.F / (GRID_RESOLUTION_Y - 1) * y;
    }
  }

  uint32_t index = 0;

  for (uint32_t x = 0; x < GRID_RESOLUTION_X - 1; ++x) {
    indices[index++] = x * GRID_RESOLUTION_Y;
    for (uint32_t y = 0; y < GRID_RESOLUTION_Y; ++y) {
      indices[index++] = x * GRID_RESOLUTION_Y + y;
      indices[index++] = (x + 1) * GRID_RESOLUTION_Y + y;
    }
    indices[index] = indices[index - 1];
    ++index;
  }

  mSphereVAO.Bind();

  mSphereVBO.Bind(GL_ARRAY_BUFFER);
  mSphereVBO.BufferData(vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

  mSphereIBO.Bind(GL_ELEMENT_ARRAY_BUFFER);
  mSphereIBO.BufferData(indices.size() * sizeof(unsigned), indices.data(), GL_STATIC_DRAW);

  mSphereVAO.EnableAttributeArray(0);
  mSphereVAO.SpecifyAttributeArrayFloat(
      0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), 0, &mSphereVBO);

  mSphereVAO.Release();
  mSphereIBO.Release();
  mSphereVBO.Release();

  // Recreate the shader if lighting or HDR rendering mode are toggled.
  mEnableLightingConnection = mProgramSettings->mGraphics.pEnableLighting.connect(
      [this](bool /*enabled*/) { mShowParticleTexOnSphereShaderDirty = true; });
  mEnableHDRConnection = mProgramSettings->mGraphics.pEnableHDR.connect(
      [this](bool /*enabled*/) { mShowParticleTexOnSphereShaderDirty = true; });

  // Add to scenegraph.
  VistaSceneGraph* pSG = GetVistaSystem()->GetGraphicsManager()->GetSceneGraph();
  mGLNode.reset(pSG->NewOpenGLNode(pSG->GetRoot(), this));
  VistaOpenSGMaterialTools::SetSortKeyOnSubtree(
      mGLNode.get(), static_cast<int>(cs::utils::DrawOrder::eOpaqueItems));
}

////////////////////////////////////////////////////////////////////////////////////////////////////

ProxyEllipsoid::~ProxyEllipsoid() {
  mProgramSettings->mGraphics.pEnableLighting.disconnect(mEnableLightingConnection);
  mProgramSettings->mGraphics.pEnableHDR.disconnect(mEnableHDRConnection);

  VistaSceneGraph* pSG = GetVistaSystem()->GetGraphicsManager()->GetSceneGraph();
  pSG->GetRoot()->DisconnectChild(mGLNode.get());

}

void ProxyEllipsoid::update(double tTime, cs::scene::CelestialObserver const& oObs) {
  
  //logger().debug("ProxyEllipsoid::update at time " + std::to_string(tTime));

  CelestialObject::update(tTime, oObs);

  mFlowRenderer->update(tTime);

}



////////////////////////////////////////////////////////////////////////////////////////////////////

void ProxyEllipsoid::setStartDate(std::string const& startDate) {

    mStartExistence = cs::utils::convert::time::toSpice(startDate);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void ProxyEllipsoid::setEndDate(std::string const& endDate) {
  mEndExistence = cs::utils::convert::time::toSpice(endDate);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void ProxyEllipsoid::setBounds(glm::vec4 const& bounds) {
  mBounds = bounds;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void ProxyEllipsoid::setSun(std::shared_ptr<const cs::scene::CelestialObject> const& sun) {
  mSun = sun;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool ProxyEllipsoid::Do() {

  if (!getIsInExistence() || !pVisible.get()) {
    return true;
  }

  //glEnable(GL_BLEND);
  //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // reported 16384 on Quadro RTX 5000; Should be enough for 2700x2700x73;
  //GLint maxTexSize = 0;
  //glGetIntegerv(GL_MAX_3D_TEXTURE_SIZE, & maxTexSize );
  //logger().debug("maxTexSize: " + std::to_string(maxTexSize));

  cs::utils::FrameTimings::ScopedTimer timer("Flow Vis");

  if (mShowParticleTexOnSphereShaderDirty) {
    initShader();
  }

  //test omission
  mFlowRenderer->seedParticleTexture();
  mFlowRenderer->renderParticleAnimation();

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  setShaderUniforms();

  // Draw.
  mSphereVAO.Bind();
  glDrawElements(GL_TRIANGLE_STRIP, (GRID_RESOLUTION_X - 1) * (2 + 2 * GRID_RESOLUTION_Y),
      GL_UNSIGNED_INT, nullptr);
  mSphereVAO.Release();

  // Clean up.
  mFlowRenderer->getCurrentParticleTexToReadFrom()->Unbind(GL_TEXTURE0);
  mFlowRenderer->getVelocity3DTexture()->Unbind(GL_TEXTURE1);

  mShowParticleTexOnSphereShader.Release();

  glDisable(GL_BLEND);

  //update time for "last frame"
  //mLastVisualRenderTime = mCurrentTime;
  mFlowRenderer->setLastVisualRenderTime(mFlowRenderer->getCurrentTime());

  return true;
}

void ProxyEllipsoid::setShaderUniforms() {

  mShowParticleTexOnSphereShader.Bind();

  glm::vec3 sunDirection(1, 0, 0);
  float     sunIlluminance(1.F);
  float     ambientBrightness(mProgramSettings->mGraphics.pAmbientBrightness.get());
  if (mSun) {
    // For all other bodies we can use the utility methods from the SolarSystem.
    if (mProgramSettings->mGraphics.pEnableHDR.get()) {
      sunIlluminance = static_cast<float>(mSolarSystem->getSunIlluminance(getWorldTransform()[3]));
    }
    sunDirection = mSolarSystem->getSunDirection(getWorldTransform()[3]);
  }
  mShowParticleTexOnSphereShader.SetUniform(
      mShowParticleTexOnSphereShader.GetUniformLocation("uSunDirection"), sunDirection[0],
      sunDirection[1], sunDirection[2]);
  mShowParticleTexOnSphereShader.SetUniform(
      mShowParticleTexOnSphereShader.GetUniformLocation("uSunIlluminance"), sunIlluminance);
  mShowParticleTexOnSphereShader.SetUniform(
      mShowParticleTexOnSphereShader.GetUniformLocation("uAmbientBrightness"), ambientBrightness);

  mShowParticleTexOnSphereShader.SetUniform(
      mShowParticleTexOnSphereShader.GetUniformLocation("uRadii"), static_cast<float>(mRadii[0]),
      static_cast<float>(mRadii[1]), static_cast<float>(mRadii[2]));
  mShowParticleTexOnSphereShader.SetUniform(
      mShowParticleTexOnSphereShader.GetUniformLocation("uBounds"),
      cs::utils::convert::toRadians(mBounds[0]), cs::utils::convert::toRadians(mBounds[1]),
      cs::utils::convert::toRadians(mBounds[2]), cs::utils::convert::toRadians(mBounds[3]));
  mShowParticleTexOnSphereShader.SetUniform(
      mShowParticleTexOnSphereShader.GetUniformLocation("uFarClip"),
      cs::utils::getCurrentFarClipDistance());

  // Get modelview and projection matrices.
  std::array<GLfloat, 16> glMatMV{};
  std::array<GLfloat, 16> glMatP{};
  glGetFloatv(GL_MODELVIEW_MATRIX, glMatMV.data());
  glGetFloatv(GL_PROJECTION_MATRIX, glMatP.data());
  auto matMV = glm::make_mat4x4(glMatMV.data()) * glm::mat4(getWorldTransform());
  glUniformMatrix4fv(mShowParticleTexOnSphereShader.GetUniformLocation("uMatModelView"), 1,
      GL_FALSE, glm::value_ptr(matMV));
  glUniformMatrix4fv(mShowParticleTexOnSphereShader.GetUniformLocation("uMatProjection"), 1,
      GL_FALSE, glMatP.data());

  // should become obsolete soon, TODO remove
  mShowParticleTexOnSphereShader.SetUniform(
      mShowParticleTexOnSphereShader.GetUniformLocation("uNumTimeSteps"),
      static_cast<float>(mPluginSettings->mNumTimeSteps));

  // TODO outsource this logic to FlowRenderer
  // in [0..1]
  double relativeTime =
      (mFlowRenderer->getCurrentTime() - mStartExistence) / (mEndExistence - mStartExistence);
  mShowParticleTexOnSphereShader.SetUniform(
      mShowParticleTexOnSphereShader.GetUniformLocation("uRelativeTime"),
      static_cast<float>(relativeTime));

  double uDurationSinceLastFrame =
      mFlowRenderer->getCurrentTime() - mFlowRenderer->getLastVisualRenderTime();
  mShowParticleTexOnSphereShader.SetUniform(
      mShowParticleTexOnSphereShader.GetUniformLocation("uDurationSinceLastFrame"),
      static_cast<float>(uDurationSinceLastFrame));

  mShowParticleTexOnSphereShader.SetUniform(
      mShowParticleTexOnSphereShader.GetUniformLocation("uFlowSpeedScale"),
      static_cast<float>(mPluginSettings->mFlowSpeedScale));

  //uParticlesImage
  mShowParticleTexOnSphereShader.SetUniform(
      mShowParticleTexOnSphereShader.GetUniformLocation("uParticlesImage"), 0);
  mFlowRenderer->getCurrentParticleTexToReadFrom()->Bind(GL_TEXTURE0);

  // 3D texture:
  mShowParticleTexOnSphereShader.SetUniform(
      mShowParticleTexOnSphereShader.GetUniformLocation("uVelocity3DTexture"), 1);
  mFlowRenderer->getVelocity3DTexture()->Bind(GL_TEXTURE1);
}



void ProxyEllipsoid::initShader() {
  mShowParticleTexOnSphereShader = VistaGLSLShader();

  // (Re-)create sphere shader.
  std::string defines = "#version 330\n";

  if (mProgramSettings->mGraphics.pEnableHDR.get()) {
    defines += "#define ENABLE_HDR\n";
  }

  if (mProgramSettings->mGraphics.pEnableLighting.get()) {
    defines += "#define ENABLE_LIGHTING\n";
  }

  mShowParticleTexOnSphereShader.InitVertexShaderFromString(defines + SPHERE_VERT);
  mShowParticleTexOnSphereShader.InitFragmentShaderFromString(defines + SPHERE_FRAG);
  mShowParticleTexOnSphereShader.Link();

  mShowParticleTexOnSphereShaderDirty = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool ProxyEllipsoid::GetBoundingBox(VistaBoundingBox& /*bb*/) {
  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::flowvis
