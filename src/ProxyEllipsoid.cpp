////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ProxyEllipsoid.hpp"
#include "logger.hpp"

#include "../../../src/cs-core/Settings.hpp"
#include "../../../src/cs-core/SolarSystem.hpp"
#include "../../../src/cs-graphics/TextureLoader.hpp"
#include "../../../src/cs-utils/FrameTimings.hpp"
#include "../../../src/cs-utils/convert.hpp"
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
uniform sampler2D uVectorTexture;
uniform float uAmbientBrightness;
uniform float uSunIlluminance;
uniform float uFarClip;

// inputs
in vec3 vNormal;
in vec3 vPosition;
in vec3 vCenter;
in vec2 vLngLat;

// outputs
layout(location = 0) out vec3 oColor;

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

    vec2 texcoords = vec2((vLngLat.x - uBounds.w) / (uBounds.y - uBounds.w),
                      1 - (vLngLat.y - uBounds.z) / (uBounds.x - uBounds.z));

    oColor = texture(uVectorTexture, texcoords).rgb;

    oColor.rg = (oColor.rg/4) + 0.25;
    oColor.b = (oColor.b/30);

    #ifdef ENABLE_HDR
      oColor = SRGBtoLINEAR(oColor);
    #endif

    oColor = oColor * uSunIlluminance;

    #ifdef ENABLE_LIGHTING
      vec3 normal = normalize(vNormal);
      float light = max(dot(normal, uSunDirection), 0.0);
      oColor = mix(oColor*uAmbientBrightness, oColor, light);
    #endif

    gl_FragDepth = length(vPosition) / uFarClip;
}
)";

////////////////////////////////////////////////////////////////////////////////////////////////////

ProxyEllipsoid::ProxyEllipsoid(std::shared_ptr<cs::core::Settings> settings,
    std::shared_ptr<cs::core::SolarSystem> solarSystem, std::string const& sCenterName,
    std::string const& sFrameName, double tStartExistence, double tEndExistence, int numTimeSteps)
    : cs::scene::CelestialObject(sCenterName, sFrameName, tStartExistence, tEndExistence)
    , mSettings(std::move(settings))
    , mSolarSystem(std::move(solarSystem))
    , mRadii(cs::core::SolarSystem::getRadii(sCenterName))
    , mNumTimeSteps(numTimeSteps) 
    //TODO check for consitency with program startup:
    , mCurrentTime(tStartExistence) {
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
  mEnableLightingConnection = mSettings->mGraphics.pEnableLighting.connect(
      [this](bool /*enabled*/) { mShaderDirty = true; });
  mEnableHDRConnection =
      mSettings->mGraphics.pEnableHDR.connect([this](bool /*enabled*/) { mShaderDirty = true; });

  // Add to scenegraph.
  VistaSceneGraph* pSG = GetVistaSystem()->GetGraphicsManager()->GetSceneGraph();
  mGLNode.reset(pSG->NewOpenGLNode(pSG->GetRoot(), this));
  VistaOpenSGMaterialTools::SetSortKeyOnSubtree(
      mGLNode.get(), static_cast<int>(cs::utils::DrawOrder::eOpaqueItems));
}

////////////////////////////////////////////////////////////////////////////////////////////////////

ProxyEllipsoid::~ProxyEllipsoid() {
  mSettings->mGraphics.pEnableLighting.disconnect(mEnableLightingConnection);
  mSettings->mGraphics.pEnableHDR.disconnect(mEnableHDRConnection);

  VistaSceneGraph* pSG = GetVistaSystem()->GetGraphicsManager()->GetSceneGraph();
  pSG->GetRoot()->DisconnectChild(mGLNode.get());

}

void ProxyEllipsoid::update(double tTime, cs::scene::CelestialObserver const& oObs) {
  
  logger().debug("ProxyEllipsoid::update at time " + std::to_string(tTime));

  CelestialObject::update(tTime, oObs);

  mCurrentTime = tTime;

}

////////////////////////////////////////////////////////////////////////////////////////////////////

void ProxyEllipsoid::setTifDirectory(std::string const& tifDirectory) {

  assert(mVectorTextures.empty);
  mVectorTextures.clear();
  //for (int timestep = 0; timestep < mNumTimeSteps; timestep++) {
  // TODO investigate why no 0 is spit out by the resampling scpripts
  for (int timestep = 1; timestep <= mNumTimeSteps; timestep++) {

    mVectorTextures.push_back(std::make_unique<VistaTexture>(GL_TEXTURE_2D));
    mVectorTextures.back()->Bind();

    auto currentFilePath = tifDirectory + "/step_" + std::to_string(timestep) + ".tif";

    auto* data = TIFFOpen(currentFilePath.c_str(), "r");
    if (!data) {
      logger().error("Failed to load GeoTiff '" + currentFilePath + "'!");
      return;
    } else {
      logger().info("Read GeoTiff '" + currentFilePath + "'");
    }

    uint32 width{};
    uint32 height{};
    TIFFGetField(data, TIFFTAG_IMAGELENGTH, &height);
    TIFFGetField(data, TIFFTAG_IMAGEWIDTH, &width);

    uint16 bpp{};
    TIFFGetField(data, TIFFTAG_BITSPERSAMPLE, &bpp);

    int16 channels{};
    TIFFGetField(data, TIFFTAG_SAMPLESPERPIXEL, &channels);

    std::vector<float> pixels(width * height * channels);

    for (unsigned y = 0; y < height; y++) {
      if (TIFFReadScanline(data, &pixels[width * channels * y], y) < 0) {
        logger().error("Failed to read tif!");
      }
    }
    logger().info("{} {}", width, height);

    gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA32F, width, height, GL_RGB, GL_FLOAT, pixels.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

    TIFFClose(data);
    mVectorTextures.back()->Unbind();

  }

  /*

  mVectorTexture = std::make_unique<VistaTexture>(GL_TEXTURE_2D);
  mVectorTexture->Bind();





  auto* data = TIFFOpen((tifDirectory + "/step_1.tif").c_str(), "r");
  if (!data) {
    logger().error("Failed to load with libtiff!");
    return;
  }

  uint32 width{};
  uint32 height{};
  TIFFGetField(data, TIFFTAG_IMAGELENGTH, &height);
  TIFFGetField(data, TIFFTAG_IMAGEWIDTH, &width);

  uint16 bpp{};
  TIFFGetField(data, TIFFTAG_BITSPERSAMPLE, &bpp);

  int16 channels{};
  TIFFGetField(data, TIFFTAG_SAMPLESPERPIXEL, &channels);

  std::vector<float> pixels(width * height * channels);

  for (unsigned y = 0; y < height; y++) {
    if (TIFFReadScanline(data, &pixels[width * channels * y], y) < 0) {
      logger().error("Failed to read tif!");
    }
  }
  logger().info("{} {}", width, height);

  gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA32F, width, height, GL_RGB, GL_FLOAT, pixels.data());
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

  TIFFClose(data);
  mVectorTexture->Unbind();

  */

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

  cs::utils::FrameTimings::ScopedTimer timer("Flow Vis");

  if (mShaderDirty) {
    mShader = VistaGLSLShader();

    // (Re-)create sphere shader.
    std::string defines = "#version 330\n";

    if (mSettings->mGraphics.pEnableHDR.get()) {
      defines += "#define ENABLE_HDR\n";
    }

    if (mSettings->mGraphics.pEnableLighting.get()) {
      defines += "#define ENABLE_LIGHTING\n";
    }

    mShader.InitVertexShaderFromString(defines + SPHERE_VERT);
    mShader.InitFragmentShaderFromString(defines + SPHERE_FRAG);
    mShader.Link();

    mShaderDirty = false;
  }

  mShader.Bind();

  glm::vec3 sunDirection(1, 0, 0);
  float     sunIlluminance(1.F);
  float     ambientBrightness(mSettings->mGraphics.pAmbientBrightness.get());

  if (mSun) {
    // For all other bodies we can use the utility methods from the SolarSystem.
    if (mSettings->mGraphics.pEnableHDR.get()) {
      sunIlluminance = static_cast<float>(mSolarSystem->getSunIlluminance(getWorldTransform()[3]));
    }

    sunDirection = mSolarSystem->getSunDirection(getWorldTransform()[3]);
  }

  mShader.SetUniform(mShader.GetUniformLocation("uSunDirection"), sunDirection[0], sunDirection[1],
      sunDirection[2]);
  mShader.SetUniform(mShader.GetUniformLocation("uSunIlluminance"), sunIlluminance);
  mShader.SetUniform(mShader.GetUniformLocation("uAmbientBrightness"), ambientBrightness);

  // Get modelview and projection matrices.
  std::array<GLfloat, 16> glMatMV{};
  std::array<GLfloat, 16> glMatP{};
  glGetFloatv(GL_MODELVIEW_MATRIX, glMatMV.data());
  glGetFloatv(GL_PROJECTION_MATRIX, glMatP.data());
  auto matMV = glm::make_mat4x4(glMatMV.data()) * glm::mat4(getWorldTransform());
  glUniformMatrix4fv(
      mShader.GetUniformLocation("uMatModelView"), 1, GL_FALSE, glm::value_ptr(matMV));
  glUniformMatrix4fv(mShader.GetUniformLocation("uMatProjection"), 1, GL_FALSE, glMatP.data());

  mShader.SetUniform(mShader.GetUniformLocation("uVectorTexture"), 0);
  mShader.SetUniform(mShader.GetUniformLocation("uBounds"),
      cs::utils::convert::toRadians(mBounds[0]), cs::utils::convert::toRadians(mBounds[1]),
      cs::utils::convert::toRadians(mBounds[2]), cs::utils::convert::toRadians(mBounds[3]));
  mShader.SetUniform(mShader.GetUniformLocation("uRadii"), static_cast<float>(mRadii[0]),
      static_cast<float>(mRadii[1]), static_cast<float>(mRadii[2]));
  mShader.SetUniform(
      mShader.GetUniformLocation("uFarClip"), cs::utils::getCurrentFarClipDistance());

  // in [0..1]
  double relativeTime = ( (mCurrentTime - mStartExistence)) / (mEndExistence - mStartExistence);
  //relativeTime = std::clamp(relativeTime, 0.0, 0.999);
  int currentTextureIndex = std::clamp(static_cast<int>( std::floor(relativeTime * static_cast<double>(mNumTimeSteps)) ), 0, mNumTimeSteps-1); 

  logger().debug("relativeTime: " + std::to_string(relativeTime));
  logger().debug("currentTextureIndex: " + std::to_string(currentTextureIndex));

  mVectorTextures[currentTextureIndex]->Bind(GL_TEXTURE0);

  // Draw.
  mSphereVAO.Bind();
  glDrawElements(GL_TRIANGLE_STRIP, (GRID_RESOLUTION_X - 1) * (2 + 2 * GRID_RESOLUTION_Y),
      GL_UNSIGNED_INT, nullptr);
  mSphereVAO.Release();

  // Clean up.
  mVectorTextures[currentTextureIndex]->Unbind(GL_TEXTURE0);
  mShader.Release();

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool ProxyEllipsoid::GetBoundingBox(VistaBoundingBox& /*bb*/) {
  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::flowvis
