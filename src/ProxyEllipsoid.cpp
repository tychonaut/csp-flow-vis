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

//interpolatable stack of 2D textures == 2D + time dimension == 2+1 D == 3D
uniform sampler3D uVelocity3DTexture;
// point in time to sample the stack of flow-textures;
uniform float     uRelativeTime;
// time interval to "Newton integrate" since last visual render frame:
uniform float     uDurationSinceLastFrame;
// non-physical, user-definable scale factor in order to make the flow speeds visually distinguishable
uniform float     uFlowSpeedScale;

// The least recently updated "random particle ping-pong image".
// (The other one is the current render target.)
uniform sampler2D uParticlesImage;

uniform sampler2D uVelocity2DTexture;

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

    //vec2 texcoords = vec2((vLngLat.x - uBounds.w) / (uBounds.y - uBounds.w),
    //                  1 - (vLngLat.y - uBounds.z) / (uBounds.x - uBounds.z));
    //oColor = texture(uVelocity2DTexture, texcoords).rgb;
    
    vec3 texcoords = vec3((vLngLat.x - uBounds.w) / (uBounds.y - uBounds.w),
                      1 - (vLngLat.y - uBounds.z) / (uBounds.x - uBounds.z),
                      8.0 * uRelativeTime);
                      
    //uRelativeTime
    oColor = texture(uVelocity3DTexture, texcoords).rgb;

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

ProxyEllipsoid::ProxyEllipsoid(std::shared_ptr<cs::core::Settings> programSettings,
    std::shared_ptr<csp::flowvis::Plugin::Settings>                pluginSettings,
    std::shared_ptr<cs::core::SolarSystem> solarSystem, std::string const& sCenterName,
    std::string const& sFrameName)
    : 
    cs::scene::CelestialObject(sCenterName, sFrameName)
    , mProgramSettings(std::move(programSettings))
    , mPluginSettings(std::move(pluginSettings))
    , mSolarSystem(std::move(solarSystem))
    , mRadii(cs::core::SolarSystem::getRadii(sCenterName))
    , mBounds(glm::vec4(0.0f, 0.0f, 0.0f, 0.0f))
    //TODO check for consitency with program startup:
    //dummy value:
    , mCurrentTime(cs::utils::convert::time::toSpice(mPluginSettings->mStartDate)) 
    , mLastVisualRenderTime(cs::utils::convert::time::toSpice(mPluginSettings->mStartDate))

{
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
      [this](bool /*enabled*/) { mPixelDisplaceShaderDirty = true; });
  mEnableHDRConnection =  mProgramSettings->mGraphics.pEnableHDR.connect(
      [this](bool /*enabled*/) { mPixelDisplaceShaderDirty = true; });

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
  
  logger().debug("ProxyEllipsoid::update at time " + std::to_string(tTime));

  CelestialObject::update(tTime, oObs);

  mCurrentTime = tTime;

}

////////////////////////////////////////////////////////////////////////////////////////////////////

// return:(width, height, numchanelsPerPixel, numBitsPerPixelAndChannel)
glm::u32vec4 getTifParams(std::string filepath) {

  auto* data = TIFFOpen(filepath.c_str(), "r");
  if (!data) {
    logger().error("Failed to load GeoTiff '" + filepath + "'!");
    return glm::u32vec4(0, 0, 0, 0);
  } else {
    logger().info("Read GeoTiff '" + filepath + "'");
  }

  uint32 width{};
  uint32 height{};
  uint32 numChannelsPerPixel{};
  uint32 bpp{};
  TIFFGetField(data, TIFFTAG_IMAGEWIDTH, &width);
  TIFFGetField(data, TIFFTAG_IMAGELENGTH, &height);
  TIFFGetField(data, TIFFTAG_SAMPLESPERPIXEL, &numChannelsPerPixel);
  TIFFGetField(data, TIFFTAG_BITSPERSAMPLE, &bpp);

  logger().info(
      "GeoTiff: filepath: {}, width: {}, height: {}, numChannelsPerPixel: {}, bpp: {}", 
       filepath, width, height, numChannelsPerPixel, bpp);

  TIFFClose(data);

  if (bpp != 8 * sizeof(GLfloat)) 
  {
    logger().error("GeoTiff has not 32 bits per pixel!");
  }
   
  return glm::u32vec4(width, height, numChannelsPerPixel, bpp);
}

void ProxyEllipsoid::loadVelocityTifFiles(std::string const& tifDirectory) {

  assert(mVelocity2DTextureArray.empty());
  mVelocity2DTextureArray.clear();

  //init 3D texture
  glEnable(GL_TEXTURE_3D);
  mVelocity3DTexture = std::make_unique<VistaTexture>(GL_TEXTURE_3D);
  
  auto firstFilePath = tifDirectory + "/step_1.tif";
  const glm::u32vec4  firstTifMeta              = getTifParams(firstFilePath);
  const uint32        width                     = firstTifMeta.x;
  const uint32        height                    = firstTifMeta.y;
  const uint32        numChannels               = firstTifMeta.z;
  const uint32        numBitsPerPixelAndChannel = firstTifMeta.w;

  std::vector<float> pixels3D(
      mPluginSettings->mNumTimeSteps * width * height * numChannels);


  //for (int timestep = 0; timestep < mNumTimeSteps; timestep++) {
  // TODO investigate why no 0 is spit out by the resampling scpripts
  for (int timestep = 1; timestep <= mPluginSettings->mNumTimeSteps; timestep++) {

    auto currentFilePath = tifDirectory + "/step_" + std::to_string(timestep) + ".tif";

    auto* data = TIFFOpen(currentFilePath.c_str(), "r");
    if (!data) {
      logger().error("Failed to load GeoTiff '" + currentFilePath + "'!");
      return;
    } else {
      logger().info("Read GeoTiff '" + currentFilePath + "'");
    }

    const glm::u32vec4 currentTifMeta = getTifParams(currentFilePath);
    if (currentTifMeta != firstTifMeta) {
      logger().error("Tif meta data differ amongst each other!");
    }

    int currenImage2DIndex = (timestep - 1) * width * height * numChannels;

    //std::vector<float> pixels2D(width * height * numChannels);

    for (unsigned y = 0; y < height; y++) {
      //if (TIFFReadScanline(data, &pixels2D[width * channels * y], y) < 0) {
      if (TIFFReadScanline(data, &pixels3D[currenImage2DIndex + width * y * numChannels], y) < 0) {
        logger().error("Failed to read tif!");
      }
    }
    logger().info("{} {}", width, height);



    mVelocity2DTextureArray.push_back(std::make_unique<VistaTexture>(GL_TEXTURE_2D));
    mVelocity2DTextureArray.back()->Bind();

    //gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA32F, width, height, GL_RGB, GL_FLOAT, pixels2D.data());
    gluBuild2DMipmaps(
        GL_TEXTURE_2D, GL_RGBA32F, width, height, GL_RGB, GL_FLOAT, &pixels3D[currenImage2DIndex]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

    mVelocity2DTextureArray.back()->Unbind();

    TIFFClose(data);

  }


  mVelocity3DTexture->Bind();
  glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA32F, width, height, mPluginSettings->mNumTimeSteps, 0,
      GL_RGB, GL_FLOAT, pixels3D.data());
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  mVelocity3DTexture->Unbind();

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

  // reported 16384 on Quadro RTX 5000; Should be enough for 2700x2700x73;
  GLint maxTexSize = 0;
  glGetIntegerv(GL_MAX_3D_TEXTURE_SIZE, & maxTexSize );
  logger().debug("maxTexSize: " + std::to_string(maxTexSize));

  cs::utils::FrameTimings::ScopedTimer timer("Flow Vis");

  if (mPixelDisplaceShaderDirty) {
    mPixelDisplaceShader = VistaGLSLShader();

    // (Re-)create sphere shader.
    std::string defines = "#version 330\n";

    if (mProgramSettings->mGraphics.pEnableHDR.get()) {
      defines += "#define ENABLE_HDR\n";
    }

    if (mProgramSettings->mGraphics.pEnableLighting.get()) {
      defines += "#define ENABLE_LIGHTING\n";
    }

    mPixelDisplaceShader.InitVertexShaderFromString(defines + SPHERE_VERT);
    mPixelDisplaceShader.InitFragmentShaderFromString(defines + SPHERE_FRAG);
    mPixelDisplaceShader.Link();

    mPixelDisplaceShaderDirty = false;
  }

  mPixelDisplaceShader.Bind();

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

  mPixelDisplaceShader.SetUniform(mPixelDisplaceShader.GetUniformLocation("uSunDirection"), sunDirection[0], sunDirection[1],
      sunDirection[2]);
  mPixelDisplaceShader.SetUniform(mPixelDisplaceShader.GetUniformLocation("uSunIlluminance"), sunIlluminance);
  mPixelDisplaceShader.SetUniform(mPixelDisplaceShader.GetUniformLocation("uAmbientBrightness"), ambientBrightness);

  // Get modelview and projection matrices.
  std::array<GLfloat, 16> glMatMV{};
  std::array<GLfloat, 16> glMatP{};
  glGetFloatv(GL_MODELVIEW_MATRIX, glMatMV.data());
  glGetFloatv(GL_PROJECTION_MATRIX, glMatP.data());
  auto matMV = glm::make_mat4x4(glMatMV.data()) * glm::mat4(getWorldTransform());
  glUniformMatrix4fv(
      mPixelDisplaceShader.GetUniformLocation("uMatModelView"), 1, GL_FALSE, glm::value_ptr(matMV));
  glUniformMatrix4fv(mPixelDisplaceShader.GetUniformLocation("uMatProjection"), 1, GL_FALSE, glMatP.data());

  mPixelDisplaceShader.SetUniform(mPixelDisplaceShader.GetUniformLocation("uRadii"), static_cast<float>(mRadii[0]),
      static_cast<float>(mRadii[1]), static_cast<float>(mRadii[2]));
  mPixelDisplaceShader.SetUniform(mPixelDisplaceShader.GetUniformLocation("uBounds"),
      cs::utils::convert::toRadians(mBounds[0]), cs::utils::convert::toRadians(mBounds[1]),
      cs::utils::convert::toRadians(mBounds[2]), cs::utils::convert::toRadians(mBounds[3]));
  mPixelDisplaceShader.SetUniform(
      mPixelDisplaceShader.GetUniformLocation("uFarClip"), cs::utils::getCurrentFarClipDistance());

  // in [0..1]
  double relativeTime = ( (mCurrentTime - mStartExistence)) / (mEndExistence - mStartExistence);
  mPixelDisplaceShader.SetUniform(mPixelDisplaceShader.GetUniformLocation("uRelativeTime"), 
      static_cast<float>(relativeTime));

  double uDurationSinceLastFrame = mCurrentTime - mLastVisualRenderTime;
  mPixelDisplaceShader.SetUniform(
      mPixelDisplaceShader.GetUniformLocation("uDurationSinceLastFrame"),
      static_cast<float>(uDurationSinceLastFrame));

   mPixelDisplaceShader.SetUniform(
      mPixelDisplaceShader.GetUniformLocation("uFlowSpeedScale"),
      static_cast<float>(mPluginSettings->mFlowSpeedScale));


  
  
  // 2D velocity tex; will be obsolete soon...
  mPixelDisplaceShader.SetUniform(mPixelDisplaceShader.GetUniformLocation("uVelocity2DTexture"), 0);
  int currentTextureIndex = std::clamp(
      static_cast<int>(
          std::floor(relativeTime * static_cast<double>(mPluginSettings->mNumTimeSteps))),
      0, mPluginSettings->mNumTimeSteps - 1); 
  mVelocity2DTextureArray[currentTextureIndex]->Bind(GL_TEXTURE0);

  // 3D texture:
  mPixelDisplaceShader.SetUniform(mPixelDisplaceShader.GetUniformLocation("uVelocity3DTexture"), 1);
  mVelocity3DTexture->Bind(GL_TEXTURE1);


  // Draw.
  mSphereVAO.Bind();
  glDrawElements(GL_TRIANGLE_STRIP, (GRID_RESOLUTION_X - 1) * (2 + 2 * GRID_RESOLUTION_Y),
      GL_UNSIGNED_INT, nullptr);
  mSphereVAO.Release();

  // Clean up.
  mVelocity2DTextureArray[currentTextureIndex]->Unbind(GL_TEXTURE0);
  mVelocity3DTexture->Unbind(GL_TEXTURE1);
  mPixelDisplaceShader.Release();

  //update time for "last frame"
  mLastVisualRenderTime = mCurrentTime;

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool ProxyEllipsoid::GetBoundingBox(VistaBoundingBox& /*bb*/) {
  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::flowvis
