////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "FlowRenderer.hpp"
#include "logger.hpp"


#include "../../../src/cs-utils/utils.hpp"

#include <VistaOGLExt/VistaGLSLShader.h>
#include <VistaOGLExt/VistaShaderRegistry.h>

//#include "../../../src/cs-graphics/TextureLoader.hpp"
//#include "../../../src/cs-utils/utils.hpp"
#include "../../../src/cs-utils/convert.hpp"



#include <glm/glm.hpp>

#include <tiffio.h>
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
    : mProgramSettings(programSettings)
    , mPluginSettings(pluginSettings)
    , mGuiManager(pGuiManager)
    // TODO check for consitency with program startup:
    // dummy value:
    , mCurrentTime(cs::utils::convert::time::toSpice(pluginSettings->mStartDate))
    , mLastVisualRenderTime(cs::utils::convert::time::toSpice(pluginSettings->mStartDate)) 
{

        logger().debug("DEBUGGING 2w3R!!!11");

  loadVelocityTifFiles(mPluginSettings->mTifDirectory);

  initParticleTextures();
}

void FlowRenderer::update(double tTime) {
  mCurrentTime = tTime;
}



void FlowRenderer::initParticleTextures() {

  // init seed of RNG, so that it is identical on all cluster machines:
  srand(5);
  
  mSeedTextureHostData.clear();
  GLuint numCells = mImageWidth * mImageHeight;
  mSeedTextureHostData.resize(numCells);
  
  // init to default: gray (0.5), "no particles"...
  for (GLuint currentCell = 0; currentCell < numCells; currentCell++) {
    mSeedTextureHostData[currentCell] = 0.5;
  }
 
  for (int i = 0; i < 2; i++) {
    mParticlePingPongTexture[i] = std::make_shared<VistaTexture>(GL_TEXTURE_2D);
    mParticlePingPongTexture[i]->Bind();

    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, mImageWidth, mImageHeight, 0, GL_RED, GL_FLOAT,
        mSeedTextureHostData.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    mParticlePingPongTexture[i]->Unbind();
  }

  glFlush();


  // seed the RNG for reproducability on each cluster node
  mRNG.seed(mSeedval);

  seedParticleTexture();

}

void FlowRenderer::seedParticleTexture() {
  
  // hack in order so not update every frame;
  static int currentSeedCycleTime = 0;
  const int  numFramesBetweenReseed = 60;
  if (currentSeedCycleTime >= numFramesBetweenReseed) {
    currentSeedCycleTime = 0;
  } else {
    currentSeedCycleTime++;
    return;
  }

  glFlush();

  getCurrentParticleTexToReadFrom()->Bind();

  //hack: read back tex from GPU:
  glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, mSeedTextureHostData.data());

  glFlush();

  const GLint                        numCells = mImageWidth * mImageHeight;
  std::uniform_int_distribution<rng_type::result_type> udist(0, numCells -1);

  const GLint numNewParticles =
      static_cast <GLint>(static_cast<float>(numCells) * mPluginSettings->mParticleSeedThreshold);
  //seed to random locations:
  for (size_t i = 0; i < numNewParticles; i++) {
    mSeedTextureHostData[udist(mRNG)] = 0.0;
    mSeedTextureHostData[udist(mRNG)] = 1.0;
  }

  //// put some new random black&white dots into the texture:
  //GLfloat currentRandomValue = 0.0f;
  //for (GLuint currentCell = 0; currentCell < numCells; currentCell++) {
  //  // in [0..1]
  //  currentRandomValue = static_cast<GLfloat>(rand()) / static_cast<GLfloat>(RAND_MAX);

  //  // if close to zero or one (defined by user via particleSeedThreshold),
  //  // then black (0.0) or white (1.0), respectively
  //  mSeedTextureHostData[currentCell] = 0.5;
  //  if (currentRandomValue < mPluginSettings->mParticleSeedThreshold) {
  //    mSeedTextureHostData[currentCell] = 0.0;
  //  }
  //  if (currentRandomValue > (1.0 - mPluginSettings->mParticleSeedThreshold)) {
  //    mSeedTextureHostData[currentCell] = 1.0;
  //  }
  //}

  glFlush();

  // hack: re-upload
  glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, mImageWidth, mImageHeight, 0, GL_RED, GL_FLOAT,
      mSeedTextureHostData.data());

  glFlush();

  getCurrentParticleTexToReadFrom()->Unbind();
}

void FlowRenderer::convectParticleTexture() {

    // some old quatsch, TODO delete
  // 2D velocity tex; will be obsolete soon...
  // mShowParticleTexOnSphereShader.SetUniform(mShowParticleTexOnSphereShader.GetUniformLocation("uVelocity2DTexture"),
  // 0); mVelocity2DTextureArray[currentTextureIndex]->Bind(GL_TEXTURE0);
  


    //TODO
}

void FlowRenderer::togglePingPongTex() {
  mCurrentRenderTargetIndex = (mCurrentRenderTargetIndex + 1) % 2;
}



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

  logger().info("GeoTiff: filepath: {}, width: {}, height: {}, numChannelsPerPixel: {}, bpp: {}",
      filepath, width, height, numChannelsPerPixel, bpp);

  TIFFClose(data);

  if (bpp != 8 * sizeof(GLfloat)) {
    logger().error("GeoTiff has not 32 bits per pixel!");
  }

  return glm::u32vec4(width, height, numChannelsPerPixel, bpp);
}



void FlowRenderer::loadVelocityTifFiles(std::string const& tifDirectory) {

  // init 3D texture
  glEnable(GL_TEXTURE_3D);
  mVelocity3DTexture = std::make_shared<VistaTexture>(GL_TEXTURE_3D);

  auto               firstFilePath             = tifDirectory + "/step_1.tif";
  const glm::u32vec4 firstTifMeta              = getTifParams(firstFilePath);
  mImageWidth                                  = firstTifMeta.x;
  mImageHeight                                 = firstTifMeta.y;
  const glm::uint32  numChannels               = firstTifMeta.z;
  const glm::uint32  numBitsPerPixelAndChannel = firstTifMeta.w;

  std::vector<float> pixels3D(
      mPluginSettings->mNumTimeSteps * mImageWidth * mImageHeight * numChannels);

  // for (int timestep = 0; timestep < mNumTimeSteps; timestep++) {
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

    int currentImage2DIndex = (timestep - 1) * mImageWidth * mImageHeight * numChannels;

    // std::vector<float> pixels2D(width * height * numChannels);

    for (unsigned y = 0; y < mImageHeight; y++) {
      // if (TIFFReadScanline(data, &pixels2D[width * channels * y], y) < 0) {
      if (TIFFReadScanline(
              data, &pixels3D[currentImage2DIndex + mImageWidth * y * numChannels], y) < 0) {
        logger().error("Failed to read tif!");
      }
    }
    logger().info("{} {}", mImageWidth, mImageHeight);

    TIFFClose(data);
  }

  mVelocity3DTexture->Bind();
  glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA32F, mImageWidth, mImageHeight,
      mPluginSettings->mNumTimeSteps, 0,
      GL_RGB, GL_FLOAT, pixels3D.data());
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  mVelocity3DTexture->Unbind();
}



} // namespace csp::flowvis
