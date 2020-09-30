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



const char* FlowRenderer::PARTICLES_SEED_AND_DISPLACE_VERT = R"(
  #version 330

  // inputs
  layout(location = 0) in vec2 vPosition;

  // uniforms
  // none ...

  // outputs
  out VaryingStruct {
    vec2 vTexcoords;
  } vsOut;


  void main() {
    
    // for lookups in the depth and color buffers
    vsOut.vTexcoords = vPosition * 0.5 + 0.5;

    // no tranformation here since we draw a full screen quad
    gl_Position = vec4(vPosition, 0, 1);

  }
)";

////////////////////////////////////////////////////////////////////////////////////////////////////

const char* FlowRenderer::PARTICLES_SEED_AND_DISPLACE_FRAG = R"(
  #version 330

  // inputs
  in VaryingStruct {
    vec2 vTexcoords;
  } vsIn;


  // The least recently updated "random particle ping-pong image".
  // (The other one is the current render target.)
  uniform sampler2D uParticlesImage;

  //interpolatable stack of 2D textures == 2D + time dimension == 2+1 D == 3D
  uniform sampler3D uVelocity3DTexture;
  uniform float     uNumTimeSteps;
  // point in time to sample the stack of flow-textures;
  uniform float     uRelativeTime;
  // time interval to "Newton integrate" since last visual render frame:
  uniform float     uDurationSinceLastFrame;
  // non-physical, user-definable scale factor in order to make the flow speeds visually distinguishable
  uniform float     uFlowSpeedScale;


// outputs
// TODO check if float can work
layout(location = 0) out float oColor;

    
void main()
{
    
    vec3 texcoords3D = vec3(vsIn.vTexcoords.x, vsIn.vTexcoords.y, uRelativeTime);   
    vec2 velocity = texture(uVelocity3DTexture, texcoords3D).rg;

    vec2 scaledVelocity = velocity * uFlowSpeedScale;

    vec2 displacement = scaledVelocity * uDurationSinceLastFrame;
    
    //TODO test this:
    //float aspectRatio_H_V = float(textureSize(uParticlesImage).x)
    //                      / float(textureSize(uParticlesImage).y);
    float aspectRatio_H_V = 1.0;
           
    //compensate for aspectration: downscale displacement horizontal tex coord by aspect ratio
    displacement.x /= aspectRatio_H_V;

    // pull from inverse direction of diplacement vector: hence MINUS:
    vec2 particlePosToPull = vsIn.vTexcoords.xy - displacement.xy;

    oColor = texture(uParticlesImage, particlePosToPull).r;
   
    //dummy value, not sure if needed to write:
    gl_FragDepth = 0.5;

}
)";

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

  initParticleAnimationFBO();
  initParticleAnimationVAO();
  initParticleAnimationShader();

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

void FlowRenderer::initParticleAnimationFBO() {

  mFBO = std::make_shared<VistaFramebufferObj>();
  
  mFBOdepthBufferTex = std::make_shared<VistaTexture>(GL_TEXTURE_2D);
  mFBOdepthBufferTex->Bind();
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, mImageWidth, mImageHeight, 0,
      GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);

  mFBO->Attach(mFBOdepthBufferTex.get(), GL_DEPTH_ATTACHMENT);
  mFBOdepthBufferTex->Unbind();

}

void FlowRenderer::initParticleAnimationVAO() {
  // create quad -------------------------------------------------------------
  std::array<float, 8> const data{-1, 1, 1, 1, -1, -1, 1, -1};

  mQuadVBO.Bind(GL_ARRAY_BUFFER);
  mQuadVBO.BufferData(data.size() * sizeof(float), data.data(), GL_STATIC_DRAW);
  mQuadVBO.Release();

  // positions
  mQuadVAO.EnableAttributeArray(0);
  mQuadVAO.SpecifyAttributeArrayFloat(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), 0, &mQuadVBO);
}

void FlowRenderer::initParticleAnimationShader() {

  mParticleAnimationShader = VistaGLSLShader();

  std::string sVert(PARTICLES_SEED_AND_DISPLACE_VERT);
  std::string sFrag(PARTICLES_SEED_AND_DISPLACE_FRAG);

  mParticleAnimationShader.InitVertexShaderFromString(sVert);
  mParticleAnimationShader.InitFragmentShaderFromString(sFrag);

  mParticleAnimationShader.Link();
}

void FlowRenderer::setupParticleAnimationShaderUniforms() {

  mParticleAnimationShader.Bind();

  mParticleAnimationShader.SetUniform(mParticleAnimationShader.GetUniformLocation("uNumTimeSteps"),
      static_cast<float>(mPluginSettings->mNumTimeSteps));

  // TODO outsource this logic to FlowRenderer
  // in [0..1]
  double startDate_spice = cs::utils::convert::time::toSpice(mPluginSettings->mStartDate);
  double endDate_spice   = cs::utils::convert::time::toSpice(mPluginSettings->mEndDate);
  double relativeTime =
      (getCurrentTime() - startDate_spice) / (endDate_spice - startDate_spice);
  mParticleAnimationShader.SetUniform(mParticleAnimationShader.GetUniformLocation("uRelativeTime"),
      static_cast<float>(relativeTime));

  double uDurationSinceLastFrame =
      getCurrentTime() - getLastVisualRenderTime();
  mParticleAnimationShader.SetUniform(
      mParticleAnimationShader.GetUniformLocation("uDurationSinceLastFrame"),
      static_cast<float>(uDurationSinceLastFrame));

  mParticleAnimationShader.SetUniform(
      mParticleAnimationShader.GetUniformLocation("uFlowSpeedScale"),
      static_cast<float>(mPluginSettings->mFlowSpeedScale));

  // uParticlesImage
  mParticleAnimationShader.SetUniform(
      mParticleAnimationShader.GetUniformLocation("uParticlesImage"), 0);
  getCurrentParticleTexToReadFrom()->Bind(GL_TEXTURE0);

  // 3D texture:
  mParticleAnimationShader.SetUniform(
      mParticleAnimationShader.GetUniformLocation("uVelocity3DTexture"), 1);
  getVelocity3DTexture()->Bind(GL_TEXTURE1);

}



void FlowRenderer::renderParticleAnimation() {

    //boilerplate code stolen from Shadows.cpp

    // save current viewport
    std::array<GLint, 4> iOrigViewport{};
    glGetIntegerv(GL_VIEWPORT, iOrigViewport.data());


    // bind the fbo
    mFBO->Bind();

    //bind current write-pingpong texture to FBO's color attachment:
    mFBO->Attach(getCurrentRenderTarget().get(), GL_COLOR_ATTACHMENT0);

    // setup viewport
    glViewport(0, 0, mImageWidth, mImageHeight);

    // clear fbo
    glClear(GL_DEPTH_BUFFER_BIT);

    // bind shader  ----------------------------------------
    mParticleAnimationShader.Bind();
    // set uniforms
    setupParticleAnimationShaderUniforms();
    
    // draw "fullscreen" quad via VAO-----------------------------------------
    mQuadVAO.Bind();
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    mQuadVAO.Release();



    // unbind fbo again
    mFBO->Release();

    mParticleAnimationShader.Release();

    // restore previous viewport
    glViewport(iOrigViewport.at(0), iOrigViewport.at(1), iOrigViewport.at(2), iOrigViewport.at(3));


    //let them ping pongs switch roles
    togglePingPongTex();
}

void FlowRenderer::seedParticleTexture() {
  
  static int currentSeedCycleTime = 0;
  currentSeedCycleTime++;

  //// hack in order so not update every frame;
  //static int currentSeedCycleTime = 0;
  //const int  numFramesBetweenReseed = 60;
  //if (currentSeedCycleTime >= numFramesBetweenReseed) {
  //  currentSeedCycleTime = 0;
  //} else {
  //  currentSeedCycleTime++;
  //  return;
  //}

  //glFlush();

  logger().debug("particle seeding #{}", currentSeedCycleTime);

  getCurrentParticleTexToReadFrom()->Bind();

  //hack: read back tex from GPU:
  glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, mSeedTextureHostData.data());

  //glFlush();

  const GLint                        numCells = mImageWidth * mImageHeight;
  std::uniform_int_distribution<rng_type::result_type> udist(0, numCells -1);

  const GLint numNewParticles =
      static_cast <GLint>(static_cast<float>(numCells) * mPluginSettings->mParticleSeedThreshold);
  //seed to random locations:
  for (size_t i = 0; i < numNewParticles; i++) {
    mSeedTextureHostData[udist(mRNG)] = 0.0;
    mSeedTextureHostData[udist(mRNG)] = 1.0;
  }

  //glFlush();

  // hack: re-upload
  glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, mImageWidth, mImageHeight, 0, GL_RED, GL_FLOAT,
      mSeedTextureHostData.data());

  //glFlush();

  getCurrentParticleTexToReadFrom()->Unbind();
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
