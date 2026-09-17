#pragma once
#include <stdexcept>
#include <vector>
#include "rayrai/example_common.hpp"
#include "rayrai/Visuals.hpp"

// Exercise the real example configuration, not a separately authored scene.
inline void checkForestShadows(ExampleApp& app, raisin::RayraiWindow& viewer,
    const std::vector<std::shared_ptr<raisin::InstancedVisuals>>& foliage) {
  for (const auto& visual : foliage) {
    if (!visual->castsShadows())
      throw std::runtime_error("Forest foliage batch does not cast shadows");
    // Freeze vegetation so the image difference measures shadows, not wind.
    visual->configureFoliageWind(0,1,0,1,0);
    visual->setFoliageImpostorPolicy(false);
  }
  auto render = [&] {
    // Let temporal AO settle after changing the caster set. Physics stays paused.
    for (int i=0;i<4;++i) {
      app.beginFrame(); app.renderViewer(viewer); app.endFrame();
    }
    gl::glFinish();
    gl::glBindTexture(gl::GL_TEXTURE_2D,viewer.getCamera().getFinalTexture());
    gl::GLint w=0,h=0;
    gl::glGetTexLevelParameteriv(gl::GL_TEXTURE_2D,0,gl::GL_TEXTURE_WIDTH,&w);
    gl::glGetTexLevelParameteriv(gl::GL_TEXTURE_2D,0,gl::GL_TEXTURE_HEIGHT,&h);
    if (w<=0 || h<=0) throw std::runtime_error("Empty shadow test image");
    std::vector<unsigned char> pixels(static_cast<size_t>(w)*h*4);
    gl::glGetTexImage(gl::GL_TEXTURE_2D,0,gl::GL_RGBA,gl::GL_UNSIGNED_BYTE,pixels.data());
    return pixels;
  };
  const auto shadowed=render();
  for (const auto& visual : foliage) visual->setCastsShadows(false);
  const auto unshadowed=render();
  if (shadowed.size()!=unshadowed.size()) throw std::runtime_error("Capture size changed");
  size_t darker=0;
  for (size_t i=0;i<shadowed.size();i+=4) {
    const int delta=int(unshadowed[i])+unshadowed[i+1]+unshadowed[i+2]
                  -int(shadowed[i])-shadowed[i+1]-shadowed[i+2];
    if (delta>36) ++darker; // More than 12 levels of average RGB darkening.
  }
  for (const auto& visual : foliage) visual->setCastsShadows(true);
  std::cout << "Foliage shadow pixels: " << darker << '\n';
  if (darker<shadowed.size()/4/200)
    throw std::runtime_error("Foliage shadows did not darken at least 0.5% of the image");
}
