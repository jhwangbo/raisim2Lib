// Prewarm rayrai shader binaries before documentation image generation.
//
// The doc-image CMake target runs this executable once before the individual
// generators so the expensive first-use GLSL compiles populate the persistent
// rayrai shader cache outside the image capture executables.

#include "doc_image_common.hpp"

#include <cstdio>
#include <string>

#include <rayrai/shaders/Shader.hpp>

int main(int argc, char** argv) {
  const std::string shaderCacheDirectory = argc >= 2 ? argv[1] : std::string{};

  doc_image::OffscreenContext gl;
  if (!gl.init("doc_image_shader_warmup")) {
    doc_image::finishAndExit(1);
  }

  Shader::resetBinaryCacheStats();
  std::printf("doc_image: prewarming rayrai shaders\n");

  raisin::RayraiWindow::prewarmShadersForCurrentContext(
      raisin::RayraiWindow::ThreadingMode::SingleThread,
      shaderCacheDirectory);

  const auto stats = Shader::binaryCacheStats();
  std::printf(
      "doc_image: shader warmup cache hits=%llu misses=%llu stores=%llu "
      "coordinated_waits=%llu\n",
      static_cast<unsigned long long>(stats.hits),
      static_cast<unsigned long long>(stats.misses),
      static_cast<unsigned long long>(stats.stores),
      static_cast<unsigned long long>(stats.coordinatedWaits));

  doc_image::finishAndExit(0);
}
