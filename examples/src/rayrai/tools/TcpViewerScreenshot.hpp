// Copyright (c) 2026 Raion Robotics Inc.
// All rights reserved.

#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace raisin
{

class RayraiWindow;

namespace tcp_viewer
{

void flipRgbaRows(std::vector<unsigned char>& rgba, int width, int height);
std::filesystem::path timestampedCapturePath(const std::filesystem::path& dir, const char* prefix);
/**
 * @brief Read the viewer's final colour texture into a top-down RGBA buffer.
 *
 * Shared by the PNG writer and the video encoder so both see identical pixels.
 * @return False (with @p status set) when the render target has no valid size.
 */
bool captureViewerRgba(RayraiWindow& viewer, std::vector<unsigned char>& rgba, int& width,
                       int& height, std::string& status);
bool saveViewerTexturePng(RayraiWindow& viewer, const std::filesystem::path& path, std::string& status);
/** Write an already-captured top-down RGBA buffer to a PNG file. */
bool saveRgbaPng(const std::vector<unsigned char>& rgba, int width, int height,
                 const std::filesystem::path& path, std::string& status);

} // namespace tcp_viewer
} // namespace raisin
