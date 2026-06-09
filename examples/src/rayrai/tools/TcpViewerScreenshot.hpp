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
bool saveViewerTexturePng(RayraiWindow& viewer, const std::filesystem::path& path, std::string& status);

} // namespace tcp_viewer
} // namespace raisin
