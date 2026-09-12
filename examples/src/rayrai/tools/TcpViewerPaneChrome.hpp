// Copyright (c) 2026 Raion Robotics Inc.
// All rights reserved.
#pragma once

#include <imgui/imgui.h>

namespace raisin::tcp_viewer {

// Use a regular window layer: the viewport foreground also covers popups.
inline ImDrawList* paneChromeDrawList(const ImVec2& size) {
  ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
  ImGui::SetNextWindowSize(size);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  ImGui::Begin("##pane_chrome", nullptr,
               ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
               ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings |
               ImGuiWindowFlags_NoFocusOnAppearing);
  ImDrawList* drawList = ImGui::GetWindowDrawList();
  ImGui::End();
  ImGui::PopStyleVar();
  return drawList;
}

} // namespace raisin::tcp_viewer
