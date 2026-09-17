#include <stdexcept>
#include <cmath>
#include <utility>
#include <iostream>
#include "forest_loading.hpp"
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
int main() {
  ImGui::CreateContext();
  try {
    auto& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = ImVec2(1280,800);
    io.DeltaTime = 1.f/60;
    io.Fonts->Build();
    ForestLoadingProgress progress;
    auto render = [&](bool background = false) {
      ImGui::NewFrame();
      if (background) {
        ImGui::SetNextWindowSize(io.DisplaySize);
        ImGui::Begin("Viewer"); ImGui::TextUnformatted("Scene"); ImGui::End();
      }
      bool shown = progress.draw();
      const auto* foreground = ImGui::GetForegroundDrawList();
      ImGui::Render();
      auto* data = ImGui::GetDrawData();
      if (background) require(data->CmdListsCount>0 &&
          data->CmdLists[data->CmdListsCount-1]==foreground,
          "Loading bar was not above the viewer");
      return std::pair<bool,int>{shown,data->TotalVtxCount};
    };
    require(!render().first,"Empty load queue displayed a bar");
    progress.update(10);
    auto pending = render();
    require(pending.first && pending.second>0,"Loading bar generated no visible UI");
    require(progress.fraction()==0.f,"Initial progress is not zero");
    progress.update(4);
    require(progress.total==10 && std::abs(progress.fraction()-.6f)<1e-6f,
            "Progress does not follow finalized asset count");
    require(render(true).second>0,"Partially loaded queue hid the progress bar");
    progress.update(0);
    auto finished = render();
    require(progress.fraction()==1.f && !finished.first && finished.second==0,
            "Completed load left the progress bar visible");
    progress.update(2);
    require(progress.total==2 && progress.fraction()==0.f && render().first,
            "A new load queue did not restart progress");
    ImGui::DestroyContext();
    std::cout << "Verified loading progress UI appears, advances, and disappears\n";
  } catch (const std::exception& error) {
    ImGui::DestroyContext();
    std::cerr << error.what() << '\n'; return 1;
  }
}
