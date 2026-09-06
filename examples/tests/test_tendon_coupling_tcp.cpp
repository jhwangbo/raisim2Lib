// Exercise the actual server serializer and viewer parser without a socket thread.
#include "tendon_example_runner.hpp"
#include "rayrai/example_common.hpp"
#include "rayrai/RaisimTcpCommon.hpp"
#include <glm/gtc/quaternion.hpp>

namespace raisim {
class RaisimServerProtocolTestAccess {
 public:
  static std::vector<char> snapshot(RaisimServer& server) {
    auto* begin=server.send_buffer.data();
    server.data_=server::set(begin,RaisimServer::version_,RaisimServer::PROTOCOL_SUPPORTED_FEATURES,
                            int32_t(0),int32_t(0));
    server.update(RaisimServer::PROTOCOL_SUPPORTED_FEATURES);
    return {begin,server.data_};
  }
  static uint32_t tag(RaisimServer& server,const std::string& name) {
    return server.tendonVisualTags_.at(name);
  }
};
}
using namespace raisim_examples::tendons;
int main(int argc,char** argv) {
  try {
    const auto options=parseOptions(argc,argv,Kind::Coupling);
    if (!options.key.empty()) World::setActivationKey(options.key);
    ExampleApp app;
    if (!app.init("Coupled tendon TCP check",320,240,false)) return 77;
    struct Cleanup { ExampleApp& app; ~Cleanup(){app.shutdown();} } cleanup{app};
    ImGui::GetIO().IniFilename=nullptr;
    Scene scene(Kind::Coupling);
    raisim::RaisimServer server(scene.world.get());
    auto viewer=std::make_shared<raisin::RayraiWindow>(std::make_shared<World>(),320,240);
    raisin::tcp_viewer::RemoteScene remote(viewer); remote.setVerbose(false);
    for (int frame=0;frame<20;++frame) {
      for (int step=0;step<100;++step) scene.step();
      const auto bytes=raisim::RaisimServerProtocolTestAccess::snapshot(server);
      raisin::tcp_viewer::BufferReader reader(bytes);
      std::vector<raisin::tcp_viewer::PendingSensorUpdate> pending;
      if (!remote.applyResponse(reader,pending) || !reader.ok)
        throw std::runtime_error("cannot decode coupling example server scene");
      for (const auto& name:{"joint_a","joint_b"}) {
        auto* cable=scene.world->getTendon(name); cable->updateGeometry(true);
        auto visual=viewer->findInstancedVisuals("remote_inst_"+
          std::to_string(raisim::RaisimServerProtocolTestAccess::tag(server,name)));
        if (!visual || visual->count()!=1 || cable->getVisualSegments().size()!=1)
          throw std::runtime_error("TCP viewer must contain both coupled cable segments");
        const auto& segment=cable->getVisualSegments().front();
        const auto p=visual->getPosition(0),s=visual->getScale(0);
        const auto q=visual->getOrientation(0);
        const auto half=glm::quat(q.x,q.y,q.z,q.w)*glm::vec3(0.f,0.f,0.5f*s.z);
        const glm::vec3 a(segment.start[0],segment.start[1],segment.start[2]);
        const glm::vec3 b(segment.end[0],segment.end[1],segment.end[2]);
        const auto& color=cable->getProperties().color;
        if (glm::length(p-half-a)>2e-5f || glm::length(p+half-b)>2e-5f ||
            std::abs(s.x-cable->getProperties().width)>1e-6 ||
            glm::length(visual->getColor1()-glm::vec4(color[0],color[1],color[2],color[3]))>1e-6f)
          throw std::runtime_error("TCP cable drawing differs from its physics route or appearance");
      }
    }
    std::cout << "Both coupled cables survive server serialization and viewer updates.\n";
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
