// Copyright (c) 2026 Raion Robotics Inc. All rights reserved.
#include "TcpViewerSimulation.hpp"
#include "tinyxml_rai/tinyxml_rai.h"
#include <chrono>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>
#include <cerrno>
#include <cstring>
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
extern char** environ;
#endif

namespace raisin::tcp_viewer {
namespace fs = std::filesystem;
namespace {
using Clock = std::chrono::steady_clock;
fs::path temporaryDirectory() {
  const auto stamp = Clock::now().time_since_epoch().count();
  for (int i = 0; i < 100; ++i) {
    auto path = fs::temp_directory_path() / ("rayrai-simulation-" + std::to_string(stamp) + "-" + std::to_string(i));
    if (fs::create_directory(path)) return path;
  }
  throw std::runtime_error("cannot create simulation working directory");
}
std::string readFailure(const fs::path& directory) {
  std::ifstream in(directory / "error.txt");
  std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
  std::ifstream log(directory / "process.log");
  log.seekg(0, std::ios::end);
  const std::streamoff size = log.tellg();
  if (size > 0) {
    log.seekg(size > 1800 ? size - std::streamoff(1800) : std::streamoff(0));
    text += std::string(std::istreambuf_iterator<char>(log), std::istreambuf_iterator<char>());
  }
  if (text.size() > 2000) text = text.substr(text.size() - 2000);
  return text.empty() ? "simulation process exited before the world could be loaded" : text;
}
#if defined(_WIN32)
std::wstring quoteArgument(const std::wstring& value) {
  std::wstring out = L"\"";
  size_t slashes = 0;
  for (wchar_t c : value) {
    if (c == L'\\') { ++slashes; continue; }
    out.append(c == L'\"' ? slashes * 2 + 1 : slashes, L'\\');
    slashes = 0; out += c;
  }
  out.append(slashes * 2, L'\\'); out += L'\"';
  return out;
}
#endif
} // namespace

DroppedSceneKind classifyDroppedScene(const fs::path& path, std::string& error) {
  error.clear();
  std::error_code ec;
  if (!fs::is_regular_file(path, ec)) { error = "File not found: " + path.string(); return DroppedSceneKind::Invalid; }
  raisim::TiXmlDocument doc(path.string().c_str());
  if (!doc.LoadFile() || !doc.RootElement()) {
    error = "Cannot read scene XML: " + path.string(); return DroppedSceneKind::Invalid;
  }
  const std::string root = doc.RootElement()->Value();
  if (root == "raisim") return DroppedSceneKind::World;
  if (root == "robot" || root == "mujoco") return DroppedSceneKind::Robot;
  error = "Expected a RaiSim world, URDF robot, or MJCF model";
  return DroppedSceneKind::Invalid;
}

struct LocalSimulation::Impl {
  State state = State::Idle;
  fs::path directory, xml;
  std::string error;
  int port = 0;
  Clock::time_point started;
#if defined(_WIN32)
  HANDLE process = nullptr, job = nullptr, input = nullptr;
#else
  pid_t process = -1;
  int input = -1;
#endif
};

LocalSimulation::LocalSimulation() : impl_(std::make_unique<Impl>()) {}
LocalSimulation::~LocalSimulation() { stop(); }
LocalSimulation::State LocalSimulation::state() const { return impl_->state; }
bool LocalSimulation::active() const { return state() == State::Starting || state() == State::Running; }
int LocalSimulation::port() const { return impl_->port; }
const std::string& LocalSimulation::error() const { return impl_->error; }
const fs::path& LocalSimulation::worldPath() const { return impl_->xml; }

bool LocalSimulation::start(const fs::path& executable, const fs::path& xml,
                            const fs::path& activationKey, std::string& error) {
  if (classifyDroppedScene(xml, error) != DroppedSceneKind::World) {
    if (error.empty()) error = "Simulation requires a RaiSim world XML file";
    return false;
  }
  std::error_code ec;
  if (!fs::is_regular_file(executable, ec)) { error = "Viewer executable not found"; return false; }
  stop();
  auto& p = *impl_;
  try {
    p.directory = temporaryDirectory();
    p.xml = fs::absolute(xml);
    const fs::path ready = p.directory / "ready.txt";
    const fs::path key = activationKey.empty() ? fs::path{} : fs::absolute(activationKey);
#if defined(_WIN32)
    SECURITY_ATTRIBUTES security{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
    HANDLE readPipe = nullptr;
    if (!CreatePipe(&readPipe, &p.input, &security, 0)) throw std::runtime_error("cannot create simulation pipe");
    SetHandleInformation(p.input, HANDLE_FLAG_INHERIT, 0);
    HANDLE log = CreateFileW((p.directory / "process.log").c_str(), GENERIC_WRITE,
                            FILE_SHARE_READ | FILE_SHARE_WRITE, &security, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (log == INVALID_HANDLE_VALUE) { CloseHandle(readPipe); throw std::runtime_error("cannot open simulation log"); }
    SIZE_T bytes = 0;
    InitializeProcThreadAttributeList(nullptr, 1, 0, &bytes);
    std::vector<unsigned char> attributes(bytes);
    STARTUPINFOEXW startup{};
    startup.StartupInfo.cb = sizeof(startup);
    startup.lpAttributeList = reinterpret_cast<LPPROC_THREAD_ATTRIBUTE_LIST>(attributes.data());
    const bool initialized = InitializeProcThreadAttributeList(startup.lpAttributeList, 1, 0, &bytes) != 0;
    HANDLE inherited[]{readPipe, log};
    bool prepared = initialized && UpdateProcThreadAttribute(startup.lpAttributeList, 0,
        PROC_THREAD_ATTRIBUTE_HANDLE_LIST, inherited, sizeof(inherited), nullptr, nullptr);
    startup.StartupInfo.dwFlags = STARTF_USESTDHANDLES;
    startup.StartupInfo.hStdInput = readPipe;
    startup.StartupInfo.hStdOutput = startup.StartupInfo.hStdError = log;
    p.job = CreateJobObjectW(nullptr, nullptr);
    JOBOBJECT_EXTENDED_LIMIT_INFORMATION limits{};
    limits.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
    prepared = prepared && p.job && SetInformationJobObject(p.job, JobObjectExtendedLimitInformation, &limits, sizeof(limits));
    std::wstring command = quoteArgument(executable.wstring()) + L" --xml-simulation-worker " +
        quoteArgument(p.xml.wstring()) + L" " + quoteArgument(ready.wstring()) + L" " + quoteArgument(key.wstring());
    PROCESS_INFORMATION process{};
    const bool launched = prepared && CreateProcessW(executable.c_str(), command.data(), nullptr, nullptr, TRUE,
        CREATE_NO_WINDOW | CREATE_SUSPENDED | EXTENDED_STARTUPINFO_PRESENT, nullptr, nullptr, &startup.StartupInfo, &process);
    if (initialized) DeleteProcThreadAttributeList(startup.lpAttributeList);
    CloseHandle(readPipe); CloseHandle(log);
    if (!launched) throw std::runtime_error("cannot launch simulation process (Windows error " + std::to_string(GetLastError()) + ")");
    p.process = process.hProcess;
    if (!AssignProcessToJobObject(p.job, p.process)) {
      CloseHandle(process.hThread);
      throw std::runtime_error("cannot attach simulation process to viewer lifetime");
    }
    ResumeThread(process.hThread); CloseHandle(process.hThread);
#else
    int pipeFds[2];
    if (pipe(pipeFds) != 0) throw std::runtime_error("cannot create simulation pipe");
    p.input = pipeFds[1];
    fcntl(pipeFds[0], F_SETFD, FD_CLOEXEC); fcntl(pipeFds[1], F_SETFD, FD_CLOEXEC);
    const int log = open((p.directory / "process.log").c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    if (log < 0) { close(pipeFds[0]); throw std::runtime_error("cannot open simulation log"); }
    std::vector<std::string> args{executable.string(), "--xml-simulation-worker", p.xml.string(), ready.string(), key.string()};
    std::vector<char*> pointers;
    for (auto& arg : args) pointers.push_back(arg.data());
    pointers.push_back(nullptr);
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, pipeFds[0], STDIN_FILENO);
    posix_spawn_file_actions_adddup2(&actions, log, STDOUT_FILENO);
    posix_spawn_file_actions_adddup2(&actions, log, STDERR_FILENO);
    posix_spawn_file_actions_addclose(&actions, pipeFds[0]);
    posix_spawn_file_actions_addclose(&actions, pipeFds[1]);
    posix_spawn_file_actions_addclose(&actions, log);
    const int result = posix_spawn(&p.process, executable.c_str(), &actions, nullptr, pointers.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    close(pipeFds[0]); close(log);
    if (result != 0) { p.process = -1; throw std::runtime_error("cannot launch simulation process: " + std::string(std::strerror(result))); }
#endif
    p.state = State::Starting;
    p.started = Clock::now();
    error.clear();
    return true;
  } catch (const std::exception& e) {
    error = e.what(); stop(); p.state = State::Failed; p.error = error; return false;
  }
}

void LocalSimulation::poll() {
  auto& p = *impl_;
  if (!active()) return;
#if defined(_WIN32)
  const bool exited = WaitForSingleObject(p.process, 0) == WAIT_OBJECT_0;
#else
  int status = 0;
  const pid_t result = waitpid(p.process, &status, WNOHANG);
  const bool exited = result == p.process || (result < 0 && errno == ECHILD);
  if (exited) p.process = -1;
#endif
  if (exited) {
    const auto failure = readFailure(p.directory);
    stop(); p.state = State::Failed; p.error = failure; return;
  }
  if (p.state == State::Starting) {
    int port = 0;
    std::ifstream ready(p.directory / "ready.txt");
    if (ready >> port && port > 0 && port <= 65535) { p.port = port; p.state = State::Running; }
    // Long model imports remain asynchronous but a hung child cannot wait forever.
    else if (Clock::now() - p.started > std::chrono::minutes(5)) {
      stop(); p.state = State::Failed; p.error = "Simulation startup timed out";
    }
  }
}

void LocalSimulation::stop() {
  auto& p = *impl_;
#if defined(_WIN32)
  if (p.input) { CloseHandle(p.input); p.input = nullptr; }
  if (p.process) {
    if (WaitForSingleObject(p.process, 200) == WAIT_TIMEOUT) TerminateProcess(p.process, 0);
    WaitForSingleObject(p.process, 1000); CloseHandle(p.process); p.process = nullptr;
  }
  if (p.job) { CloseHandle(p.job); p.job = nullptr; }
#else
  if (p.input >= 0) { close(p.input); p.input = -1; }
  if (p.process > 0) {
    kill(p.process, SIGTERM);
    int status = 0;
    const auto deadline = Clock::now() + std::chrono::milliseconds(200);
    pid_t result;
    do {
      result = waitpid(p.process, &status, WNOHANG);
      if (result == p.process || (result < 0 && errno == ECHILD)) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (Clock::now() < deadline);
    if (result == 0 || (result < 0 && errno == EINTR)) {
      kill(p.process, SIGKILL);
      while (waitpid(p.process, &status, 0) < 0 && errno == EINTR) {}
    }
    p.process = -1;
  }
#endif
  if (!p.directory.empty()) { std::error_code ignored; fs::remove_all(p.directory, ignored); }
  p.directory.clear(); p.xml.clear(); p.error.clear(); p.port = 0; p.state = State::Idle;
}

fs::path viewerExecutablePath(const char* argv0) {
#if defined(_WIN32)
  std::vector<wchar_t> path(32768);
  const DWORD size = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
  if (size && size < path.size()) return fs::path(std::wstring(path.data(), size));
#elif defined(__APPLE__)
  uint32_t size = 0;
  _NSGetExecutablePath(nullptr, &size);
  std::vector<char> path(size);
  if (_NSGetExecutablePath(path.data(), &size) == 0) return fs::weakly_canonical(path.data());
#else
  std::error_code error;
  const auto path = fs::read_symlink("/proc/self/exe", error);
  if (!error) return path;
#endif
  return fs::absolute(argv0 ? argv0 : "rayrai_tcp_viewer");
}
} // namespace raisin::tcp_viewer
