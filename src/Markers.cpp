#include "Markers.h"

#include <mc_control/ControllerServer.h>

#include <algorithm>
#include <cmath>
#include <string_view>

namespace mc_isaac
{

namespace
{

constexpr size_t MAX_TRAJECTORY_POINTS = 300;
constexpr size_t MAX_HISTORY = 500;

std::string key(const mc_control::ElementId & id)
{
  std::string out;
  for(const auto & c : id.category) { out += c + "/"; }
  return out + id.name;
}

/** mc_rtc line widths are in meters, debug draw widths in pixels */
double pixels(double width_m)
{
  return std::clamp(width_m * 300.0, 1.0, 8.0);
}

/** Rotation in the usual (Eigen) convention: sva stores its transpose */
Eigen::Matrix3d world_rotation(const sva::PTransformd & pose)
{
  return pose.rotation().transpose();
}

} // namespace

void Markers::update()
{
  // the in-process server keeps its last publication: parse each publication once (accumulated real-time
  // trajectories and plot points would be duplicated otherwise)
  if(server_ != nullptr)
  {
    const auto data = server_->data();
    if(data.second == 0) { return; }
    const auto hash = std::hash<std::string_view>{}(std::string_view(data.first, data.second));
    if(hash == lastPublication_) { return; }
    lastPublication_ = hash;
  }
  run(buffer_, lastReceived_);
}

void Markers::started()
{
  lines_.clear();
  points_.clear();
  handles_.clear();
  seen_.clear();
}

void Markers::stopped()
{
  // forget the history of real-time trajectories that disappeared from the GUI
  for(auto it = history_.begin(); it != history_.end();)
  {
    if(std::find(seen_.begin(), seen_.end(), it->first) == seen_.end()) { it = history_.erase(it); }
    else { ++it; }
  }
}

void Markers::line(const Eigen::Vector3d & a, const Eigen::Vector3d & b, const mc_rtc::gui::Color & c, double width_px)
{
  lines_.insert(lines_.end(), {a.x(), a.y(), a.z(), b.x(), b.y(), b.z(), c.r, c.g, c.b, c.a, width_px});
}

void Markers::polyline(const std::vector<Eigen::Vector3d> & points,
                       const mc_rtc::gui::Color & c,
                       double width_px,
                       bool closed)
{
  if(points.size() < 2) { return; }
  const size_t step = std::max<size_t>(1, points.size() / MAX_TRAJECTORY_POINTS);
  size_t prev = 0;
  for(size_t i = step; i < points.size(); i += step)
  {
    line(points[prev], points[i], c, width_px);
    prev = i;
  }
  if(prev != points.size() - 1) { line(points[prev], points.back(), c, width_px); }
  if(closed) { line(points.back(), points.front(), c, width_px); }
}

void Markers::frame(const sva::PTransformd & pose, double size)
{
  const Eigen::Matrix3d R = world_rotation(pose);
  const Eigen::Vector3d & o = pose.translation();
  line(o, o + size * R.col(0), {1, 0, 0}, 3.0);
  line(o, o + size * R.col(1), {0, 1, 0}, 3.0);
  line(o, o + size * R.col(2), {0, 0, 1}, 3.0);
}

void Markers::arrowLines(const Eigen::Vector3d & start,
                         const Eigen::Vector3d & end,
                         const mc_rtc::gui::ArrowConfig & config)
{
  const Eigen::Vector3d dir = end - start;
  const double length = dir.norm();
  if(length < 1e-9) { return; }
  const double width = pixels(config.shaft_diam);
  line(start, end, config.color, width);
  // two head strokes in a plane containing the arrow
  const Eigen::Vector3d u = dir / length;
  Eigen::Vector3d n = u.cross(Eigen::Vector3d::UnitZ());
  if(n.norm() < 1e-6) { n = u.cross(Eigen::Vector3d::UnitX()); }
  n.normalize();
  const double head = std::min(config.head_len, 0.5 * length);
  const Eigen::Vector3d base = end - head * u;
  line(end, base + 0.5 * config.head_diam * n, config.color, width);
  line(end, base - 0.5 * config.head_diam * n, config.color, width);
}

void Markers::addHandle(const ElementId & id,
                        const ElementId & requestId,
                        const std::string & kind,
                        const sva::PTransformd & pose)
{
  handles_[key(id)] = Handle{requestId, kind, pose};
}

void Markers::point3d(const ElementId & id,
                      const ElementId & requestId,
                      bool ro,
                      const Eigen::Vector3d & pos,
                      const mc_rtc::gui::PointConfig & config)
{
  const auto & c = config.color;
  points_.insert(points_.end(), {pos.x(), pos.y(), pos.z(), c.r, c.g, c.b, c.a, std::clamp(config.scale * 500.0, 4.0, 20.0)});
  if(!ro) { addHandle(id, requestId, "point", sva::PTransformd(pos)); }
}

void Markers::trajectory(const ElementId &,
                         const std::vector<Eigen::Vector3d> & points,
                         const mc_rtc::gui::LineConfig & config)
{
  polyline(points, config.color, pixels(config.width));
}

void Markers::trajectory(const ElementId &,
                         const std::vector<sva::PTransformd> & points,
                         const mc_rtc::gui::LineConfig & config)
{
  std::vector<Eigen::Vector3d> positions;
  positions.reserve(points.size());
  for(const auto & p : points) { positions.push_back(p.translation()); }
  polyline(positions, config.color, pixels(config.width));
}

void Markers::trajectory(const ElementId & id, const Eigen::Vector3d & point, const mc_rtc::gui::LineConfig & config)
{
  const auto k = key(id);
  seen_.push_back(k);
  auto & history = history_[k];
  history.push_back(point);
  while(history.size() > MAX_HISTORY) { history.pop_front(); }
  polyline({history.begin(), history.end()}, config.color, pixels(config.width));
}

void Markers::trajectory(const ElementId & id, const sva::PTransformd & point, const mc_rtc::gui::LineConfig & config)
{
  trajectory(id, Eigen::Vector3d(point.translation()), config);
}

void Markers::polygon(const ElementId & id,
                      const std::vector<std::vector<Eigen::Vector3d>> & points,
                      const mc_rtc::gui::Color & color)
{
  polygon(id, points, mc_rtc::gui::LineConfig(color));
}

void Markers::polygon(const ElementId &,
                      const std::vector<std::vector<Eigen::Vector3d>> & points,
                      const mc_rtc::gui::LineConfig & config)
{
  for(const auto & poly : points) { polyline(poly, config.color, pixels(config.width), true); }
}

void Markers::force(const ElementId &,
                    const ElementId &,
                    const sva::ForceVecd & force,
                    const sva::PTransformd & pos,
                    const mc_rtc::gui::ForceConfig & config,
                    bool)
{
  const Eigen::Vector3d start = pos.translation();
  arrowLines(start, start + config.force_scale * force.force(), config);
}

void Markers::arrow(const ElementId &,
                    const ElementId &,
                    const Eigen::Vector3d & start,
                    const Eigen::Vector3d & end,
                    const mc_rtc::gui::ArrowConfig & config,
                    bool)
{
  arrowLines(start, end, config);
}

void Markers::rotation(const ElementId & id, const ElementId & requestId, bool ro, const sva::PTransformd & pos)
{
  frame(pos, 0.1);
  if(!ro) { addHandle(id, requestId, "rotation", pos); }
}

void Markers::transform(const ElementId & id, const ElementId & requestId, bool ro, const sva::PTransformd & pos)
{
  frame(pos, 0.1);
  if(!ro) { addHandle(id, requestId, "transform", pos); }
}

void Markers::xytheta(const ElementId &, const ElementId &, bool, const Eigen::Vector3d & xytheta, double altitude)
{
  const Eigen::Matrix3d R = Eigen::AngleAxisd(xytheta.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  frame(sva::PTransformd(Eigen::Matrix3d(R.transpose()), Eigen::Vector3d(xytheta.x(), xytheta.y(), altitude)), 0.1);
}

mc_rtc::Configuration Markers::toConfig() const
{
  mc_rtc::Configuration out;
  out.add("lines", lines_);
  out.add("points", points_);
  auto handles = out.array("handles");
  for(const auto & [k, h] : handles_)
  {
    const Eigen::Quaterniond q(world_rotation(h.pose));
    const auto & t = h.pose.translation();
    auto entry = handles.object();
    entry.add("id", k);
    entry.add("kind", h.kind);
    entry.add("pose", std::vector<double>{t.x(), t.y(), t.z(), q.w(), q.x(), q.y(), q.z()});
  }
  return out;
}

void Markers::request(const std::string & k, const std::vector<double> & pose)
{
  auto it = handles_.find(k);
  if(it == handles_.end() || pose.size() != 7) { return; }
  const auto & h = it->second;
  const Eigen::Vector3d t(pose[0], pose[1], pose[2]);
  const Eigen::Quaterniond q = Eigen::Quaterniond(pose[3], pose[4], pose[5], pose[6]).normalized();
  // same request data as the mc_mujoco 3D client (sva rotations are transposed with respect to Eigen)
  const sva::PTransformd X(q.inverse(), t);
  if(h.kind == "point") { send_request(h.requestId, t); }
  else if(h.kind == "rotation") { send_request(h.requestId, Eigen::Matrix3d(X.rotation())); }
  else { send_request(h.requestId, X); }
}

} // namespace mc_isaac
