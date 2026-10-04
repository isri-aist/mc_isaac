#pragma once

#include <mc_control/ControllerClient.h>

#include <deque>
#include <map>
#include <string>
#include <vector>

namespace mc_isaac
{

/** In-process mc_rtc GUI client turning the 3D GUI elements into debug lines/points for the Isaac viewport.
 *
 * Editable elements (Point3D, Transform, Rotation) become handles the user can move in Isaac; the new pose is
 * sent back as a regular GUI request (same behaviour as the mc_mujoco 3D client).
 */
struct Markers : public mc_control::ControllerClient
{
  using ElementId = mc_control::ElementId;

  struct Handle
  {
    ElementId requestId;
    std::string kind; // point | transform | rotation
    sva::PTransformd pose;
  };

  /** Refresh from the latest GUI state published by the in-process ControllerServer */
  void update();

  virtual ~Markers() = default;

  /** Scene description sent to the server: {lines: [x1 y1 z1 x2 y2 z2 r g b a width]..., points: [x y z r g b a size]...,
   * handles: [{id, kind, pose: [x y z qw qx qy qz]}]} */
  mc_rtc::Configuration toConfig() const;

  /** Pose of handle key edited in Isaac ([x y z qw qx qy qz]) -> GUI request */
  void request(const std::string & key, const std::vector<double> & pose);

protected:
  void started() override;
  void stopped() override;
  // only 3D elements are drawn in Isaac: ignore the other widgets silently (the base class warns for each)
  void default_impl(const std::string &, const ElementId &) override {}

  void point3d(const ElementId & id,
               const ElementId & requestId,
               bool ro,
               const Eigen::Vector3d & pos,
               const mc_rtc::gui::PointConfig & config) override;
  void trajectory(const ElementId & id,
                  const std::vector<Eigen::Vector3d> & points,
                  const mc_rtc::gui::LineConfig & config) override;
  void trajectory(const ElementId & id,
                  const std::vector<sva::PTransformd> & points,
                  const mc_rtc::gui::LineConfig & config) override;
  void trajectory(const ElementId & id, const Eigen::Vector3d & point, const mc_rtc::gui::LineConfig & config) override;
  void trajectory(const ElementId & id, const sva::PTransformd & point, const mc_rtc::gui::LineConfig & config) override;
  void polygon(const ElementId & id,
               const std::vector<std::vector<Eigen::Vector3d>> & points,
               const mc_rtc::gui::Color & color) override;
  void polygon(const ElementId & id,
               const std::vector<std::vector<Eigen::Vector3d>> & points,
               const mc_rtc::gui::LineConfig & config) override;
  void force(const ElementId & id,
             const ElementId & requestId,
             const sva::ForceVecd & force,
             const sva::PTransformd & pos,
             const mc_rtc::gui::ForceConfig & config,
             bool ro) override;
  void arrow(const ElementId & id,
             const ElementId & requestId,
             const Eigen::Vector3d & start,
             const Eigen::Vector3d & end,
             const mc_rtc::gui::ArrowConfig & config,
             bool ro) override;
  void rotation(const ElementId & id, const ElementId & requestId, bool ro, const sva::PTransformd & pos) override;
  void transform(const ElementId & id, const ElementId & requestId, bool ro, const sva::PTransformd & pos) override;
  void xytheta(const ElementId & id,
               const ElementId & requestId,
               bool ro,
               const Eigen::Vector3d & xytheta,
               double altitude) override;

private:
  /** Debug-draw line [x1 y1 z1 x2 y2 z2 r g b a width] */
  void line(const Eigen::Vector3d & a, const Eigen::Vector3d & b, const mc_rtc::gui::Color & c, double width_px);
  /** Lines between consecutive points, decimated to MAX_TRAJECTORY_POINTS */
  void polyline(const std::vector<Eigen::Vector3d> & points,
                const mc_rtc::gui::Color & c,
                double width_px,
                bool closed = false);
  /** RGB axes of a frame */
  void frame(const sva::PTransformd & pose, double size);
  /** Shaft + two head strokes */
  void arrowLines(const Eigen::Vector3d & start, const Eigen::Vector3d & end, const mc_rtc::gui::ArrowConfig & config);
  /** Editable element: the server shows a sphere the user can move (pose sent back through request()) */
  void addHandle(const ElementId & id, const ElementId & requestId, const std::string & kind, const sva::PTransformd & pose);

  std::vector<char> buffer_;
  size_t lastPublication_ = 0;
  std::chrono::system_clock::time_point lastReceived_ = std::chrono::system_clock::now();
  std::vector<double> lines_, points_;
  std::map<std::string, Handle> handles_;
  /** Real-time trajectories: history accumulated by the client */
  std::map<std::string, std::deque<Eigen::Vector3d>> history_;
  std::vector<std::string> seen_;
};

} // namespace mc_isaac
