// Copyright 2026 TDC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MUJOCO_DOCK_SIM_PLUGIN__DOCK_SIM_PLUGIN_HPP_
#define MUJOCO_DOCK_SIM_PLUGIN__DOCK_SIM_PLUGIN_HPP_

#include <atomic>
#include <condition_variable>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include "mujoco_dock_sim_plugin/action/dock.hpp"
#include "mujoco_dock_sim_plugin/action/release.hpp"
#include "mujoco_dock_sim_plugin/srv/dock.hpp"
#include "mujoco_dock_sim_plugin/srv/release.hpp"
#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_dock_sim_plugin
{

/**
 * @brief Simulates docking between pairs of MuJoCo *sites* by driving weld equality
 *        constraints.
 *
 * Concept
 * -------
 * The plugin owns a pool of weld equality constraints ("the weld pool").  Every pool
 * entry can be re-pointed to an arbitrary pair of bodies and given an arbitrary target
 * relative pose at runtime, and can be activated / deactivated at will.  A docking
 * interface is a pair of sites; the plugin resolves each site to the rigid body it is
 * attached to and welds those two bodies.
 *
 * A group describes one family of interfaces:
 *   * `sites`        - the sites that belong to the group (a site may be in several),
 *   * `male_sites` / `female_sites` - optional gender split.  When a group defines
 *     either list only a male/female pair may dock.  When neither is defined the group
 *     is genderless and any two of its sites may dock.
 *   * `position_tolerance` / `rotation_tolerance` - how close the two sites must be to
 *     the docking target for a dock to start / to stay valid,
 *   * `dock_target` - the target relative pose of site2 expressed in site1's frame:
 *     `position` (3) plus `quaternion` (4, in (w, x, y, z) order),
 *   * `dock_target.symmetry` - which other orientations of that target also count as
 *     docked: `mode` (`none` / `discrete` / `continuous`), `axis` (`x` / `y` / `z`, in
 *     site1's frame) and `angles_deg` (discrete mode).  The pair is welded to the
 *     *nearest* accepted orientation, never to a phase the hardware is not in,
 *   * `dock_time` / `release_time` - how long the relative pose must stay inside the
 *     tolerance before a dock is declared successful (and how long a release takes).
 *
 * Site frames
 * -----------
 * The plugin never looks at the meaning of a site frame, but the whole scenario is
 * easiest to describe when every interface follows the same convention:
 *
 *   * `+z` is the **outward normal of the mating plane** (it points away from the body
 *     the interface belongs to, i.e. towards the part it mates with), and
 *   * `+x` is the **alignment direction** of the connector (e.g. the long edge of a
 *     rectangular boss / slot).
 *
 * Two mated interfaces then have anti-parallel z axes, and for a rectangular connector
 * the relative orientation of a mated pair is a 180 deg rotation about the y axis, i.e.
 * `dock_target.quaternion = (0, 0, 1, 0)` (z opposite, x opposite, y parallel).
 *
 * Docking sequence (per pair):
 *   IDLE -> (relative pose inside tolerance) -> WAIT_DOCKING
 *        -> (stays inside tolerance for dock_time) -> welded, DOCKED
 *        -> (leaves tolerance) -> back to IDLE
 * Release sequence:
 *   DOCKED -> WAIT_RELEASING -> (release_time elapsed) -> weld deactivated -> IDLE
 *
 * Threading
 * ---------
 * Service / action callbacks run on the ROS executor threads and only push jobs into a
 * queue.  The docking state machines are advanced in `update()`, which runs on the
 * ros2_control control thread and receives a recent *snapshot* of mjData.  The actual
 * mutation of `mjData::eq_active` and of the weld pool (which lives in mjModel) happens
 * in `pre_step()`, which runs on the physics thread immediately before `mj_step()`, so
 * the changes are guaranteed to be visible to exactly the next physics step.
 *
 * NOTE: `update()` receives a snapshot on which `eq_active` changes would be discarded
 * (only ctrl/qfrc_applied are copied back), which is why the activation lives in
 * `pre_step()`.
 *
 * Weld pool
 * ---------
 * The number of equality constraints in a compiled mjModel is fixed, so the plugin cannot
 * append constraints at runtime.  Instead the MJCF declares the welds and the `welds_pool`
 * parameter lists the ones this plugin owns:
 *
 *   * a weld in `welds_pool` is a pool slot: the plugin may re-point it to any interface
 *     pair and activate / deactivate it,
 *   * a weld *not* in `welds_pool` is never touched by the plugin,
 *   * a weld in `welds_pool` that is already active in the MJCF means the corresponding
 *     interface pair starts out docked; the plugin adopts it as a DOCKED pair so it can be
 *     released normally.
 */
class DockSimPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  using Dock = mujoco_dock_sim_plugin::action::Dock;
  using Release = mujoco_dock_sim_plugin::action::Release;
  using DockSrv = mujoco_dock_sim_plugin::srv::Dock;
  using ReleaseSrv = mujoco_dock_sim_plugin::srv::Release;
  using GoalHandleDock = rclcpp_action::ServerGoalHandle<Dock>;
  using GoalHandleRelease = rclcpp_action::ServerGoalHandle<Release>;

  DockSimPlugin() = default;
  ~DockSimPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void pre_step(mjData* data) override;
  void on_reset(mjData* data) override;
  void cleanup() override;

private:
  // ---------------------------------------------------------------- configuration
  /**
   * @brief Symmetry of the target orientation: which *other* orientations also count as
   *        "docked".
   *
   * Real interfaces are rarely unique in orientation:
   *   * a rectangular (0.1 x 0.04) connector fits in two phases (180 deg apart about its
   *     normal), so both may be accepted (`DISCRETE` with angles {0, 180}),
   *   * a square flange fits in four (`DISCRETE`, {0, 90, 180, 270}),
   *   * a circular connector fits at any phase (`CONTINUOUS`),
   *   * a keyed / asymmetric interface only fits one way (`NONE`).
   *
   * The plugin measures how far the current relative orientation is from the *set*
   * {R_axis(angle) o dock_target.quaternion} and welds the pair to the nearest member of
   * that set, so a symmetric connector is never forced into a nominal phase.
   */
  enum class SymmetryMode
  {
    NONE,       ///< only the target orientation itself is accepted
    DISCRETE,   ///< rotations from `angles_deg` about `axis` are also accepted
    CONTINUOUS  ///< any rotation about `axis` is accepted
  };

  struct Group
  {
    std::string name;
    std::set<int> sites;       ///< all sites of the group (male + female + neutral)
    std::set<int> male;        ///< optional
    std::set<int> female;      ///< optional
    bool gendered{ false };
    double position_tolerance{ 0.01 };
    double rotation_tolerance{ 0.1 };
    /// Target pose of site2 in site1's frame: position + quaternion (w, x, y, z).
    double dock_target[7]{ 0, 0, 0, 1, 0, 0, 0 };
    SymmetryMode symmetry{ SymmetryMode::NONE };
    int symmetry_axis{ 2 };               ///< 0 = x, 1 = y, 2 = z (in site1's frame)
    std::vector<double> symmetry_angles;  ///< radians; DISCRETE only
    double dock_time{ 1.0 };
    double release_time{ 0.0 };
  };

  /// One weld constraint of the pool.
  struct Slot
  {
    int eq_id{ -1 };              ///< index into mjModel.eq_*
    std::string weld_name;
    bool declared{ false };       ///< listed in `existing_welds`
    bool in_use{ false };         ///< currently assigned to a pair
    int pair_s1{ -1 };            ///< site ids of the assigned pair
    int pair_s2{ -1 };
  };

  enum class State
  {
    IDLE,
    WAIT_DOCKING,
    DOCKED,
    WAIT_RELEASING
  };

  struct JobResult
  {
    bool success{ false };
    std::string message;
    std::string state;
    double relpose[7]{ 0, 0, 0, 1, 0, 0, 0 };
  };

  /// A tracked interface pair.
  struct Pair
  {
    int site1{ -1 };
    int site2{ -1 };
    int group{ -1 };
    int slot{ -1 };
    State state{ State::IDLE };
    rclcpp::Time state_since{ 0, 0, RCL_ROS_TIME };
    bool has_deadline{ false };
    rclcpp::Time deadline{ 0, 0, RCL_ROS_TIME };

    // Completion plumbing (at most one of these is set).
    std::shared_ptr<std::promise<JobResult>> promise;
    std::shared_ptr<GoalHandleDock> dock_goal;
    std::shared_ptr<GoalHandleRelease> release_goal;
  };

  /// A request handed over from a ROS callback thread to update().
  struct Job
  {
    enum class Kind
    {
      DOCK,
      RELEASE
    } kind{ Kind::DOCK };
    int site1{ -1 };
    int site2{ -1 };
    int group{ -1 };
    double timeout{ 0.0 };
    std::shared_ptr<std::promise<JobResult>> promise;
    std::shared_ptr<GoalHandleDock> dock_goal;
    std::shared_ptr<GoalHandleRelease> release_goal;
  };

  // ------------------------------------------------------------------- parameters
  bool loadParameters();
  bool loadGroups();
  void buildPool();

  // -------------------------------------------------------------------- helpers
  int resolveSite(const std::string& name) const;
  int findGroup(int s1, int s2, const std::string& requested) const;
  bool groupAllows(const Group& g, int s1, int s2) const;
  /// Acquire a free pool slot.  Slots whose weld is currently *active* are skipped: they
  /// belong to an interface that is docked (e.g. an MJCF weld that was active at load
  /// time) and re-pointing them would silently break that weld.
  int acquireSlot(const mjData* data, int s1, int s2, int group);
  void releaseSlot(int slot);
  int findPair(int s1, int s2) const;

  /// Pool slot whose weld connects exactly these two bodies (-1 when none).
  int findSlotByBodies(int b1, int b2) const;

  /// Track a pair that is already welded in the simulation (e.g. a weld that was
  /// active in the MJCF at load time) as DOCKED.  Returns the pair index, or -1.
  int adoptDockedPair(const mjData* data, int site1, int site2, int group);

  /// Pool slot connecting the bodies of these two sites (-1 when none).
  int slotForSites(int site1, int site2) const;

  /// Relative pose of site2 in site1's frame from a (snapshot) mjData.
  static void relativePose(const mjData* data, int s1, int s2, double* pos, double* quat);

  /// Angle (rad) between two orientations; quaternions are sign-insensitive here.
  static double quatAngle(const double* a, const double* b);

  /// Distance of a measured pose to the group's docking target set, plus the nearest
  /// pose of that set (the pose the pair is welded to).
  void nearestTarget(const Group& g, const double* pos, const double* quat, double* pos_tgt,
                     double* quat_tgt, double& pos_err, double& rot_err) const;

  /// Write the target relative pose (site semantics) of a pair into the given slot.
  void writePairToSlot(const mjData* data, int slot, int s1, int s2, const double* site_relpose);

  static void quatMul(const double* a, const double* b, double* out);
  static void quatConj(const double* q, double* out);
  static void quatRotate(const double* q, const double* v, double* out);
  static void matToQuat(const double* m, double* q);
  static void quatFromAxisAngle(const double* axis_unit, double angle, double* q);
  /// Shortest-arc rotation taking (unit) `from` onto (unit) `to`.
  static void quatFromTwoVectors(const double* from, const double* to, double* q);
  /// Unit vector along axis `index` (0 = x, 1 = y, 2 = z).
  static void unitAxis(int index, double* out);

  // ------------------------------------------------------------------- callbacks
  void handleDockSrv(const DockSrv::Request::SharedPtr req, DockSrv::Response::SharedPtr res);
  void handleReleaseSrv(const ReleaseSrv::Request::SharedPtr req, ReleaseSrv::Response::SharedPtr res);
  rclcpp_action::GoalResponse handleDockGoal(const rclcpp_action::GoalUUID& uuid,
                                             std::shared_ptr<const Dock::Goal> goal);
  rclcpp_action::CancelResponse handleDockCancel(const std::shared_ptr<GoalHandleDock> goal);
  void handleDockAccepted(const std::shared_ptr<GoalHandleDock> goal);
  rclcpp_action::GoalResponse handleReleaseGoal(const rclcpp_action::GoalUUID& uuid,
                                                std::shared_ptr<const Release::Goal> goal);
  rclcpp_action::CancelResponse handleReleaseCancel(const std::shared_ptr<GoalHandleRelease> goal);
  void handleReleaseAccepted(const std::shared_ptr<GoalHandleRelease> goal);

  /// Shared entry point used by both the services (blocking) and the actions.
  JobResult runJob(Job job);

  /// Advance the state machines and publish action feedback. Called from update().
  void advance(const mjData* data);
  void publishFeedback(const Pair& pair, const double* pos, const double* quat, double pos_err,
                       double rot_err, const rclcpp::Time& now);
  void completePair(Pair& pair, const JobResult& result);

  // ----------------------------------------------------------------------- ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("DockSimPlugin") };
  rclcpp::Service<DockSrv>::SharedPtr dock_srv_;
  rclcpp::Service<ReleaseSrv>::SharedPtr release_srv_;
  rclcpp_action::Server<Dock>::SharedPtr dock_action_;
  rclcpp_action::Server<Release>::SharedPtr release_action_;

  // ------------------------------------------------------------------- model/data
  const mjModel* model_{ nullptr };
  mjModel* mutable_model_{ nullptr };  ///< the pool lives in mjModel; only touched in pre_step()

  // ------------------------------------------------------------------ weld pool
  std::vector<Slot> pool_;
  std::vector<std::string> welds_pool_;

  // --------------------------------------------------------------------- state
  std::vector<Group> groups_;
  std::vector<Pair> pairs_;

  /// Queue of jobs coming from ROS threads, drained in update().
  std::mutex jobs_mutex_;
  std::vector<Job> jobs_;

  /// Requests to mutate the live mjData / mjModel, applied in pre_step().
  struct PendingWeld
  {
    int slot{ -1 };
    bool active{ false };
    bool repoint{ false };
    int site1{ -1 };
    int site2{ -1 };
    double relpose[7]{ 0, 0, 0, 1, 0, 0, 0 };
  };
  std::mutex pending_mutex_;
  std::vector<PendingWeld> pending_;

  std::atomic_bool dirty_{ false };  ///< set by update(), consumed by pre_step()
};

}  // namespace mujoco_dock_sim_plugin

#endif  // MUJOCO_DOCK_SIM_PLUGIN__DOCK_SIM_PLUGIN_HPP_
