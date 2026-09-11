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

#include "mujoco_dock_sim_plugin/dock_sim_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

namespace mujoco_dock_sim_plugin
{

namespace
{
constexpr double kQuatNormEps = 1e-9;
constexpr double kDefaultTimeout = 30.0;
}  // namespace

// ============================================================================
// quaternion / pose helpers  (quaternions are (w, x, y, z))
// ============================================================================

void DockSimPlugin::quatMul(const double* a, const double* b, double* out)
{
  const double w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  const double x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  const double y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  const double z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
  out[0] = w;
  out[1] = x;
  out[2] = y;
  out[3] = z;
}

void DockSimPlugin::quatConj(const double* q, double* out)
{
  out[0] = q[0];
  out[1] = -q[1];
  out[2] = -q[2];
  out[3] = -q[3];
}

void DockSimPlugin::quatRotate(const double* q, const double* v, double* out)
{
  // out = q * v * q^-1  =  v + 2w(qv x v) + 2 qv x (qv x v)
  const double w = q[0];
  const double tx = 2.0 * (q[2] * v[2] - q[3] * v[1]);
  const double ty = 2.0 * (q[3] * v[0] - q[1] * v[2]);
  const double tz = 2.0 * (q[1] * v[1] - q[2] * v[0]);
  out[0] = v[0] + w * tx + (q[2] * tz - q[3] * ty);
  out[1] = v[1] + w * ty + (q[3] * tx - q[1] * tz);
  out[2] = v[2] + w * tz + (q[1] * ty - q[2] * tx);
}

void DockSimPlugin::matToQuat(const double* m, double* q)
{
  // m is a row-major 3x3 rotation matrix
  const double tr = m[0] + m[4] + m[8];
  if (tr > 0.0)
  {
    const double s = std::sqrt(tr + 1.0) * 2.0;
    q[0] = 0.25 * s;
    q[1] = (m[7] - m[5]) / s;
    q[2] = (m[2] - m[6]) / s;
    q[3] = (m[3] - m[1]) / s;
  }
  else if (m[0] > m[4] && m[0] > m[8])
  {
    const double s = std::sqrt(1.0 + m[0] - m[4] - m[8]) * 2.0;
    q[0] = (m[7] - m[5]) / s;
    q[1] = 0.25 * s;
    q[2] = (m[1] + m[3]) / s;
    q[3] = (m[2] + m[6]) / s;
  }
  else if (m[4] > m[8])
  {
    const double s = std::sqrt(1.0 + m[4] - m[0] - m[8]) * 2.0;
    q[0] = (m[2] - m[6]) / s;
    q[1] = (m[1] + m[3]) / s;
    q[2] = 0.25 * s;
    q[3] = (m[5] + m[7]) / s;
  }
  else
  {
    const double s = std::sqrt(1.0 + m[8] - m[0] - m[4]) * 2.0;
    q[0] = (m[3] - m[1]) / s;
    q[1] = (m[2] + m[6]) / s;
    q[2] = (m[5] + m[7]) / s;
    q[3] = 0.25 * s;
  }
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n > kQuatNormEps)
  {
    for (int i = 0; i < 4; ++i)
    {
      q[i] /= n;
    }
  }
  else
  {
    q[0] = 1.0;
    q[1] = q[2] = q[3] = 0.0;
  }
}

void DockSimPlugin::relativePose(const mjData* data, int s1, int s2, double* pos, double* quat)
{
  // x_rel = inv(x_site1) o x_site2
  const double* p1 = data->site_xpos + 3 * s1;
  const double* p2 = data->site_xpos + 3 * s2;
  double q1[4], q2[4];
  matToQuat(data->site_xmat + 9 * s1, q1);
  matToQuat(data->site_xmat + 9 * s2, q2);

  double q1c[4];
  quatConj(q1, q1c);
  const double d[3] = { p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2] };
  quatRotate(q1c, d, pos);
  quatMul(q1c, q2, quat);
  const double n = std::sqrt(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
  if (n > kQuatNormEps)
  {
    for (int i = 0; i < 4; ++i)
    {
      quat[i] /= n;
    }
  }
}

void DockSimPlugin::poseError(const double* pos, const double* quat, const double* target, double& pos_err,
                              double& rot_err)
{
  const double dx = pos[0] - target[0];
  const double dy = pos[1] - target[1];
  const double dz = pos[2] - target[2];
  pos_err = std::sqrt(dx * dx + dy * dy + dz * dz);

  double dot = quat[0] * target[3] + quat[1] * target[4] + quat[2] * target[5] + quat[3] * target[6];
  dot = std::min(1.0, std::max(-1.0, std::fabs(dot)));
  rot_err = 2.0 * std::acos(dot);
}

// ============================================================================
// init
// ============================================================================

bool DockSimPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  node_ = node;
  logger_ = node_->get_logger().get_child(node->get_sub_namespace());
  model_ = model;
  // The weld pool lives in mjModel (eq_obj1id / eq_obj2id / eq_objtype / eq_data).
  // It is only ever mutated from pre_step(), i.e. on the physics thread immediately
  // before mj_step(), which is the only place where that is safe.
  mutable_model_ = const_cast<mjModel*>(model);

  if (!loadParameters() || !loadGroups())
  {
    return false;
  }
  buildPool();

  dock_srv_ = node_->create_service<DockSrv>(
      "dock", std::bind(&DockSimPlugin::handleDockSrv, this, std::placeholders::_1, std::placeholders::_2));
  release_srv_ = node_->create_service<ReleaseSrv>(
      "release",
      std::bind(&DockSimPlugin::handleReleaseSrv, this, std::placeholders::_1, std::placeholders::_2));

  dock_action_ = rclcpp_action::create_server<Dock>(
      node_, "dock", std::bind(&DockSimPlugin::handleDockGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DockSimPlugin::handleDockCancel, this, std::placeholders::_1),
      std::bind(&DockSimPlugin::handleDockAccepted, this, std::placeholders::_1));
  release_action_ = rclcpp_action::create_server<Release>(
      node_, "release",
      std::bind(&DockSimPlugin::handleReleaseGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DockSimPlugin::handleReleaseCancel, this, std::placeholders::_1),
      std::bind(&DockSimPlugin::handleReleaseAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(logger_,
              "DockSimPlugin initialised: %zu group(s), weld pool of %zu constraint(s). Services '%s', '%s'; actions "
              "'dock', 'release'.",
              groups_.size(), pool_.size(), dock_srv_->get_service_name(), release_srv_->get_service_name());
  for (const auto& g : groups_)
  {
    RCLCPP_INFO(logger_, "  group '%s': %zu site(s)%s pos_tol=%.4f rot_tol=%.4f dock_time=%.2fs release_time=%.2fs",
                g.name.c_str(), g.sites.size(), g.gendered ? " (gendered)" : "", g.position_tolerance,
                g.rotation_tolerance, g.dock_time, g.release_time);
  }
  return true;
}

bool DockSimPlugin::loadParameters()
{
  const std::string prefix = "mujoco_plugins." + node_->get_sub_namespace() + ".";
  auto declare = [&](const std::string& n, auto def) {
    const std::string full = prefix + n;
    if (!node_->has_parameter(full))
    {
      node_->declare_parameter(full, def);
    }
    return full;
  };

  // The weld pool is exactly the list given in `welds_pool`.  MuJoCo fixes the number of
  // equality constraints at compile time, so the plugin cannot create welds on the fly;
  // instead the MJCF declares a set of welds and this list selects the ones the plugin
  // owns.  Welds that are not listed are never touched by this plugin.
  welds_pool_ = node_->get_parameter(declare("welds_pool", std::vector<std::string>{})).as_string_array();
  if (welds_pool_.empty())
  {
    RCLCPP_WARN(logger_,
                "No 'welds_pool' configured; the plugin has no weld constraints to drive and every dock request "
                "will be rejected.");
  }
  return true;
}

int DockSimPlugin::resolveSite(const std::string& name) const
{
  return mj_name2id(model_, mjOBJ_SITE, name.c_str());
}

bool DockSimPlugin::loadGroups()
{
  const std::string root = "mujoco_plugins." + node_->get_sub_namespace() + ".groups";
  const auto list = node_->list_parameters({ root }, 0u);

  const size_t init = root.size() + 1;
  std::vector<std::string> names;
  for (const auto& p : list.names)
  {
    if (p.size() <= init)
    {
      continue;
    }
    const auto dot = p.find_first_of('.', init);
    const std::string name = p.substr(init, dot - init);
    if (!name.empty() && std::find(names.begin(), names.end(), name) == names.end())
    {
      names.push_back(name);
    }
  }

  if (names.empty())
  {
    RCLCPP_WARN(logger_, "No interface groups configured under '%s'; docking requests will be rejected.", root.c_str());
    return true;
  }

  for (const auto& name : names)
  {
    const std::string p = root + "." + name + ".";
    auto declare = [&](const std::string& n, auto def) {
      const std::string full = p + n;
      if (!node_->has_parameter(full))
      {
        node_->declare_parameter(full, def);
      }
      return full;
    };

    Group g;
    g.name = name;
    const auto site_names = node_->get_parameter(declare("sites", std::vector<std::string>{})).as_string_array();
    const auto male_names = node_->get_parameter(declare("male_sites", std::vector<std::string>{})).as_string_array();
    const auto female_names =
        node_->get_parameter(declare("female_sites", std::vector<std::string>{})).as_string_array();
    g.position_tolerance = node_->get_parameter(declare("position_tolerance", 0.01)).as_double();
    g.rotation_tolerance = node_->get_parameter(declare("rotation_tolerance", 0.1)).as_double();
    g.dock_time = node_->get_parameter(declare("dock_time", 1.0)).as_double();
    g.release_time = node_->get_parameter(declare("release_time", 0.0)).as_double();

    const auto relpose = node_->get_parameter(declare("dock_relpose", std::vector<double>{})).as_double_array();
    if (!relpose.empty())
    {
      if (relpose.size() != 7)
      {
        RCLCPP_ERROR(logger_, "group '%s': dock_relpose must have exactly 7 values (pos + quat), got %zu.", name.c_str(),
                     relpose.size());
        return false;
      }
      std::copy(relpose.begin(), relpose.end(), g.dock_relpose);
      double n = 0.0;
      for (int i = 3; i < 7; ++i)
      {
        n += g.dock_relpose[i] * g.dock_relpose[i];
      }
      if (n < kQuatNormEps)
      {
        g.dock_relpose[3] = 1.0;
        g.dock_relpose[4] = g.dock_relpose[5] = g.dock_relpose[6] = 0.0;
      }
      else
      {
        n = std::sqrt(n);
        for (int i = 3; i < 7; ++i)
        {
          g.dock_relpose[i] /= n;
        }
      }
    }

    auto add = [&](const std::vector<std::string>& in, std::set<int>& out, const char* what) {
      for (const auto& s : in)
      {
        const int id = resolveSite(s);
        if (id < 0)
        {
          RCLCPP_ERROR(logger_, "group '%s': unknown %s site '%s'.", name.c_str(), what, s.c_str());
          return false;
        }
        out.insert(id);
      }
      return true;
    };

    if (!add(site_names, g.sites, "sites") || !add(male_names, g.male, "male_sites") ||
        !add(female_names, g.female, "female_sites"))
    {
      return false;
    }
    g.sites.insert(g.male.begin(), g.male.end());
    g.sites.insert(g.female.begin(), g.female.end());
    g.gendered = !g.male.empty() || !g.female.empty();
    if (g.gendered && (g.male.empty() || g.female.empty()))
    {
      RCLCPP_ERROR(logger_, "group '%s': a gendered group needs both male_sites and female_sites.", name.c_str());
      return false;
    }
    if (g.sites.size() < 2)
    {
      RCLCPP_WARN(logger_, "group '%s' has fewer than 2 sites.", name.c_str());
    }
    groups_.push_back(std::move(g));
  }
  return true;
}

void DockSimPlugin::buildPool()
{
  pool_.clear();

  // The pool is exactly `welds_pool`: those welds are driven by the plugin, everything else
  // in the model is left alone.  A weld that is already active in the MJCF means the
  // corresponding interface pair starts out docked (handled in update()).
  for (const auto& name : welds_pool_)
  {
    const int id = mj_name2id(model_, mjOBJ_EQUALITY, name.c_str());
    if (id < 0)
    {
      RCLCPP_WARN(logger_, "welds_pool entry '%s' is not an equality constraint in the model; skipping.", name.c_str());
      continue;
    }
    if (model_->eq_type[id] != mjEQ_WELD)
    {
      RCLCPP_WARN(logger_, "welds_pool entry '%s' is not a weld constraint; skipping.", name.c_str());
      continue;
    }
    Slot s;
    s.eq_id = id;
    s.weld_name = name;
    s.declared = true;
    pool_.push_back(std::move(s));
  }

  if (pool_.empty())
  {
    RCLCPP_WARN(logger_, "The weld pool is empty; docking requests cannot be served.");
  }
}

// ============================================================================
// pair / group helpers
// ============================================================================

int DockSimPlugin::findPair(int s1, int s2) const
{
  for (size_t i = 0; i < pairs_.size(); ++i)
  {
    if (pairs_[i].group < 0)
    {
      continue;
    }
    if ((pairs_[i].site1 == s1 && pairs_[i].site2 == s2) || (pairs_[i].site1 == s2 && pairs_[i].site2 == s1))
    {
      return static_cast<int>(i);
    }
  }
  return -1;
}

bool DockSimPlugin::groupAllows(const Group& g, int s1, int s2) const
{
  if (s1 == s2 || g.sites.count(s1) == 0 || g.sites.count(s2) == 0)
  {
    return false;
  }
  if (!g.gendered)
  {
    return true;
  }
  return (g.male.count(s1) && g.female.count(s2)) || (g.female.count(s1) && g.male.count(s2));
}

int DockSimPlugin::findGroup(int s1, int s2, const std::string& requested) const
{
  for (size_t i = 0; i < groups_.size(); ++i)
  {
    if (!requested.empty() && groups_[i].name != requested)
    {
      continue;
    }
    if (groupAllows(groups_[i], s1, s2))
    {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int DockSimPlugin::acquireSlot(int s1, int s2, int /*group*/)
{
  for (size_t i = 0; i < pool_.size(); ++i)
  {
    if (pool_[i].in_use && pool_[i].pair_s1 == s1 && pool_[i].pair_s2 == s2)
    {
      return static_cast<int>(i);
    }
  }
  for (size_t i = 0; i < pool_.size(); ++i)
  {
    if (!pool_[i].in_use)
    {
      pool_[i].in_use = true;
      pool_[i].pair_s1 = s1;
      pool_[i].pair_s2 = s2;
      return static_cast<int>(i);
    }
  }
  return -1;
}

void DockSimPlugin::releaseSlot(int slot)
{
  if (slot >= 0 && slot < static_cast<int>(pool_.size()))
  {
    pool_[slot].in_use = false;
    pool_[slot].pair_s1 = -1;
    pool_[slot].pair_s2 = -1;
  }
}

int DockSimPlugin::findSlotByBodies(int b1, int b2) const
{
  for (size_t i = 0; i < pool_.size(); ++i)
  {
    const int eq = pool_[i].eq_id;
    if (eq < 0 || eq >= model_->neq || model_->eq_objtype[eq] != mjOBJ_BODY)
    {
      continue;
    }
    const int o1 = model_->eq_obj1id[eq];
    const int o2 = model_->eq_obj2id[eq];
    if ((o1 == b1 && o2 == b2) || (o1 == b2 && o2 == b1))
    {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int DockSimPlugin::adoptDockedPair(const mjData* data, int site1, int site2, int group)
{
  // The MJCF may already contain active welds describing pairs that start out docked
  // (as the space_sim lens/dock welds do).  Those are not in `pairs_` yet, so a release
  // request would otherwise be answered with "not docked" while the weld stays on.
  const int b1 = model_->site_bodyid[site1];
  const int b2 = model_->site_bodyid[site2];
  const int slot = findSlotByBodies(b1, b2);
  if (slot < 0)
  {
    RCLCPP_DEBUG(logger_, "adopt: no pool slot connects bodies %d and %d.", b1, b2);
    return -1;
  }
  const int eq = pool_[slot].eq_id;
  const bool active = (eq >= 0 && eq < model_->neq) && data->eq_active[eq];
  RCLCPP_DEBUG(logger_, "adopt: slot %d (weld '%s') connects bodies %d/%d, active=%d.", slot,
               pool_[slot].weld_name.c_str(), b1, b2, static_cast<int>(active));
  if (!active)
  {
    return -1;
  }

  Pair p;
  p.site1 = site1;
  p.site2 = site2;
  p.group = group;
  p.slot = slot;
  p.state = State::DOCKED;
  p.state_since = node_->get_clock()->now();
  pairs_.push_back(p);

  pool_[slot].in_use = true;
  pool_[slot].pair_s1 = site1;
  pool_[slot].pair_s2 = site2;
  RCLCPP_INFO(logger_, "Adopted pre-existing active weld '%s' as a docked pair.", pool_[slot].weld_name.c_str());
  return static_cast<int>(pairs_.size()) - 1;
}

int DockSimPlugin::slotForSites(int site1, int site2) const
{
  return findSlotByBodies(model_->site_bodyid[site1], model_->site_bodyid[site2]);
}

void DockSimPlugin::writePairToSlot(const mjData* /*data*/, int slot, int s1, int s2, const double* site_relpose)
{
  if (slot < 0 || slot >= static_cast<int>(pool_.size()))
  {
    return;
  }
  const int eq = pool_[slot].eq_id;
  if (eq < 0 || eq >= model_->neq)
  {
    return;
  }
  const int b1 = model_->site_bodyid[s1];
  const int b2 = model_->site_bodyid[s2];

  const double* sp1 = model_->site_pos + 3 * s1;
  const double* sq1 = model_->site_quat + 4 * s1;
  const double* sp2 = model_->site_pos + 3 * s2;
  const double* sq2 = model_->site_quat + 4 * s2;

  // We want the weld to drive x_body2 = x_body1 o relpose_body with
  //   relpose_body = S1 o T_site o inv(S2)
  // where S1/S2 are the site frames inside their bodies and T_site is the desired
  // relative pose of site2 expressed in site1's frame.
  //
  // Derivation: x_site1 = x_body1 o S1 and x_site2 = x_body2 o S2, and the docking
  // target is x_site2 = x_site1 o T_site, hence
  //   x_body2 = x_site2 o inv(S2) = x_body1 o S1 o T_site o inv(S2).
  double sq2c[4];
  quatConj(sq2, sq2c);

  double tmp[4];
  quatMul(site_relpose + 3, sq2c, tmp);  // T.rot * inv(S2.rot)
  double rel_q[4];
  quatMul(sq1, tmp, rel_q);              // S1.rot * ...

  // position: p_S1 + R(S1) * ( T.pos - R(T.rot) * R(S2.rot)^-1 * p_S2 )
  double s2inv[3];
  quatRotate(sq2c, sp2, s2inv);
  double s2rot[3];
  quatRotate(site_relpose + 3, s2inv, s2rot);
  const double inner[3] = { site_relpose[0] - s2rot[0], site_relpose[1] - s2rot[1],
                            site_relpose[2] - s2rot[2] };
  double rel_p[3];
  quatRotate(sq1, inner, rel_p);
  for (int i = 0; i < 3; ++i)
  {
    rel_p[i] += sp1[i];
  }

  // Only pre_step() (physics thread) may reach this point.
  mjModel* m = mutable_model_;
  m->eq_objtype[eq] = mjOBJ_BODY;
  m->eq_obj1id[eq] = b1;
  m->eq_obj2id[eq] = b2;
  // mjModel::eq_data layout for a weld (mjNEQDATA == 11):
  //   [0:3] anchor in body2 frame, [3:6] relpose position, [6] relpose qw,
  //   [7:10] relpose qx,qy,qz, [10] torquescale
  double* d = m->eq_data + eq * mjNEQDATA;
  d[0] = d[1] = d[2] = 0.0;
  d[3] = rel_p[0];
  d[4] = rel_p[1];
  d[5] = rel_p[2];
  d[6] = rel_q[0];
  d[7] = rel_q[1];
  d[8] = rel_q[2];
  d[9] = rel_q[3];
  d[10] = 1.0;
}

// ============================================================================
// job plumbing
// ============================================================================

DockSimPlugin::JobResult DockSimPlugin::runJob(Job job)
{
  auto promise = std::make_shared<std::promise<JobResult>>();
  job.promise = promise;
  auto future = promise->get_future();

  const double timeout = job.timeout > 0.0 ? job.timeout : kDefaultTimeout;
  {
    std::lock_guard<std::mutex> lock(jobs_mutex_);
    jobs_.push_back(std::move(job));
  }

  if (future.wait_for(std::chrono::duration<double>(timeout)) == std::future_status::timeout)
  {
    JobResult r;
    r.success = false;
    r.state = "timeout";
    r.message = "Timed out after " + std::to_string(timeout) + " s.";
    return r;
  }
  return future.get();
}

// ============================================================================
// ROS callbacks
// ============================================================================

void DockSimPlugin::handleDockSrv(const DockSrv::Request::SharedPtr req, DockSrv::Response::SharedPtr res)
{
  Job job;
  job.kind = Job::Kind::DOCK;
  job.site1 = resolveSite(req->site1);
  job.site2 = resolveSite(req->site2);
  job.timeout = req->timeout;

  if (job.site1 < 0 || job.site2 < 0)
  {
    res->success = false;
    res->state = "rejected";
    res->message = "Unknown site: '" + (job.site1 < 0 ? req->site1 : req->site2) + "'.";
    return;
  }
  job.group = findGroup(job.site1, job.site2, req->group);
  if (job.group < 0)
  {
    res->success = false;
    res->state = "rejected";
    res->message =
        "Sites '" + req->site1 + "' and '" + req->site2 + "' are not a valid pair for a configured group.";
    return;
  }

  const JobResult r = runJob(std::move(job));
  res->success = r.success;
  res->state = r.state;
  res->message = r.message;
  res->relpose.position.x = r.relpose[0];
  res->relpose.position.y = r.relpose[1];
  res->relpose.position.z = r.relpose[2];
  res->relpose.orientation.w = r.relpose[3];
  res->relpose.orientation.x = r.relpose[4];
  res->relpose.orientation.y = r.relpose[5];
  res->relpose.orientation.z = r.relpose[6];
}

void DockSimPlugin::handleReleaseSrv(const ReleaseSrv::Request::SharedPtr req, ReleaseSrv::Response::SharedPtr res)
{
  Job job;
  job.kind = Job::Kind::RELEASE;
  job.site1 = resolveSite(req->site1);
  job.site2 = resolveSite(req->site2);
  job.timeout = req->timeout;
  if (job.site1 < 0 || job.site2 < 0)
  {
    res->success = false;
    res->state = "rejected";
    res->message = "Unknown site: '" + (job.site1 < 0 ? req->site1 : req->site2) + "'.";
    return;
  }
  const JobResult r = runJob(std::move(job));
  res->success = r.success;
  res->state = r.state;
  res->message = r.message;
}

rclcpp_action::GoalResponse DockSimPlugin::handleDockGoal(const rclcpp_action::GoalUUID&,
                                                          std::shared_ptr<const Dock::Goal> goal)
{
  if (resolveSite(goal->site1) < 0 || resolveSite(goal->site2) < 0)
  {
    RCLCPP_WARN(logger_, "Rejecting dock goal: unknown site.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DockSimPlugin::handleDockCancel(const std::shared_ptr<GoalHandleDock>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DockSimPlugin::handleDockAccepted(const std::shared_ptr<GoalHandleDock> goal)
{
  Job job;
  job.kind = Job::Kind::DOCK;
  job.site1 = resolveSite(goal->get_goal()->site1);
  job.site2 = resolveSite(goal->get_goal()->site2);
  job.timeout = goal->get_goal()->timeout;
  job.group = findGroup(job.site1, job.site2, goal->get_goal()->group);
  job.dock_goal = goal;
  std::lock_guard<std::mutex> lock(jobs_mutex_);
  jobs_.push_back(std::move(job));
}

rclcpp_action::GoalResponse DockSimPlugin::handleReleaseGoal(const rclcpp_action::GoalUUID&,
                                                             std::shared_ptr<const Release::Goal> goal)
{
  if (resolveSite(goal->site1) < 0 || resolveSite(goal->site2) < 0)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DockSimPlugin::handleReleaseCancel(const std::shared_ptr<GoalHandleRelease>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DockSimPlugin::handleReleaseAccepted(const std::shared_ptr<GoalHandleRelease> goal)
{
  Job job;
  job.kind = Job::Kind::RELEASE;
  job.site1 = resolveSite(goal->get_goal()->site1);
  job.site2 = resolveSite(goal->get_goal()->site2);
  job.timeout = goal->get_goal()->timeout;
  job.release_goal = goal;
  std::lock_guard<std::mutex> lock(jobs_mutex_);
  jobs_.push_back(std::move(job));
}

void DockSimPlugin::completePair(Pair& pair, const JobResult& result)
{
  if (pair.promise)
  {
    pair.promise->set_value(result);
    pair.promise.reset();
  }
  if (pair.dock_goal)
  {
    auto res = std::make_shared<Dock::Result>();
    res->success = result.success;
    res->message = result.message;
    res->relpose.position.x = result.relpose[0];
    res->relpose.position.y = result.relpose[1];
    res->relpose.position.z = result.relpose[2];
    res->relpose.orientation.w = result.relpose[3];
    res->relpose.orientation.x = result.relpose[4];
    res->relpose.orientation.y = result.relpose[5];
    res->relpose.orientation.z = result.relpose[6];
    if (result.success)
    {
      pair.dock_goal->succeed(res);
    }
    else
    {
      pair.dock_goal->abort(res);
    }
    pair.dock_goal.reset();
  }
  if (pair.release_goal)
  {
    auto res = std::make_shared<Release::Result>();
    res->success = result.success;
    res->message = result.message;
    if (result.success)
    {
      pair.release_goal->succeed(res);
    }
    else
    {
      pair.release_goal->abort(res);
    }
    pair.release_goal.reset();
  }
}

void DockSimPlugin::publishFeedback(const Pair& pair, const double* /*pos*/, const double* /*quat*/, double pos_err,
                                    double rot_err, const rclcpp::Time& now)
{
  const bool in_tol = pair.group >= 0 && pos_err <= groups_[pair.group].position_tolerance &&
                      rot_err <= groups_[pair.group].rotation_tolerance;

  if (pair.dock_goal && pair.dock_goal->is_active())
  {
    auto fb = std::make_shared<Dock::Feedback>();
    fb->state = (pair.state == State::DOCKED) ? "docked" : (in_tol ? "within_tolerance" : "waiting");
    fb->elapsed = (now - pair.state_since).seconds();
    fb->position_error = pos_err;
    fb->rotation_error = rot_err;
    pair.dock_goal->publish_feedback(fb);
  }
  if (pair.release_goal && pair.release_goal->is_active())
  {
    auto fb = std::make_shared<Release::Feedback>();
    fb->state = (pair.state == State::WAIT_RELEASING) ? "releasing" : "waiting";
    fb->elapsed = (now - pair.state_since).seconds();
    pair.release_goal->publish_feedback(fb);
  }
}

// ============================================================================
// update(): advance the state machines (control thread, data is a snapshot)
// ============================================================================

void DockSimPlugin::update(const mjModel* /*model*/, mjData* data)
{
  const rclcpp::Time now = node_->get_clock()->now();

  // ---- 1. drain the job queue ------------------------------------------------
  std::vector<Job> jobs;
  {
    std::lock_guard<std::mutex> lock(jobs_mutex_);
    jobs.swap(jobs_);
  }

  for (auto& job : jobs)
  {
    int existing = findPair(job.site1, job.site2);
    int group = job.group >= 0 ? job.group : findGroup(job.site1, job.site2, "");

    // A pair may already be welded by the initial MJCF (e.g. the lenses start docked on
    // their sockets).  Adopt those so that release/dock act on the real weld.
    if (existing < 0)
    {
      const int g = group >= 0 ? group : findGroup(job.site1, job.site2, "");
      const int adopted = adoptDockedPair(data, job.site1, job.site2, g);
      if (adopted >= 0)
      {
        existing = adopted;
        if (group < 0)
        {
          group = g;
        }
      }
    }

    if (job.kind == Job::Kind::RELEASE)
    {
      if (existing < 0 || pairs_[existing].state != State::DOCKED)
      {
        // Fallback: the pair may be welded by the MJCF without being tracked (e.g. the
        // weld was active at load time but the snapshot's eq_active was not what we
        // expected).  If a pool slot connects the two bodies, release it directly.
        const int slot = slotForSites(job.site1, job.site2);
        if (slot >= 0 && data->eq_active[pool_[slot].eq_id])
        {
          PendingWeld pw;
          pw.slot = slot;
          pw.active = false;
          {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending_.push_back(pw);
          }
          dirty_.store(true);
          releaseSlot(slot);
          auto res = std::make_shared<Release::Result>();
          res->success = true;
          res->message = "Released (untracked weld).";
          if (job.promise)
          {
            JobResult r;
            r.success = true;
            r.state = "released";
            r.message = res->message;
            job.promise->set_value(r);
          }
          if (job.release_goal)
          {
            job.release_goal->succeed(res);
          }
          continue;
        }

        auto res = std::make_shared<Release::Result>();
        res->success = true;
        res->message = "Pair is not docked; nothing to release.";
        if (job.promise)
        {
          JobResult r;
          r.success = true;
          r.state = "not_docked";
          r.message = res->message;
          job.promise->set_value(r);
        }
        if (job.release_goal)
        {
          job.release_goal->succeed(res);
        }
        continue;
      }
      Pair& p = pairs_[existing];
      p.state = State::WAIT_RELEASING;
      p.state_since = now;
      const double to = job.timeout > 0.0 ? job.timeout : groups_[p.group].release_time * 3.0 + 5.0;
      p.deadline = now + rclcpp::Duration::from_seconds(to);
      p.has_deadline = true;
      p.promise = job.promise;
      p.release_goal = job.release_goal;
      continue;
    }

    // ---- DOCK ---------------------------------------------------------------
    std::string reject;
    if (group < 0)
    {
      reject = "Sites are not a valid pair for any configured group.";
    }
    else if (existing >= 0 && pairs_[existing].state == State::DOCKED)
    {
      auto res = std::make_shared<Dock::Result>();
      res->success = true;
      res->message = "Pair is already docked.";
      if (job.promise)
      {
        JobResult r;
        r.success = true;
        r.state = "already_docked";
        r.message = res->message;
        job.promise->set_value(r);
      }
      if (job.dock_goal)
      {
        job.dock_goal->succeed(res);
      }
      continue;
    }
    else
    {
      const int slot = (existing >= 0) ? pairs_[existing].slot : acquireSlot(job.site1, job.site2, group);
      if (slot < 0)
      {
        reject = "No free slot in the weld pool (pool size " + std::to_string(pool_.size()) + ").";
      }
      else
      {
        int idx = existing;
        if (idx < 0)
        {
          Pair np;
          np.site1 = job.site1;
          np.site2 = job.site2;
          pairs_.push_back(np);
          idx = static_cast<int>(pairs_.size()) - 1;
        }
        Pair& p = pairs_[idx];
        p.group = group;
        p.slot = slot;
        p.state = State::WAIT_DOCKING;
        p.state_since = now;
        const double to = job.timeout > 0.0 ? job.timeout : groups_[group].dock_time * 3.0 + 5.0;
        p.deadline = now + rclcpp::Duration::from_seconds(to);
        p.has_deadline = true;
        p.promise = job.promise;
        p.dock_goal = job.dock_goal;
        continue;
      }
    }

    if (job.promise)
    {
      JobResult r;
      r.success = false;
      r.state = "rejected";
      r.message = reject;
      job.promise->set_value(r);
    }
    if (job.dock_goal)
    {
      auto res = std::make_shared<Dock::Result>();
      res->success = false;
      res->message = reject;
      job.dock_goal->abort(res);
    }
  }

  // ---- 2. advance the active pairs -------------------------------------------
  std::vector<PendingWeld> pending;
  std::vector<std::pair<int, JobResult>> completions;

  for (size_t i = 0; i < pairs_.size(); ++i)
  {
    Pair& p = pairs_[i];
    if (p.group < 0 || p.state == State::IDLE)
    {
      continue;
    }
    const Group& g = groups_[p.group];

    double pos[3], quat[4];
    relativePose(data, p.site1, p.site2, pos, quat);
    double pos_err, rot_err;
    poseError(pos, quat, g.dock_relpose, pos_err, rot_err);
    const bool in_tol = pos_err <= g.position_tolerance && rot_err <= g.rotation_tolerance;

    if (p.has_deadline && now > p.deadline)
    {
      PendingWeld pw;
      pw.slot = p.slot;
      pw.active = false;
      pending.push_back(pw);
      releaseSlot(p.slot);

      JobResult r;
      r.success = false;
      r.state = "timeout";
      r.message = "Timed out waiting for the docking/release sequence to complete.";
      completions.emplace_back(static_cast<int>(i), r);

      p.state = State::IDLE;
      p.group = -1;
      p.slot = -1;
      p.has_deadline = false;
      continue;
    }

    if (p.state == State::WAIT_DOCKING)
    {
      if (!in_tol)
      {
        // left the tolerance window before dock_time elapsed -> restart the timer
        p.state_since = now;
      }
      publishFeedback(p, pos, quat, pos_err, rot_err, now);
      if (in_tol && (now - p.state_since).seconds() >= g.dock_time)
      {
        PendingWeld pw;
        pw.slot = p.slot;
        pw.active = true;
        pw.repoint = true;
        pw.site1 = p.site1;
        pw.site2 = p.site2;
        std::copy(g.dock_relpose, g.dock_relpose + 7, pw.relpose);
        pending.push_back(pw);

        JobResult r;
        r.success = true;
        r.state = "docked";
        r.message = "Docking successful.";
        relativePose(data, p.site1, p.site2, pos, quat);
        r.relpose[0] = pos[0];
        r.relpose[1] = pos[1];
        r.relpose[2] = pos[2];
        r.relpose[3] = quat[0];
        r.relpose[4] = quat[1];
        r.relpose[5] = quat[2];
        r.relpose[6] = quat[3];
        completions.emplace_back(static_cast<int>(i), r);

        p.state = State::DOCKED;
        p.state_since = now;
        p.has_deadline = false;
      }
    }
    else if (p.state == State::WAIT_RELEASING)
    {
      publishFeedback(p, pos, quat, pos_err, rot_err, now);
      if ((now - p.state_since).seconds() >= g.release_time)
      {
        PendingWeld pw;
        pw.slot = p.slot;
        pw.active = false;
        pending.push_back(pw);

        JobResult r;
        r.success = true;
        r.state = "released";
        r.message = "Released.";
        completions.emplace_back(static_cast<int>(i), r);

        releaseSlot(p.slot);
        p.state = State::IDLE;
        p.slot = -1;
        p.group = -1;
        p.has_deadline = false;
      }
    }
  }

  // ---- 3. hand the mjModel / mjData mutations to pre_step() ------------------
  if (!pending.empty())
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    for (const auto& pw : pending)
    {
      pending_.push_back(pw);
    }
    dirty_.store(true);
  }

  for (auto& c : completions)
  {
    completePair(pairs_[c.first], c.second);
  }
}

// ============================================================================
// pre_step(): apply the queued mutations to the *live* mjData / mjModel
// ============================================================================

void DockSimPlugin::pre_step(mjData* data)
{
  if (!dirty_.load())
  {
    return;
  }
  dirty_.store(false);

  std::vector<PendingWeld> pending;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending.swap(pending_);
  }

  for (const auto& pw : pending)
  {
    if (pw.slot < 0 || pw.slot >= static_cast<int>(pool_.size()))
    {
      continue;
    }
    const int eq = pool_[pw.slot].eq_id;
    if (eq < 0 || eq >= model_->neq)
    {
      continue;
    }
    if (pw.repoint)
    {
      writePairToSlot(data, pw.slot, pw.site1, pw.site2, pw.relpose);
    }
    data->eq_active[eq] = pw.active ? 1 : 0;
  }
}

void DockSimPlugin::on_reset(mjData* /*data*/)
{
  // A world reset restores eq_active from the MJCF defaults and invalidates every
  // latched state, so drop everything and let callers dock again from scratch.
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_.clear();
    dirty_.store(false);
  }
  for (auto& p : pairs_)
  {
    p.state = State::IDLE;
    p.group = -1;
    p.slot = -1;
    p.has_deadline = false;
    p.promise.reset();
    p.dock_goal.reset();
    p.release_goal.reset();
  }
  pairs_.clear();
  for (auto& s : pool_)
  {
    s.in_use = false;
    s.pair_s1 = -1;
    s.pair_s2 = -1;
  }
  RCLCPP_INFO(logger_, "DockSimPlugin state reset.");
}

void DockSimPlugin::cleanup()
{
  RCLCPP_INFO(logger_, "DockSimPlugin cleanup.");
  dock_srv_.reset();
  release_srv_.reset();
  dock_action_.reset();
  release_action_.reset();
  node_.reset();
}

}  // namespace mujoco_dock_sim_plugin

PLUGINLIB_EXPORT_CLASS(mujoco_dock_sim_plugin::DockSimPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
