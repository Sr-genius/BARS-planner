// bars_planner.cpp
//
// Reactive 3-D safety-corridor planner. A time-decayed voxel map feeds a
// three-dimensional A* guide, which is expanded into an overlapping free-space
// corridor and a hard-projected smooth path. A braking-aware barrier bounds the
// closing speed against static and dynamic obstacles, and the resulting
// command is tracked under the acceleration limit.
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <string>

using Eigen::Vector3d;

// ---------------- configurable parameters ----------------
struct Params {
  // Occupancy and admissible altitude band
  double r_infl     = 0.40;   // a cell is free when its 3D clearance exceeds this
  double z_min      = 0.6;
  double z_max      = 2.0;
  double z_cruise   = 1.0;

  // Vehicle motion limits
  double v_max      = 1.0;
  double a_max      = 1.0;

  // Global guide search
  double res_g      = 0.6;    // grid resolution (m)
  double w_climb    = 2.5;    // weight applied to vertical steps
  double plan_int   = 0.8;    // replanning period (s)
  double lookahead  = 0.9;    // lookahead distance along the path (m)

  // Safety barrier
  double cbf_alpha    = 3.0;
  double cbf_r_safe   = 0.42;
  double cbf_infl     = 2.0;  // barrier active radius
  int    cbf_K        = 5;    // nearest obstacles considered
  double cbf_brake_margin = 0.05;  // extra stop margin outside the safety radius

  // Dynamic objects use a larger, independent clearance
  double ped_r_safe  = 0.60;
  double ped_alpha   = 4.0;

  // Goal approach
  double goal_r      = 0.01;  // arrival radius
  double approach_min_v = 0.05;  // floor on the approach speed

  // Heading control
  double yaw_rate_max   = 2.0;
  double yaw_turn_min_v = 0.15;
  double yaw_lock_dist  = 0.7;

  // Clearance term in the guide cost
  double clear_ref  = 0.9;
  double w_clear    = 0.8;

  // Occupancy grid origin and extent in the world frame
  double ox = -10.0;
  double oy = -13.0;
  double gsize_x = 20.0;
  double gsize_y = 26.0;
};
static Params cfg;

// Wrap an angle into (-pi, pi] so a turn across the +/-pi seam never commands
// a long reverse rotation.
static double wrapPi(double a) {
  if (a >  M_PI) a -= 2.0 * M_PI;
  if (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// ---------------- time-decayed voxel map ----------------
// Static structure is re-observed every frame and retained. Measurements that
// are not refreshed within the retention window are removed, so moving
// objects do not persist as phantom walls.
struct VoxelMap {
  pcl::KdTreeFLANN<pcl::PointXYZ> kd;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  bool ready = false;

  double voxel_size = 0.1;
  double t_keep = 12.0;  // seconds a point stays in the map without re-observation

  static constexpr size_t HARD_CAP = 400000;
  static constexpr size_t CAP_TARGET = 300000;

  struct StoredPoint { pcl::PointXYZ p; double t; };
  std::unordered_map<long long, StoredPoint> cells;

  long long keyOf(double x, double y, double z) const {
    long long i = llround(x / voxel_size),
              j = llround(y / voxel_size),
              k = llround(z / voxel_size);
    return ((i + 512) << 20) | ((j + 512) << 10) | (k + 64);
  }

  void insert(const pcl::PointCloud<pcl::PointXYZ>& c, double now) {
    for (const auto& p : c.points) {
      long long k = keyOf(p.x, p.y, p.z);
      auto it = cells.find(k);
      if (it == cells.end()) {
        cells[k] = StoredPoint{p, now};
      } else {
        it->second.p = p;
        it->second.t = now;
      }
    }
    // Bound the map size so the nearest-neighbour rebuild stays tractable.
    if (cells.size() > HARD_CAP) {
      auto it = cells.begin();
      size_t drop = cells.size() - CAP_TARGET;
      while (it != cells.end() && drop-- > 0) it = cells.erase(it);
    }
  }

  void prune(double now) {
    for (auto it = cells.begin(); it != cells.end();) {
      if (now - it->second.t > t_keep) it = cells.erase(it);
      else ++it;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr buildCloud() {
    auto out = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    out->reserve(cells.size());
    for (const auto& kv : cells) out->push_back(kv.second.p);
    return out;
  }

  void rebuild(double now) {
    prune(now);
    cloud = buildCloud();
    kd.setInputCloud(cloud);
    ready = true;
  }

  void clear() { cells.clear(); ready = false; }

  double clearance(const Vector3d& p) const {
    if (!ready) return 99.0;
    pcl::PointXYZ q; q.x = p.x(); q.y = p.y(); q.z = p.z();
    std::vector<int> idx(1);
    std::vector<float> d2(1);
    if (kd.nearestKSearch(q, 1, idx, d2) <= 0) return 99.0;
    return std::sqrt(static_cast<double>(d2[0]));
  }

  std::vector<Vector3d> nearby(const Vector3d& p, double radius, int maxn) const {
    std::vector<Vector3d> out;
    if (!ready) return out;
    pcl::PointXYZ q; q.x = p.x(); q.y = p.y(); q.z = p.z();
    std::vector<int> idx;
    std::vector<float> d2;
    int n = kd.radiusSearch(q, radius, idx, d2);
    for (int i = 0; i < n && static_cast<int>(out.size()) < maxn; ++i) {
      const auto& pt = cloud->points[idx[i]];
      out.push_back(Vector3d(pt.x, pt.y, pt.z));
    }
    return out;
  }
};

// ---------------- planner ----------------
class BarsPlanner {
 public:
  BarsPlanner() : nh_(), pnh_("~") {
    loadParams();
    resolveTopics();

    cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &BarsPlanner::cloudCb, this);
    dynamic_sub_ = nh_.subscribe(dynamic_topic_, 1, &BarsPlanner::dynamicCb, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &BarsPlanner::odomCb, this);
    goal_sub_ = nh_.subscribe(goal_topic_, 1, &BarsPlanner::goalCb, this);
    cancel_sub_ = nh_.subscribe(cancel_topic_, 1, &BarsPlanner::cancelCb, this);
    cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(command_topic_, 1);

    ROS_INFO_STREAM("[bars] planner ready, publishing " << command_topic_);
  }

 private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber cloud_sub_, dynamic_sub_, odom_sub_, goal_sub_, cancel_sub_;
  ros::Publisher cmd_pub_;

  VoxelMap map_;

  // Interface topics
  std::string cloud_topic_, dynamic_topic_, odom_topic_, goal_topic_,
              cancel_topic_, command_topic_;

  Vector3d pose_ = Vector3d(0, 0, 1);
  Vector3d goal_ = Vector3d(0, 1, 1);
  bool has_cloud_ = false, has_odom_ = false, has_goal_ = false, cancel_ = false;
  bool grid_dirty_ = false;

  std::vector<Vector3d> dynamic_pts_;  // latest dynamic-object cloud (world frame)

  // Occupancy grid
  int nx_ = 0, ny_ = 0, nz_ = 0;
  std::vector<char> free_cells_;
  std::vector<float> cell_clearance_;
  std::vector<Vector3d> path_;
  std::vector<double> arc_;        // cumulative arc length
  std::vector<double> ball_r_;     // free-space ball radius per path point
  double total_arc_ = 0.0;

  Vector3d executed_v_ = Vector3d::Zero();
  ros::Time last_plan_;
  bool arrived_ = false;
  double yaw_hold_ = 0.0;
  bool yaw_inited_ = false;

  int idxOf(int i, int j, int k) const { return (i * ny_ + j) * nz_ + k; }

  bool inGrid(int i, int j, int k) const {
    return i >= 0 && i < nx_ && j >= 0 && j < ny_ && k >= 0 && k < nz_;
  }

  Vector3d gridPos(int i, int j, int k) const {
    return Vector3d(cfg.ox + (i + 0.5) * cfg.res_g,
                    cfg.oy + (j + 0.5) * cfg.res_g,
                    cfg.z_min + (k + 0.5) * cfg.res_g);
  }

  void posToGrid(const Vector3d& p, int& i, int& j, int& k) {
    i = static_cast<int>(std::floor((p.x() - cfg.ox) / cfg.res_g));
    j = static_cast<int>(std::floor((p.y() - cfg.oy) / cfg.res_g));
    k = static_cast<int>(std::floor((p.z() - cfg.z_min) / cfg.res_g));
    i = std::max(0, std::min(nx_ - 1, i));
    j = std::max(0, std::min(ny_ - 1, j));
    k = std::max(0, std::min(nz_ - 1, k));
  }

  void resolveTopics() {
    pnh_.param<std::string>("cloud_topic", cloud_topic_, "pointcloud");
    pnh_.param<std::string>("dynamic_object_topic", dynamic_topic_, "dynamic_objects");
    pnh_.param<std::string>("odom_topic", odom_topic_, "odometry");
    pnh_.param<std::string>("goal_topic", goal_topic_, "goal");
    pnh_.param<std::string>("cancel_topic", cancel_topic_, "cancel");
    pnh_.param<std::string>("command_topic", command_topic_, "position_command");
  }

  void loadParams() {
    pnh_.param("r_infl", cfg.r_infl, cfg.r_infl);
    pnh_.param("z_min", cfg.z_min, cfg.z_min);
    pnh_.param("z_max", cfg.z_max, cfg.z_max);
    pnh_.param("z_cruise", cfg.z_cruise, cfg.z_cruise);
    pnh_.param("v_max", cfg.v_max, cfg.v_max);
    pnh_.param("a_max", cfg.a_max, cfg.a_max);
    pnh_.param("res_g", cfg.res_g, cfg.res_g);
    pnh_.param("w_climb", cfg.w_climb, cfg.w_climb);
    pnh_.param("plan_int", cfg.plan_int, cfg.plan_int);
    pnh_.param("lookahead", cfg.lookahead, cfg.lookahead);
    pnh_.param("cbf_alpha", cfg.cbf_alpha, cfg.cbf_alpha);
    pnh_.param("cbf_r_safe", cfg.cbf_r_safe, cfg.cbf_r_safe);
    pnh_.param("cbf_infl", cfg.cbf_infl, cfg.cbf_infl);
    pnh_.param("cbf_K", cfg.cbf_K, cfg.cbf_K);
    pnh_.param("cbf_brake_margin", cfg.cbf_brake_margin, cfg.cbf_brake_margin);
    pnh_.param("ped_r_safe", cfg.ped_r_safe, cfg.ped_r_safe);
    pnh_.param("ped_alpha", cfg.ped_alpha, cfg.ped_alpha);
    pnh_.param("goal_r", cfg.goal_r, cfg.goal_r);
    pnh_.param("approach_min_v", cfg.approach_min_v, cfg.approach_min_v);
    pnh_.param("yaw_rate_max", cfg.yaw_rate_max, cfg.yaw_rate_max);
    pnh_.param("yaw_turn_min_v", cfg.yaw_turn_min_v, cfg.yaw_turn_min_v);
    pnh_.param("yaw_lock_dist", cfg.yaw_lock_dist, cfg.yaw_lock_dist);
    pnh_.param("clear_ref", cfg.clear_ref, cfg.clear_ref);
    pnh_.param("w_clear", cfg.w_clear, cfg.w_clear);
    pnh_.param("grid_ox", cfg.ox, cfg.ox);
    pnh_.param("grid_oy", cfg.oy, cfg.oy);
    pnh_.param("grid_sx", cfg.gsize_x, cfg.gsize_x);
    pnh_.param("grid_sy", cfg.gsize_y, cfg.gsize_y);
    pnh_.param("t_keep", map_.t_keep, map_.t_keep);
    pnh_.param("map_voxel_size", map_.voxel_size, map_.voxel_size);
  }

  void buildFreeGrid() {
    nx_ = static_cast<int>(std::ceil(cfg.gsize_x / cfg.res_g));
    ny_ = static_cast<int>(std::ceil(cfg.gsize_y / cfg.res_g));
    nz_ = static_cast<int>(std::floor((cfg.z_max - cfg.z_min) / cfg.res_g));
    if (nz_ < 1) nz_ = 1;

    free_cells_.assign(nx_ * ny_ * nz_, 0);
    cell_clearance_.assign(nx_ * ny_ * nz_, 99.0f);

    for (int i = 0; i < nx_; ++i)
      for (int j = 0; j < ny_; ++j)
        for (int k = 0; k < nz_; ++k) {
          Vector3d c = gridPos(i, j, k);
          float cl = static_cast<float>(map_.clearance(c));
          int id = idxOf(i, j, k);
          cell_clearance_[id] = cl;
          free_cells_[id] = (cl > cfg.r_infl) ? 1 : 0;
        }

    ROS_INFO_STREAM("[bars] free grid " << nx_ << "x" << ny_ << "x" << nz_
                    << ", free cells "
                    << std::count(free_cells_.begin(), free_cells_.end(), 1));
  }

  // Three-dimensional A* over the 26-connected free cells.
  bool astar(std::vector<Vector3d>& out) {
    int si, sj, sk, ti, tj, tk;
    posToGrid(pose_, si, sj, sk);
    posToGrid(goal_, ti, tj, tk);
    if (nx_ <= 0 || ny_ <= 0 || nz_ <= 0) return false;

    free_cells_[idxOf(si, sj, sk)] = 1;
    free_cells_[idxOf(ti, tj, tk)] = 1;

    auto heuristic = [&](int i, int j, int k) {
      Vector3d a = gridPos(i, j, k), b = gridPos(ti, tj, k);
      return (a - b).norm();
    };

    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    std::unordered_map<int, double> g;
    std::unordered_map<int, int> parent;

    int start_id = idxOf(si, sj, sk);
    int goal_id = idxOf(ti, tj, tk);
    g[start_id] = 0.0;
    open.push({heuristic(si, sj, sk), start_id});

    std::vector<int> neighbour_code;
    for (int di = -1; di <= 1; ++di)
      for (int dj = -1; dj <= 1; ++dj)
        for (int dk = -1; dk <= 1; ++dk)
          if (di || dj || dk) neighbour_code.push_back((di * 100 + dj) * 10 + dk);

    int expanded = 0;
    bool found = false;
    while (!open.empty() && expanded < 20000) {
      Entry top = open.top();
      open.pop();
      double f = top.first;
      int cur = top.second;
      if (g.count(cur) && f > g[cur] + heuristic(0, 0, 0) + 1e-3) continue;

      int ci = cur / (ny_ * nz_);
      int rem = cur % (ny_ * nz_);
      int cj = rem / nz_;
      int ck = rem % nz_;
      if (cur == goal_id) { found = true; break; }
      ++expanded;

      for (int code : neighbour_code) {
        int di = code / 1000;
        int r2 = code % 1000;
        int dj = r2 / 10;
        int dk = r2 % 10;
        int ni = ci + di, nj = cj + dj, nk = ck + dk;
        if (!inGrid(ni, nj, nk)) continue;

        int nid = idxOf(ni, nj, nk);
        if (!free_cells_[nid]) continue;

        double step = std::sqrt(static_cast<double>(di * di + dj * dj + dk * dk))
                      * cfg.res_g;
        double climb = cfg.w_climb * std::fabs(static_cast<double>(dk)) * cfg.res_g;

        // Cells below the desired clearance carry a margin penalty. Free cells
        // never pay beyond this, so the admissible search still terminates.
        double margin = 0.0;
        float cc = cell_clearance_[nid];
        if (cc < cfg.clear_ref) margin = cfg.w_clear * (cfg.clear_ref - cc);

        double ng = g[cur] + step + climb + margin;
        if (!g.count(nid) || ng < g[nid]) {
          g[nid] = ng;
          parent[nid] = cur;
          open.push({ng + heuristic(ni, nj, nk), nid});
        }
      }
    }

    if (!found) {
      ROS_WARN_STREAM("[bars] guide search failed after " << expanded << " expansions");
      return false;
    }

    std::vector<int> cells;
    int c = goal_id;
    while (true) {
      cells.push_back(c);
      if (c == start_id) break;
      c = parent[c];
    }
    std::reverse(cells.begin(), cells.end());

    out.clear();
    for (int cell : cells) {
      int i = cell / (ny_ * nz_);
      int rem = cell % (ny_ * nz_);
      int j = rem / nz_;
      int k = rem % nz_;
      out.push_back(gridPos(i, j, k));
    }
    return true;
  }

  // Build the smooth path and per-point free-space balls along the guide.
  void buildPath(const std::vector<Vector3d>& guide) {
    path_ = guide;
    ball_r_.assign(path_.size(), 0.3);

    for (size_t i = 0; i < path_.size(); ++i) {
      double cl = map_.clearance(path_[i]);
      ball_r_[i] = std::max(0.2, std::min(2.0, cl - cfg.r_infl));
    }

    // Laplacian relaxation, with every smoothed point hard-projected back into
    // its guide free-space ball so the path cannot leave collision-free space.
    for (int it = 0; it < 6; ++it) {
      for (size_t i = 1; i + 1 < path_.size(); ++i) {
        Vector3d sm = 0.25 * path_[i - 1] + 0.5 * path_[i] + 0.25 * path_[i + 1];
        Vector3d d = sm - path_[i];
        double lim = 0.75 * ball_r_[i];
        if (d.norm() > lim) d = d.normalized() * lim;

        Vector3d next = path_[i] + d;
        Vector3d e = next - guide[i];
        if (e.norm() > ball_r_[i])
          next = guide[i] + e.normalized() * ball_r_[i];

        next.z() = std::max(cfg.z_min, std::min(cfg.z_max, next.z()));
        path_[i] = next;
      }
    }

    arc_.assign(path_.size(), 0.0);
    total_arc_ = 0.0;
    for (size_t i = 1; i < path_.size(); ++i) {
      total_arc_ += (path_[i] - path_[i - 1]).norm();
      arc_[i] = total_arc_;
    }
  }

  void plan() {
    if (grid_dirty_) {
      buildFreeGrid();
      grid_dirty_ = false;
    }
    std::vector<Vector3d> guide;
    if (!astar(guide)) {
      ROS_WARN("[bars] planning failed, keeping the previous path");
      return;
    }
    buildPath(guide);
    last_plan_ = ros::Time::now();

    double mn = 1e9, mx = -1e9;
    for (const auto& p : path_) { mn = std::min(mn, p.z()); mx = std::max(mx, p.z()); }
    ROS_INFO_STREAM("[bars] path with " << path_.size() << " points, arc "
                    << total_arc_ << " m, altitude [" << mn << ", " << mx << "] m");
  }

  // Lookahead point at a fixed arc distance ahead of the nearest path point.
  Vector3d lookaheadPoint() {
    if (path_.size() < 2) return goal_;

    size_t best = 0;
    double bd = 1e18;
    for (size_t i = 0; i < path_.size(); ++i) {
      double d = (path_[i] - pose_).norm();
      if (d < bd) { bd = d; best = i; }
    }

    double target_arc = arc_[best] + cfg.lookahead;
    size_t i = best;
    while (i + 1 < path_.size() && arc_[i + 1] < target_arc) ++i;
    if (i + 1 >= path_.size()) return path_.back();

    double seg = arc_[i + 1] - arc_[i];
    double t = seg > 1e-3 ? (target_arc - arc_[i]) / seg : 0.0;
    return path_[i] + t * (path_[i + 1] - path_[i]);
  }

  // Safety barrier for static and dynamic obstacles.
  Vector3d barrierFilter(const Vector3d& p, Vector3d v) {
    auto obstacles = map_.nearby(p, cfg.cbf_infl, cfg.cbf_K);

    // Braking-aware constraint. The approach speed is bounded by what constant
    // braking at the acceleration limit can dissipate in the room before the
    // safety radius, so the constraint is feasible under the downstream tracker.
    for (const auto& oj : obstacles) {
      Vector3d n = p - oj;
      double d = n.norm();
      if (d < 1e-3) continue;
      n /= d;
      double room = std::max(0.0, d - cfg.cbf_r_safe - cfg.cbf_brake_margin);
      double v_allow = std::sqrt(2.0 * cfg.a_max * room);
      double v_app = -n.dot(v);  // positive when closing
      if (v_app > v_allow) v += (v_app - v_allow) * n;
    }

    // Linear barrier retained as a final guard for any residual violation.
    for (const auto& oj : obstacles) {
      Vector3d n = p - oj;
      double d = n.norm();
      if (d < 1e-3) continue;
      n /= d;
      double h = d - cfg.cbf_r_safe;
      double limit = -cfg.cbf_alpha * h;
      double nv = n.dot(v);
      if (nv < limit) v += (limit - nv) * n;
    }

    // Dynamic objects use the larger independent clearance. The residual
    // correction is applied along the corridor axis (the lateral component is
    // removed), so the vehicle yields in the travel direction and in height.
    for (const auto& oj : dynamic_pts_) {
      Vector3d n = p - oj;
      double d = n.norm();
      if (d < 1e-3) continue;
      n /= d;
      double room = std::max(0.0, d - cfg.ped_r_safe - cfg.cbf_brake_margin);
      double v_allow = std::sqrt(2.0 * cfg.a_max * room);
      double v_app = -n.dot(v);
      if (v_app > v_allow) v += (v_app - v_allow) * n;
    }

    for (const auto& oj : dynamic_pts_) {
      Vector3d n = p - oj;
      double d = n.norm();
      if (d < 1e-3) continue;
      n /= d;
      double h = d - cfg.ped_r_safe;
      double limit = -cfg.ped_alpha * h;
      double nv = n.dot(v);
      if (nv < limit) {
        Vector3d push = n;
        push.y() = 0.0;  // corridor axis and altitude only
        if (push.squaredNorm() < 1e-6) push = Vector3d(1, 0, 0);
        push.normalize();
        v += (limit - nv) * push;
      }
    }
    return v;
  }

  // ---------------- ROS callbacks ----------------
  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& m) {
    auto pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*m, *pc);
    double now = ros::Time::now().toSec();
    map_.insert(*pc, now);
    map_.rebuild(now);
    grid_dirty_ = true;
    has_cloud_ = true;
  }

  void dynamicCb(const sensor_msgs::PointCloud2ConstPtr& m) {
    auto pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*m, *pc);
    dynamic_pts_.clear();
    dynamic_pts_.reserve(pc->size());
    for (const auto& p : pc->points) dynamic_pts_.push_back(Vector3d(p.x, p.y, p.z));
  }

  void odomCb(const nav_msgs::OdometryConstPtr& m) {
    pose_ = Vector3d(m->pose.pose.position.x, m->pose.pose.position.y,
                     m->pose.pose.position.z);
    has_odom_ = true;

    // Initialise the commanded heading from the current yaw once so the first
    // command never jumps from an assumed zero heading.
    if (!yaw_inited_) {
      const auto& q = m->pose.pose.orientation;
      double siny = 2.0 * (q.w * q.z + q.x * q.y);
      double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      yaw_hold_ = std::atan2(siny, cosy);
      yaw_inited_ = true;
    }
  }

  void goalCb(const geometry_msgs::PoseStampedConstPtr& m) {
    goal_ = Vector3d(m->pose.position.x, m->pose.position.y, m->pose.position.z);
    has_goal_ = true;
    arrived_ = false;
    executed_v_ = Vector3d::Zero();
    path_.clear();
    ROS_INFO_STREAM("[bars] goal set to (" << goal_.x() << ", " << goal_.y()
                    << ", " << goal_.z() << ")");
  }

  void cancelCb(const std_msgs::BoolConstPtr& m) {
    cancel_ = m->data;
    if (cancel_) {
      path_.clear();
      arrived_ = false;
      executed_v_ = Vector3d::Zero();
    }
  }

  void run() {
    ros::Rate rate(50);
    const double dt = 0.02;

    while (ros::ok()) {
      ros::spinOnce();

      if (!cancel_ && has_goal_ && has_cloud_ && has_odom_) {
        ros::Time now = ros::Time::now();
        if (path_.empty() || (now - last_plan_).toSec() > cfg.plan_int) plan();

        double goal_xy = (goal_.head<2>() - pose_.head<2>()).norm();
        Vector3d v_des;

        if (goal_xy < cfg.goal_r) {
          arrived_ = true;
          v_des = Vector3d::Zero();
        } else {
          Vector3d to = lookaheadPoint() - pose_;
          double d = to.norm();
          if (d < 1e-3) {
            v_des = Vector3d::Zero();
          } else {
            v_des = to / d * cfg.v_max;
            // Slow down near the goal, with a floor that keeps the vehicle
            // closing the final distance instead of converging asymptotically.
            if (goal_xy < 2.0) {
              double k = std::max(goal_xy / 2.0, cfg.approach_min_v / cfg.v_max);
              v_des *= k;
            }
          }
        }

        v_des = barrierFilter(pose_, v_des);
        if (v_des.norm() > cfg.v_max) v_des = v_des.normalized() * cfg.v_max;

        // Align the heading with the executed velocity so the nose points where
        // the vehicle actually moves. The heading is held at low speed and near
        // the goal to reject noise and overshoot.
        double horizontal_speed = executed_v_.head<2>().norm();
        if (horizontal_speed >= cfg.yaw_turn_min_v && goal_xy >= cfg.yaw_lock_dist) {
          double yaw_des = std::atan2(executed_v_.y(), executed_v_.x());
          double yerr = wrapPi(yaw_des - yaw_hold_);
          double ystep = cfg.yaw_rate_max * dt;
          yerr = std::max(-ystep, std::min(ystep, yerr));
          yaw_hold_ = wrapPi(yaw_hold_ + yerr);
        }

        Vector3d dv = v_des - executed_v_;
        double max_dv = cfg.a_max * dt;
        if (dv.norm() > max_dv) dv = dv.normalized() * max_dv;
        executed_v_ += dv;
        if (executed_v_.norm() > cfg.v_max)
          executed_v_ = executed_v_.normalized() * cfg.v_max;

        Vector3d next = pose_ + executed_v_ * dt;
        next.z() = std::max(cfg.z_min, std::min(cfg.z_max, next.z()));

        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.position.x = next.x(); cmd.position.y = next.y(); cmd.position.z = next.z();
        cmd.velocity.x = executed_v_.x(); cmd.velocity.y = executed_v_.y();
        cmd.velocity.z = executed_v_.z();
        cmd.acceleration.x = dv.x() / dt; cmd.acceleration.y = dv.y() / dt;
        cmd.acceleration.z = dv.z() / dt;
        cmd.yaw = yaw_hold_;
        cmd.yaw_dot = 0.0;
        cmd_pub_.publish(cmd);
      }

      rate.sleep();
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "bars_planner");
  BarsPlanner planner;
  planner.run();
  return 0;
}
