// ソルバーの解を RViz に表示するノード（#35 のたたき台、#31 の確認用）
//
// 机の上の置き場に並べたピースを、解の順に1個ずつ「持ち上げる → 運びながら回す → ケースに下ろす」
// アニメーションで表示する。今運んでいるピースには、指が閉じる向き（ピースの x 軸）を白い矢印で出す。
// 位置と向きは board_geometry（#31）で計算しているので、表示がケースのマスとずれていれば変換が間違っている。
//
// 起動：ros2 launch tukamukai solver_viz.launch.py
// トピック：solver_viz/markers（visualization_msgs/MarkerArray、フレームは base_link）

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tukamukai/board_geometry.hpp"
#include "tukamukai/solver.hpp"

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace
{

// "I:1,S:1,L:2" を {種類: 個数} にする
tukamukai::PieceCounts parse_pieces(const std::string & text)
{
  tukamukai::PieceCounts counts;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item.erase(std::remove(item.begin(), item.end(), ' '), item.end());
    if (item.empty()) {
      continue;
    }
    const auto colon = item.find(':');
    if (colon != 1 || colon + 1 >= item.size()) {
      throw std::invalid_argument("pieces の書き方が違います（例 \"I:1,S:1,L:2\"）: " + item);
    }
    counts[item[0]] += std::stoi(item.substr(colon + 1));
  }
  return counts;
}

// テトリス標準色
std_msgs::msg::ColorRGBA kind_color(char kind)
{
  static const std::map<char, std::array<float, 3>> colors{
    {'I', {0.0F, 0.80F, 0.90F}},  // 水色
    {'O', {1.0F, 0.85F, 0.0F}},   // 黄
    {'T', {0.60F, 0.20F, 0.80F}},  // 紫
    {'S', {0.20F, 0.80F, 0.20F}},  // 緑
    {'Z', {0.90F, 0.10F, 0.10F}},  // 赤
    {'J', {0.10F, 0.30F, 0.90F}},  // 青
    {'L', {1.0F, 0.55F, 0.0F}},   // オレンジ
  };
  std_msgs::msg::ColorRGBA c;
  const auto & rgb = colors.at(kind);
  c.r = rgb[0];
  c.g = rgb[1];
  c.b = rgb[2];
  c.a = 1.0F;
  return c;
}

geometry_msgs::msg::Pose make_pose(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.z = std::sin(yaw / 2.0);
  pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

geometry_msgs::msg::Point make_point(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// 0→1 をなめらかにつなぐ（動き始めと終わりをゆっくりにする）
double smoothstep(double u)
{
  u = std::clamp(u, 0.0, 1.0);
  return u * u * (3.0 - 2.0 * u);
}

// 3次元の位置と向き（アニメーションの途中の姿勢）
struct Pose3D
{
  double x;
  double y;
  double z;
  double yaw;
};

}  // namespace

class SolverViz : public rclcpp::Node
{
public:
  SolverViz()
  : Node("solver_viz")
  {
    rows_ = declare_parameter("rows", 4);
    cols_ = declare_parameter("cols", 4);
    const auto pieces_text = declare_parameter("pieces", std::string("I:1,S:1,L:2"));
    frame_.x = declare_parameter("case_x", 0.16);
    frame_.y = declare_parameter("case_y", -0.04);
    frame_.z = declare_parameter("case_z", 0.0);
    frame_.yaw = declare_parameter("case_yaw", 0.0);
    frame_.cell_size = declare_parameter("cell_size", tukamukai::kDefaultCellSize);
    piece_height_ = declare_parameter("piece_height", 0.010);
    // 置き場（ピースが待っている場所）。指定しなければケースの左隣（y が大きい側）に置く
    tray_x_ = declare_parameter("tray_x", frame_.x);
    tray_y_ = declare_parameter(
      "tray_y", frame_.y + cols_ * frame_.cell_size + 0.06);
    tray_columns_ = declare_parameter("tray_columns", 4);
    tray_spacing_ = declare_parameter("tray_spacing", 0.09);
    lift_height_ = declare_parameter("lift_height", 0.06);
    step_time_ = declare_parameter("step_time", 2.0);
    loop_ = declare_parameter("loop", true);
    frame_id_ = declare_parameter("frame_id", std::string("base_link"));

    pub_ = create_publisher<MarkerArray>("solver_viz/markers", 10);

    const auto pieces = parse_pieces(pieces_text);
    const auto start = std::chrono::steady_clock::now();
    const auto result = tukamukai::solve(rows_, cols_, pieces);
    const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;

    if (!result) {
      RCLCPP_ERROR(
        get_logger(), "%d×%d を %s では埋められません（ケースだけ表示します）",
        rows_, cols_, pieces_text.c_str());
    } else {
      // TODO(#30): 置く順番が決まったら、ここで並べ替える。今はソルバーが見つけた順
      placements_ = *result;
      print_solution(elapsed.count());
    }

    start_time_ = now();
    clear_markers();
    timer_ = create_wall_timer(std::chrono::milliseconds(33), [this]() {publish();});
  }

private:
  void print_solution(double seconds) const
  {
    std::ostringstream out;
    out << rows_ << "×" << cols_ << " の解（" << seconds << " 秒）\n"
        << tukamukai::render(rows_, cols_, placements_) << "\n"
        << "順番 種類 回転角  base_link での置き場所 x, y [m]  yaw [deg]\n";
    for (size_t i = 0; i < placements_.size(); ++i) {
      const auto & p = placements_[i];
      const auto pose = tukamukai::placement_in_robot(p, frame_);
      char line[128];
      std::snprintf(
        line, sizeof(line), "%4zu   %c   %4d    %7.4f, %7.4f            %7.1f\n",
        i + 1, p.kind, p.rotation, pose.x, pose.y, pose.yaw * 180.0 / M_PI);
      out << line;
    }
    RCLCPP_INFO(get_logger(), "\n%s", out.str().c_str());
  }

  // i 番目のピースが置き場で待っている位置。置き場は tray_spacing 間隔の格子（x 方向に tray_columns 個ずつ）
  Pose3D tray_pose(size_t i) const
  {
    const auto columns = static_cast<size_t>(std::max(1, tray_columns_));
    return {
      tray_x_ + static_cast<double>(i % columns) * tray_spacing_,
      tray_y_ + static_cast<double>(i / columns) * tray_spacing_,
      frame_.z,
      frame_.yaw};
  }

  Pose3D target_pose(size_t i) const
  {
    const auto p = tukamukai::placement_in_robot(placements_[i], frame_);
    return {p.x, p.y, frame_.z, p.yaw};
  }

  // 持ち上げる（0〜0.2）→ 運びながら回す（0.2〜0.8）→ 下ろす（0.8〜1）
  Pose3D moving_pose(size_t i, double u) const
  {
    const Pose3D from = tray_pose(i);
    const Pose3D to = target_pose(i);
    const double move = smoothstep((u - 0.2) / 0.6);
    double z = from.z;
    if (u < 0.2) {
      z += lift_height_ * smoothstep(u / 0.2);
    } else if (u < 0.8) {
      z += lift_height_;
    } else {
      z += lift_height_ * (1.0 - smoothstep((u - 0.8) / 0.2));
    }
    return {
      from.x + (to.x - from.x) * move,
      from.y + (to.y - from.y) * move,
      z,
      from.yaw + tukamukai::normalize_angle(to.yaw - from.yaw) * move};
  }

  Marker base_marker(const std::string & ns, int id, int type) const
  {
    Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = now();
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = Marker::ADD;
    return m;
  }

  void clear_markers()
  {
    MarkerArray array;
    Marker m;
    m.action = Marker::DELETEALL;
    array.markers.push_back(m);
    pub_->publish(array);
  }

  void add_case(MarkerArray & array) const
  {
    const double s = frame_.cell_size;
    const double c = std::cos(frame_.yaw);
    const double sn = std::sin(frame_.yaw);
    const double cx = rows_ * s / 2.0;
    const double cy = cols_ * s / 2.0;

    // ケースの底板（灰色）
    Marker plate = base_marker("case", 0, Marker::CUBE);
    plate.pose = make_pose(
      frame_.x + c * cx - sn * cy, frame_.y + sn * cx + c * cy, frame_.z - 0.001, frame_.yaw);
    plate.scale.x = rows_ * s;
    plate.scale.y = cols_ * s;
    plate.scale.z = 0.002;
    plate.color.r = plate.color.g = plate.color.b = 0.75F;
    plate.color.a = 1.0F;
    array.markers.push_back(plate);

    // マスの線（ケース座標で描いて、マーカーの姿勢でケースの位置に置く）
    Marker grid = base_marker("case", 1, Marker::LINE_LIST);
    grid.pose = make_pose(frame_.x, frame_.y, frame_.z + 0.0005, frame_.yaw);
    grid.scale.x = 0.001;
    grid.color.r = grid.color.g = grid.color.b = 0.2F;
    grid.color.a = 1.0F;
    for (int r = 0; r <= rows_; ++r) {
      grid.points.push_back(make_point(r * s, 0.0, 0.0));
      grid.points.push_back(make_point(r * s, cols_ * s, 0.0));
    }
    for (int col = 0; col <= cols_; ++col) {
      grid.points.push_back(make_point(0.0, col * s, 0.0));
      grid.points.push_back(make_point(rows_ * s, col * s, 0.0));
    }
    array.markers.push_back(grid);

    // ケース座標の向き（赤：x 軸 = 行が増える向き、緑：y 軸 = 列が増える向き）
    for (int axis = 0; axis < 2; ++axis) {
      Marker arrow = base_marker("case_axes", axis, Marker::ARROW);
      const double yaw = frame_.yaw + (axis == 0 ? 0.0 : M_PI / 2.0);
      arrow.pose = make_pose(frame_.x, frame_.y, frame_.z + 0.001, yaw);
      arrow.scale.x = 0.03;
      arrow.scale.y = 0.003;
      arrow.scale.z = 0.003;
      arrow.color.r = axis == 0 ? 1.0F : 0.0F;
      arrow.color.g = axis == 0 ? 0.0F : 0.8F;
      arrow.color.a = 1.0F;
      array.markers.push_back(arrow);
    }
  }

  void add_piece(MarkerArray & array, size_t i, const Pose3D & pose) const
  {
    const auto & p = placements_[i];
    const double s = frame_.cell_size;

    // ピース：基準の向きの4マスを、ピースの位置と向き（yaw）で置く
    Marker piece = base_marker("pieces", static_cast<int>(i), Marker::CUBE_LIST);
    piece.pose = make_pose(pose.x, pose.y, pose.z + piece_height_ / 2.0, pose.yaw);
    piece.scale.x = s * 0.92;  // 隣とのすき間を見せるため少し小さく
    piece.scale.y = s * 0.92;
    piece.scale.z = piece_height_;
    piece.color = kind_color(p.kind);
    for (const auto & [dx, dy] : tukamukai::cell_offsets(p.kind, s)) {
      piece.points.push_back(make_point(dx, dy, 0.0));
    }
    array.markers.push_back(piece);

    // 置く順番と種類
    Marker label = base_marker("order", static_cast<int>(i), Marker::TEXT_VIEW_FACING);
    label.pose = make_pose(pose.x, pose.y, pose.z + piece_height_ + 0.015, 0.0);
    label.scale.z = 0.012;
    label.color.r = label.color.g = label.color.b = 1.0F;
    label.color.a = 1.0F;
    label.text = std::to_string(i + 1) + ":" + p.kind;  // スペースを入れると RViz で文字が離れて表示される
    array.markers.push_back(label);
  }

  void publish()
  {
    MarkerArray array;
    add_case(array);

    const size_t n = placements_.size();
    const double t = (now() - start_time_).seconds();
    // 全部置いたあと、2ステップぶん止めてから最初に戻る
    const double cycle = (static_cast<double>(n) + 2.0) * step_time_;
    const double time = (loop_ && n > 0) ? std::fmod(t, cycle) : t;
    const double step = time / step_time_;
    const size_t current = static_cast<size_t>(step);

    for (size_t i = 0; i < n; ++i) {
      if (i < current) {
        add_piece(array, i, target_pose(i));
      } else if (i > current) {
        add_piece(array, i, tray_pose(i));
      } else {
        const Pose3D pose = moving_pose(i, step - static_cast<double>(current));
        add_piece(array, i, pose);
        add_grasp_arrow(array, pose);
      }
    }
    if (current >= n) {
      Marker arrow = base_marker("grasp", 0, Marker::ARROW);
      arrow.action = Marker::DELETE;
      array.markers.push_back(arrow);
    }
    pub_->publish(array);
  }

  // 指が閉じる向き（ピースの x 軸）を白い矢印で出す。取っ手の長手方向はこれと直角
  void add_grasp_arrow(MarkerArray & array, const Pose3D & pose) const
  {
    Marker arrow = base_marker("grasp", 0, Marker::ARROW);
    const double len = 0.03;
    const double z = pose.z + piece_height_ + 0.003;
    arrow.points.push_back(
      make_point(pose.x - std::cos(pose.yaw) * len / 2, pose.y - std::sin(pose.yaw) * len / 2, z));
    arrow.points.push_back(
      make_point(pose.x + std::cos(pose.yaw) * len / 2, pose.y + std::sin(pose.yaw) * len / 2, z));
    arrow.pose.orientation.w = 1.0;
    arrow.scale.x = 0.002;
    arrow.scale.y = 0.005;
    arrow.scale.z = 0.006;
    arrow.color.r = arrow.color.g = arrow.color.b = 1.0F;
    arrow.color.a = 1.0F;
    array.markers.push_back(arrow);
  }

  int rows_;
  int cols_;
  tukamukai::CaseFrame frame_;
  double piece_height_;
  double tray_x_;
  double tray_y_;
  int tray_columns_;
  double tray_spacing_;
  double lift_height_;
  double step_time_;
  bool loop_;
  std::string frame_id_;
  std::vector<tukamukai::Placement> placements_;
  rclcpp::Time start_time_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SolverViz>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("solver_viz"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
