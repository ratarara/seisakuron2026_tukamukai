#!/usr/bin/env python3
# ピースを色で見つけ、位置と向きを tf piece_<種類>_<番号> として出すノード
#
# fill_4x4（piece_source:=tf）が、この tf を読んでピースをつかみ、置く向きまで手首を回す。
#   ros2 run tukamukai piece_tf_node.py
#
# - 色の切り出し：堀さんの perception/scripts/colorecog の色の範囲と処理を使う
#   （オレンジ＝L、緑＝S、青＝I。中間発表の 3 種類）
# - 向き：ソルバーと同じ形を回して領域と重ね、一番よく重なる角度（C++ の piece_angle と同じ方法、
#   docs/piece_angle.md）
# - 位置：取っ手（領域の重心）を通るカメラの視線と、ピースの上面の高さ（plane_z）の平面との交点。
#   tf2 でカメラのフレームから base_link に直す（demo.launch.py use_d435:=true が必要）
# - アームが動いている間の画像では tf を出さない（カメラの位置の変化で判定）
# - 確認用に、見つけたピースと角度を描いた画像を ~/image に出す

from collections import deque
import math

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

# 色の範囲（HSV、OpenCV の H は 0〜179）。perception/scripts/colorecog（堀さん）と同じ
COLORS = {
    'L': ([5, 100, 100], [25, 255, 255]),    # オレンジ
    'S': ([30, 50, 40], [90, 255, 255]),     # 緑
    'I': ([90, 70, 50], [130, 255, 255]),    # 青（水色を含む）
}

# ソルバーと同じ形（include/tukamukai/solver.hpp の shape()）。(行, 列)
SHAPES = {
    'I': [(0, 0), (0, 1), (0, 2), (0, 3)],
    'O': [(0, 0), (0, 1), (1, 0), (1, 1)],
    'T': [(0, 0), (0, 1), (0, 2), (1, 1)],
    'S': [(0, 1), (0, 2), (1, 0), (1, 1)],
    'Z': [(0, 0), (0, 1), (1, 1), (1, 2)],
    'J': [(0, 0), (1, 0), (1, 1), (1, 2)],
    'L': [(0, 2), (1, 0), (1, 1), (1, 2)],
}


def symmetry_period(kind):
    """形が同じに見える回転の周期 [rad]."""
    if kind == 'O':
        return math.pi / 2
    if kind in ('I', 'S', 'Z'):
        return math.pi
    return 2 * math.pi


def rotate(x, y, yaw):
    """画像を見て反時計回りに yaw 回す（画像は y が下向き）."""
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c + y * s, -x * s + y * c


def piece_cells(kind, yaw, cell_px, cx, cy):
    """形を向き yaw・1 マス cell_px で (cx, cy) に置いたときの、4 マスの四角形の角 [px]."""
    cells = SHAPES[kind]
    mr = sum(r for r, _ in cells) / 4.0
    mc = sum(c for _, c in cells) / 4.0
    result = []
    for r, c in cells:
        corners = []
        for dr, dc in ((-0.5, -0.5), (-0.5, 0.5), (0.5, 0.5), (0.5, -0.5)):
            x, y = rotate((c + dc - mc) * cell_px, (r + dr - mr) * cell_px, yaw)
            corners.append((cx + x, cy + y))
        result.append(corners)
    return result


def estimate_angle(contour, kind, coarse_deg=5.0, fine_deg=0.5):
    """輪郭の内側を 1 つのピースとみなし、(向き [rad], 重心 x, y, 1 マス [px], 重なり) を返す."""
    m = cv2.moments(contour)
    if m['m00'] <= 0:
        return None
    cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
    cell_px = math.sqrt(m['m00'] / 4.0)
    pad = int(math.ceil(cell_px))
    x, y, w, h = cv2.boundingRect(contour)
    ox, oy = x - pad, y - pad
    region = np.zeros((h + 2 * pad, w + 2 * pad), np.uint8)
    cv2.drawContours(region, [contour], 0, 255, cv2.FILLED, offset=(-ox, -oy))
    area = np.count_nonzero(region)

    def overlap(yaw):
        shape = np.zeros_like(region)
        for cell in piece_cells(kind, yaw, cell_px, cx - ox, cy - oy):
            cv2.fillConvexPoly(shape, np.round(np.array(cell)).astype(np.int32), 255)
        both = np.count_nonzero(cv2.bitwise_and(region, shape))
        return both / float(area + np.count_nonzero(shape) - both)

    period = symmetry_period(kind)
    best_yaw, best = 0.0, -1.0
    for a in np.arange(0.0, period, math.radians(coarse_deg)):
        s = overlap(a)
        if s > best:
            best_yaw, best = a, s
    start = best_yaw
    for a in np.arange(start - math.radians(coarse_deg), start + math.radians(coarse_deg) + 1e-9,
                       math.radians(fine_deg)):
        s = overlap(a)
        if s > best:
            best_yaw, best = a, s
    yaw = math.remainder(best_yaw, 2 * math.pi)
    return yaw, cx, cy, cell_px, best


def quat_rotate(q, v):
    """クォータニオン q（x, y, z, w）でベクトル v を回す."""
    qx, qy, qz, qw = q
    u = np.array([qx, qy, qz])
    v = np.array(v, dtype=float)
    return 2 * np.dot(u, v) * u + (qw * qw - np.dot(u, u)) * v + 2 * qw * np.cross(u, v)


class PieceTfNode(Node):

    def __init__(self):
        super().__init__('piece_tf_node')
        self.image_topic = self.declare_parameter('image_topic', '/camera/color/image_raw').value
        self.info_topic = self.declare_parameter(
            'camera_info_topic', '/camera/color/camera_info').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        # ピースの上面の高さ [m]（base_link から）。机の面 ＋ ピースの厚み 5 mm
        self.plane_z = self.declare_parameter('plane_z', 0.005).value
        self.min_area = self.declare_parameter('min_area_px', 500.0).value
        self.min_score = self.declare_parameter('min_score', 0.8).value
        # still_window_sec 秒前の画像からカメラがこれ以上動いていたら、tf を出さない [m]、[deg]
        self.still_window = self.declare_parameter('still_window_sec', 0.3).value
        # 画像の時刻のカメラの位置（tf）がまだ届いていないときは、届いている一番新しい tf を使う。
        # ただし画像の時刻との差がこれ以下のときだけ [s]（関節の角度の tf は 1 秒に 20 回ほどしか
        # 来ない。tf を出すのはカメラが止まっているときだけなので、少しの差なら位置は同じ）
        self.tf_tolerance = self.declare_parameter('tf_tolerance_sec', 0.2).value
        self.max_rate = self.declare_parameter('max_rate_hz', 3.0).value
        self.last_processed = None
        self.still_m = self.declare_parameter('still_tolerance_m', 0.002).value
        self.still_deg = self.declare_parameter('still_tolerance_deg', 1.0).value
        self.show_window = self.declare_parameter('show_window', False).value

        self.bridge = CvBridge()
        self.K = None
        self.camera_history = deque()  # (時刻 [s], カメラの位置と向き)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.image_pub = self.create_publisher(Image, '~/image', 1)
        self.create_subscription(CameraInfo, self.info_topic, self.on_info, 1)
        self.create_subscription(Image, self.image_topic, self.on_image, 1)
        self.get_logger().info(
            f'listening to {self.image_topic}; publishing tf piece_<kind>_<n> '
            f'in {self.base_frame} (plane_z={self.plane_z:.3f} m)')

    def on_info(self, msg):
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)

    def detect(self, bgr):
        """色ごとに領域を見つけ、(種類, 向き, 重心 x, y, 1 マス, 重なり, 輪郭) の一覧を返す."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)
        h, w = bgr.shape[:2]
        found = []
        for kind, (lower, upper) in COLORS.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv2.contourArea(contour) < self.min_area:
                    continue
                x, y, bw, bh = cv2.boundingRect(contour)
                if x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1:
                    continue  # 画像の端で切れている（角度を間違えやすい）
                r = estimate_angle(contour, kind)
                if r is None or r[4] < self.min_score:
                    continue
                found.append((kind,) + r + (contour,))
        found.sort(key=lambda p: p[2])  # 画像の左から右の順
        return found

    def on_image(self, msg):
        # 処理が重いので、max_rate_hz より多くは処理しない（止まって撮るので、これで足りる）
        now = self.get_clock().now()
        if self.last_processed is not None and \
                (now - self.last_processed).nanoseconds * 1e-9 < 1.0 / self.max_rate:
            return
        self.last_processed = now

        # カメラの位置（base_link から）。取れなければ画像の上だけで表示する
        camera = None
        status = ''
        if self.K is None:
            status = 'no camera_info'
        else:
            # tf を待たない（待っている間は、同じスレッドで tf を受け取れないため）
            t = None
            stamp = Time.from_msg(msg.header.stamp)
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, msg.header.frame_id, stamp)
            except TransformException:
                try:
                    latest = self.tf_buffer.lookup_transform(
                        self.base_frame, msg.header.frame_id, Time())
                    lag = (stamp - Time.from_msg(latest.header.stamp)).nanoseconds * 1e-9
                    # 時刻 0 は static tf（カメラの位置を固定で与えたとき）
                    if Time.from_msg(latest.header.stamp).nanoseconds == 0 or \
                            abs(lag) <= self.tf_tolerance:
                        t = latest
                    else:
                        status = f'tf too old ({lag:.2f} s)'
                except TransformException as e:
                    status = f'no tf {self.base_frame} <- {msg.header.frame_id}'
                    self.get_logger().warn(f'{status}: {e}', throttle_duration_sec=5.0)
            if t is not None:
                tr, rot = t.transform.translation, t.transform.rotation
                camera = (np.array([tr.x, tr.y, tr.z]), (rot.x, rot.y, rot.z, rot.w))

        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        pieces = self.detect(bgr)

        stamp_sec = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        still = camera is not None and self.is_still(stamp_sec, camera)
        if camera is not None and not still:
            status = 'camera moving'
        counts = {}
        for kind, yaw, cx, cy, cell_px, score, contour in pieces:
            n = counts.get(kind, 0)
            counts[kind] = n + 1
            name = f'piece_{kind}_{n}'
            label = f'{name} {math.degrees(yaw):.0f}deg ({score:.2f})'
            if camera is not None:
                pos, robot_yaw = self.to_robot(camera, cx, cy, yaw)
                if pos is not None:
                    label += f' -> ({pos[0]:.3f}, {pos[1]:.3f}) {math.degrees(robot_yaw):.0f}deg'
                    if still:
                        self.send_tf(msg.header.stamp, name, pos, robot_yaw)
            self.draw(bgr, kind, yaw, cx, cy, cell_px, contour, label)
        if status:
            cv2.putText(bgr, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        out = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        out.header = msg.header
        self.image_pub.publish(out)
        if self.show_window:
            cv2.imshow('piece_tf_node', bgr)
            cv2.waitKey(1)

    def is_still(self, now, camera):
        """still_window_sec 秒前からカメラがほとんど動いていなければ True."""
        history = self.camera_history
        history.append((now, camera))
        while len(history) > 1 and now - history[1][0] >= self.still_window:
            history.popleft()  # 窓より古いものは、窓のちょうど外側の 1 つだけ残す
        then, prev = history[0]
        if now - then < self.still_window:
            return False  # まだ窓の長さ分の記録がない
        moved = np.linalg.norm(camera[0] - prev[0])
        dot = abs(float(np.dot(camera[1], prev[1])))
        turned = math.degrees(2 * math.acos(min(1.0, dot)))
        return moved <= self.still_m and turned <= self.still_deg

    def to_robot(self, camera, cx, cy, image_yaw):
        """画像上の点と向きを、base_link での位置と yaw にする（ピースの上面との交点）."""
        origin, q = camera
        k = self.K
        ray = np.array([(cx - k[0, 2]) / k[0, 0], (cy - k[1, 2]) / k[1, 1], 1.0])
        d = quat_rotate(q, ray)
        if abs(d[2]) < 1e-6:
            return None, None
        s = (self.plane_z - origin[2]) / d[2]
        if s <= 0:
            return None, None
        pos = origin + s * d
        # 画像の下向き（光軸のフレームの y 軸）が base_link で向いている角度 ＋ 画像上の向き
        down = quat_rotate(q, [0.0, 1.0, 0.0])
        robot_yaw = math.remainder(math.atan2(down[1], down[0]) + image_yaw, 2 * math.pi)
        return pos, robot_yaw

    def send_tf(self, stamp, name, pos, yaw):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.base_frame
        t.child_frame_id = name
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(self.plane_z)
        t.transform.rotation.z = math.sin(yaw / 2)
        t.transform.rotation.w = math.cos(yaw / 2)
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def draw(bgr, kind, yaw, cx, cy, cell_px, contour, label):
        cv2.drawContours(bgr, [contour], 0, (255, 255, 255), 1)
        for cell in piece_cells(kind, yaw, cell_px, cx, cy):
            cv2.polylines(bgr, [np.round(np.array(cell)).astype(np.int32)], True, (0, 0, 0), 2)
        dx, dy = rotate(0.0, 1.5 * cell_px, yaw)
        cv2.arrowedLine(bgr, (int(cx), int(cy)), (int(cx + dx), int(cy + dy)), (0, 0, 255), 2,
                        tipLength=0.25)
        at = (int(cx - 40), int(cy - 0.9 * cell_px))
        cv2.putText(bgr, label, at, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(bgr, label, at, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)


def main():
    rclpy.init()
    node = PieceTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
