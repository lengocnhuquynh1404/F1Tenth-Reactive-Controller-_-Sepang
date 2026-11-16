#!/usr/bin/env python3
"""
Fast Follow-the-Gap + Disparity Extender (v3)  -- F1Tenth gym, ROS 2 Humble
  * AUTO SPEED-UP after every clean lap, local slow-down where the car touched a wall
  * PRINTS EVERY LAP TIME + writes them to a CSV file

Run (same command as before):
  python3 fast_gap_controller.py --ros-args -p vmax:=2.0
Race run (locked at the best learned speed, no further tuning):
  python3 fast_gap_controller.py --ros-args -p vmax:=2.0 -p load:=true -p freeze:=true

Main parameters: vmax (starting speed)   vstep (increase per clean lap, default 0.25)
                 vmax_max (cap, 6.0)     autotune (true/false)   load / freeze
"""
import math
import json
import os
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from ackermann_msgs.msg import AckermannDriveStamped
except ImportError:          # allows testing the core outside ROS
    Node = object


class FastGapCore:
    def __init__(self, p, log=print):
        self.log = log
        g = lambda k, d: p.get(k, d)
        self.vmax = float(g('vmax', 2.0))
        self.vmin = g('vmin', 1.0)
        self.decel = g('decel', 4.0)
        self.dmargin = g('dmargin', 0.8)
        self.half_w = g('half_width', 0.30)
        self.disp_thresh = g('disp_thresh', 0.5)
        self.max_range = g('max_range', 10.0)
        self.fov = math.radians(g('fov_deg', 90.0))
        self.kp = g('kp', 1.0)
        self.sref = g('sref', 0.25)
        self.alpha = g('alpha', 0.8)
        self.max_steer = 0.4189
        self.prev_steer = 0.0
        # ---- auto speed-up ----
        self.autotune = bool(g('autotune', True))
        self.vstep = g('vstep', 0.25)
        self.vmax_max = g('vmax_max', 6.0)
        self.clear_thr = g('clear_thr', 0.17)
        self.freeze = bool(g('freeze', False))
        self.state_file = g('state_file', 'fast_gap_state.json')
        self.log_file = g('log_file', 'laps.csv')
        self.good_vmax = None
        self.best_time = None
        self.bad_vmax = None
        self.bad_count = 0
        self.locked = self.freeze
        self.zones = []
        self.laps_done = 0
        self.times = []
        self.start = None
        self.left = False
        self.lap_t0 = None
        self.reset_stats()
        if g('load', False) and os.path.exists(self.state_file):
            s = json.load(open(self.state_file))
            self.good_vmax, self.best_time = s.get('good_vmax'), s.get('best_time')
            self.bad_vmax, self.zones = s.get('bad_vmax'), s.get('zones', [])
            if self.good_vmax:
                self.vmax = self.good_vmax
            self.locked = bool(s.get('locked', False)) or self.freeze
            self.log(f'Loaded state: vmax={self.vmax:.2f}, {len(self.zones)} slow zones, locked={self.locked}')
        if self.log_file and not self.freeze and not os.path.exists(self.log_file):
            open(self.log_file, 'w').write('lap,time_s,vmax,min_clearance_m,result\n')

    def reset_stats(self):
        self.min_clear = 99.0
        self.worst_xy = None

    def save(self):
        if self.freeze:
            return
        json.dump({'good_vmax': self.good_vmax, 'best_time': self.best_time,
                   'bad_vmax': self.bad_vmax, 'locked': self.locked, 'zones': self.zones},
                  open(self.state_file, 'w'))

    # ------------------------------------------------------------ utilities
    @staticmethod
    def side_clearance(ranges, amin, inc):
        ang = amin + np.arange(len(ranges)) * inc
        rr = np.nan_to_num(np.array(ranges, dtype=float), nan=10.0, posinf=10.0)
        rr = np.where(rr <= 0.01, 10.0, rr)
        lat = rr * np.abs(np.sin(ang))
        m = (np.abs(ang) >= math.radians(60)) & (np.abs(ang) <= math.radians(120))
        return float(lat[m].min()) if m.any() else 99.0

    def zone_factor(self, x, y):
        f = 1.0
        for zx, zy, zf in self.zones:
            if math.hypot(x - zx, y - zy) < 4.0:
                f = min(f, zf)
        return f

    # ------------------------------------------------------------ end of lap
    def lap_done(self, t):
        lap_t = t - self.lap_t0
        self.lap_t0 = t
        self.laps_done += 1
        self.times.append(lap_t)
        clean = self.min_clear >= self.clear_thr
        best = min(self.times)
        self.log(f'=== LAP {self.laps_done}: {lap_t:.2f} s | best {best:.2f} s | vmax={self.vmax:.2f} | '
                 f'closest wall {self.min_clear:.2f} m | {"CLEAN" if clean else "DIRTY"} ===')
        if self.log_file and not self.freeze:
            with open(self.log_file, 'a') as f:
                f.write(f'{self.laps_done},{lap_t:.2f},{self.vmax:.2f},{self.min_clear:.2f},'
                        f'{"clean" if clean else "dirty"}\n')
        if self.autotune and not self.freeze:
            if clean:
                if self.good_vmax is None or self.vmax >= self.good_vmax:
                    self.good_vmax, self.best_time = self.vmax, lap_t
                self.bad_count = 0
                if not self.locked:
                    nxt = self.vmax + self.vstep
                    if nxt <= self.vmax_max + 1e-9 and (self.bad_vmax is None or nxt < self.bad_vmax - 1e-9):
                        self.vmax = nxt
                    else:
                        self.locked = True
            else:
                self.bad_count += 1
                if self.worst_xy is not None and self.bad_count <= 3:
                    x, y = self.worst_xy
                    for z in self.zones:
                        if math.hypot(x - z[0], y - z[1]) < 4.0:
                            z[2] = max(0.4, z[2] * 0.8)
                            break
                    else:
                        self.zones.append([x, y, 0.8])
                    self.log(f'    -> wall contact at x={x:.1f}, y={y:.1f}: slowing down locally, retrying at vmax={self.vmax:.2f}')
                else:
                    self.bad_vmax = self.vmax if self.bad_vmax is None else min(self.bad_vmax, self.vmax)
                    if self.good_vmax is not None:
                        self.vmax, self.locked = self.good_vmax, True
                    else:
                        self.vmax = max(1.0, self.vmax - self.vstep)
            self.save()
            if self.locked and self.good_vmax is not None:
                self.log(f'>>> LOCKED at vmax={self.good_vmax:.2f} (best lap {self.best_time:.2f} s)')
        self.reset_stats()

    # ------------------------------------------------------------ every scan message
    def step(self, ranges, amin, inc, x, y, t):
        if self.start is None:
            self.start, self.lap_t0 = (x, y), t
        d0 = math.hypot(x - self.start[0], y - self.start[1])
        if d0 > 15.0:
            self.left = True
        if self.left and d0 < 2.0:
            self.left = False
            self.lap_done(t)
        c = self.side_clearance(ranges, amin, inc)
        if c < self.min_clear:
            self.min_clear, self.worst_xy = c, (x, y)

        r = np.array(ranges, dtype=np.float64)
        r = np.nan_to_num(r, nan=0.0, posinf=self.max_range, neginf=0.0)
        r = np.clip(r, 0.0, self.max_range)
        ang = amin + np.arange(len(r)) * inc
        idx = np.where(np.abs(ang) <= self.fov)[0]
        lo, hi = idx[0], idx[-1]
        raw, ext = r.copy(), r.copy()
        diff = np.diff(raw[lo:hi + 1])
        for k in np.where(np.abs(diff) > self.disp_thresh)[0]:
            i = lo + k
            if raw[i] < raw[i + 1]:
                near = raw[i]
                n = int(math.ceil(math.atan2(self.half_w, max(near, 0.1)) / inc))
                j2 = min(hi + 1, i + 1 + n)
                ext[i + 1:j2] = np.minimum(ext[i + 1:j2], near)
            else:
                near = raw[i + 1]
                n = int(math.ceil(math.atan2(self.half_w, max(near, 0.1)) / inc))
                j1 = max(lo, i + 1 - n)
                ext[j1:i + 1] = np.minimum(ext[j1:i + 1], near)
        w = max(3, int(0.022 / inc))
        seg = np.convolve(ext[lo:hi + 1], np.ones(w) / w, mode='same')
        sa = ang[lo:hi + 1]
        cand = np.where(seg >= 0.9 * seg.max())[0]
        target = sa[cand[np.argmin(np.abs(sa[cand]))]]
        steer = float(np.clip(self.kp * target, -self.max_steer, self.max_steer))
        steer = self.alpha * steer + (1 - self.alpha) * self.prev_steer
        self.prev_steer = steer

        front = np.abs(ang) < math.radians(6)
        d_front = float(raw[front].min()) if front.any() else self.max_range
        vlim = self.vmax * self.zone_factor(x, y)
        v_curve = vlim - (vlim - min(self.vmin, vlim)) * min(1.0, abs(steer) / self.sref)
        v_brake = math.sqrt(2.0 * self.decel * max(d_front - self.dmargin, 0.0))
        v = max(min(self.vmin, vlim), min(v_curve, v_brake, vlim))
        return steer, float(v)


class FastGap(Node):
    def __init__(self):
        super().__init__('fast_gap_controller')
        defaults = dict(vmax=2.0, vmin=1.0, decel=4.0, dmargin=0.8, half_width=0.30,
                        disp_thresh=0.5, max_range=10.0, fov_deg=90.0, kp=1.0, sref=0.25,
                        alpha=0.8, vstep=0.25, vmax_max=6.0, clear_thr=0.17, autotune=True,
                        load=False, freeze=False, skip=5,
                        state_file='fast_gap_state.json', log_file='laps.csv')
        p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.skip, self.cnt, self.pos = int(p['skip']), 0, None
        self.core = FastGapCore(p, log=self.get_logger().info)
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

    def odom_cb(self, m):
        self.pos = (m.pose.pose.position.x, m.pose.pose.position.y)

    def scan_cb(self, msg):
        self.cnt += 1
        if self.pos is None or self.cnt % self.skip:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        steer, speed = self.core.step(msg.ranges, msg.angle_min, msg.angle_increment, *self.pos, t)
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.drive.steering_angle = steer
        out.drive.speed = speed
        self.pub.publish(out)


def main():
    rclpy.init()
    node = FastGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()     dan code vo read me
