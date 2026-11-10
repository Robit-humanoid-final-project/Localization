"""
RoboCup 휴머노이드용 Monte Carlo Localization (MCL)

입력
  - Prediction : IK Walk 이동량(dx, dy: 로봇 기준) + IMU yaw
  - Update     : Vision의 L/T/X 랜드마크 (type, 로봇 기준 x, y, confidence)
출력
  - 필드 좌표계의 로봇 포즈 (x, y, theta)

좌표계
  - 필드 좌표: 원점=센터, x=필드 긴 방향, y=짧은 방향, 단위 m, 각도 rad
  - 로봇 좌표: x=전방, y=좌측, 단위 m

실행:  python mcl_localization.py        (시뮬레이션 데모)
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

# 랜드마크 type
L_SHAPE, T_SHAPE, X_SHAPE = 0, 1, 2


# ----------------------------------------------------------------------------
# 유틸
# ----------------------------------------------------------------------------
def wrap_angle(a):
    """각도를 [-pi, pi]로 정규화 (스칼라/배열 모두 가능)"""
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


# ----------------------------------------------------------------------------
# 지도: 필드 규격에서 만드는 고정 랜드마크 (오도메트리와 무관한 기준자)
# ----------------------------------------------------------------------------
@dataclass
class FieldConfig:
    # 아래 값은 예시입니다. 실제 대회 규정집의 필드 치수로 바꾸세요.
    length: float = 9.0            # 필드 길이 (x 방향)
    width: float = 6.0             # 필드 폭 (y 방향)
    goal_area_depth: float = 1.0   # 골 에어리어 깊이
    goal_area_width: float = 3.0
    penalty_depth: float = 2.0     # 페널티 에어리어 깊이
    penalty_width: float = 5.0
    circle_radius: float = 0.75    # 센터서클 반지름


class FieldMap:
    def __init__(self, cfg: FieldConfig = FieldConfig()):
        self.cfg = cfg
        self.landmarks = {L_SHAPE: [], T_SHAPE: [], X_SHAPE: []}
        self._build()
        # numpy 배열로 변환 (type -> (M, 2))
        self.pts = {t: np.array(v, dtype=float).reshape(-1, 2)
                    for t, v in self.landmarks.items()}

    def _build(self):
        c = self.cfg
        hx, hy = c.length / 2, c.width / 2

        # 필드 모서리: L
        for sx in (-1, 1):
            for sy in (-1, 1):
                self.landmarks[L_SHAPE].append((sx * hx, sy * hy))

        # 센터라인 ∩ 사이드라인: T
        for sy in (-1, 1):
            self.landmarks[T_SHAPE].append((0.0, sy * hy))

        # 센터라인 ∩ 센터서클: X
        for sy in (-1, 1):
            self.landmarks[X_SHAPE].append((0.0, sy * c.circle_radius))

        # 골/페널티 에어리어: 앞쪽 모서리 = L, 골라인과 만나는 점 = T
        for sx in (-1, 1):
            for depth, w in ((c.goal_area_depth, c.goal_area_width),
                             (c.penalty_depth, c.penalty_width)):
                front_x = sx * (hx - depth)
                for sy in (-1, 1):
                    self.landmarks[L_SHAPE].append((front_x, sy * w / 2))
                    self.landmarks[T_SHAPE].append((sx * hx, sy * w / 2))

    def bounds(self):
        return (-self.cfg.length / 2, self.cfg.length / 2,
                -self.cfg.width / 2, self.cfg.width / 2)


# ----------------------------------------------------------------------------
# 데이터 구조
# ----------------------------------------------------------------------------
@dataclass
class Observation:
    """Vision이 보낸 랜드마크 하나 (로봇 기준, 단위 m)"""
    type: int            # L_SHAPE / T_SHAPE / X_SHAPE
    x: float             # 전방
    y: float             # 좌측
    confidence: float = 1.0


@dataclass
class MCLParams:
    num_particles: int = 1000

    # 모션 노이즈 (오도메트리 모델)
    alpha_trans_trans: float = 0.10   # 이동량 -> 이동 노이즈 (비율)
    alpha_dir_trans: float = 0.05     # 이동량 -> 방향 노이즈 (rad/m)
    alpha_yaw: float = 0.05           # yaw 변화량 -> yaw 노이즈
    min_xy_noise: float = 0.005       # 정지 시에도 남기는 최소 노이즈 (m)
    min_yaw_noise: float = 0.002      # (rad)

    # 측정 모델
    sigma_base: float = 0.10          # 관측 위치 기본 표준편차 (m)
    sigma_per_meter: float = 0.05     # 거리에 비례해 커지는 표준편차
    gate_dist: float = 1.5            # 지도 랜드마크와 이 거리 넘으면 매칭 실패
    outlier_floor: float = 1e-3       # 매칭 실패/오검출 때 주는 바닥 likelihood
    min_confidence: float = 0.3       # 이보다 낮은 관측은 무시

    # 리샘플링
    resample_neff_ratio: float = 0.5  # Neff < N * ratio 일 때 리샘플링
    resample_noise_xy: float = 0.02   # 리샘플링 후 추가 노이즈 (m)
    resample_noise_yaw: float = 0.01  # (rad)
    random_inject_ratio: float = 0.0  # 복구용 랜덤 파티클 비율 (0이면 끔)

    # 갱신 조건
    min_obs_for_update: int = 1


# ----------------------------------------------------------------------------
# MCL 본체
# ----------------------------------------------------------------------------
class MonteCarloLocalization:
    def __init__(self, field_map: FieldMap, params: MCLParams = MCLParams(),
                 seed: Optional[int] = None):
        self.map = field_map
        self.p = params
        self.rng = np.random.default_rng(seed)
        n = params.num_particles
        self.x = np.zeros(n)
        self.y = np.zeros(n)
        self.th = np.zeros(n)
        self.w = np.ones(n) / n
        self.pose = np.zeros(3)         # 현재 추정 포즈
        self.initialized = False

    # ---------------- [0] 초기화 ----------------
    def init_gaussian(self, pose: Tuple[float, float, float],
                      std: Tuple[float, float, float] = (0.3, 0.3, 0.2)):
        """초기 포즈를 대략 아는 경우: 필드 좌표 기준 가우시안"""
        n = self.p.num_particles
        self.x = self.rng.normal(pose[0], std[0], n)
        self.y = self.rng.normal(pose[1], std[1], n)
        self.th = wrap_angle(self.rng.normal(pose[2], std[2], n))
        self._reset_weights()
        self.pose = np.array(pose, dtype=float)
        self.initialized = True

    def init_uniform(self, region: Optional[Tuple[float, float, float, float]] = None,
                     theta_range: Tuple[float, float] = (-np.pi, np.pi)):
        """위치를 모르는 경우: 영역(기본: 필드 전체) 균일 분포"""
        xmin, xmax, ymin, ymax = region or self.map.bounds()
        n = self.p.num_particles
        self.x = self.rng.uniform(xmin, xmax, n)
        self.y = self.rng.uniform(ymin, ymax, n)
        self.th = wrap_angle(self.rng.uniform(theta_range[0], theta_range[1], n))
        self._reset_weights()
        self.pose = self._estimate()
        self.initialized = True

    def init_sideline_start(self, side: int = -1, x_range=(-3.0, -1.0),
                            y_offset: float = 0.0, std_y: float = 0.3,
                            std_th: float = 0.2):
        """
        대회 규칙 기반 초기화: 사이드라인 바깥에서 필드 안쪽을 보고 시작.
        side=-1: y<0 쪽 사이드라인(위쪽을 바라봄), side=+1: y>0 쪽(아래쪽을 바라봄)
        """
        n = self.p.num_particles
        hy = self.map.cfg.width / 2
        self.x = self.rng.uniform(x_range[0], x_range[1], n)
        self.y = self.rng.normal(side * (hy + y_offset), std_y, n)
        facing = np.pi / 2 if side < 0 else -np.pi / 2
        self.th = wrap_angle(self.rng.normal(facing, std_th, n))
        self._reset_weights()
        self.pose = self._estimate()
        self.initialized = True

    def _reset_weights(self):
        self.w = np.ones(self.p.num_particles) / self.p.num_particles

    # ---------------- [1] Prediction ----------------
    def predict(self, dx: float, dy: float, yaw: Optional[float] = None,
                d_yaw: Optional[float] = None):
        """
        dx, dy : IK Walk 이동량 (로봇 기준, m). 이번 사이클에서 움직인 '변화량'
        yaw    : IMU yaw(rad, 필드 기준으로 맞춰진 값) -> 직전 값과의 차이로 변화량 계산
        d_yaw  : yaw 변화량을 직접 줄 때 (yaw와 둘 중 하나만)
        파티클마다 독립 노이즈를 샘플링해서 이동 (불확실성 증가)
        """
        n = self.p.num_particles
        if d_yaw is None:
            d_yaw = 0.0 if yaw is None else self._yaw_delta(yaw)

        trans = float(np.hypot(dx, dy))
        local_dir = float(np.arctan2(dy, dx)) if trans > 1e-9 else 0.0

        # 노이즈 크기 (이동량에 비례, 정지해도 최소값 유지)
        trans_std = self.p.alpha_trans_trans * trans + self.p.min_xy_noise
        dir_std = self.p.alpha_dir_trans * trans + self.p.min_yaw_noise
        yaw_std = self.p.alpha_yaw * abs(d_yaw) + self.p.min_yaw_noise

        n_trans = trans + self.rng.normal(0.0, trans_std, n)
        n_dir = local_dir + self.rng.normal(0.0, dir_std, n)

        # 로봇 기준 이동 방향을 각 파티클의 yaw로 회전해서 필드 좌표 이동량으로
        world_dir = self.th + n_dir
        self.x += n_trans * np.cos(world_dir)
        self.y += n_trans * np.sin(world_dir)
        self.th = wrap_angle(self.th + d_yaw + self.rng.normal(0.0, yaw_std, n))

        # 필드 밖으로 못 나가게 (여유 0.5 m)
        xmin, xmax, ymin, ymax = self.map.bounds()
        self.x = np.clip(self.x, xmin - 0.5, xmax + 0.5)
        self.y = np.clip(self.y, ymin - 0.5, ymax + 0.5)

        self.pose = self._estimate()

    _last_yaw: Optional[float] = None

    def _yaw_delta(self, yaw: float) -> float:
        if self._last_yaw is None:
            self._last_yaw = yaw
            return 0.0
        d = float(wrap_angle(yaw - self._last_yaw))
        self._last_yaw = yaw
        return d

    # ---------------- [2] Update (가중치) ----------------
    def update(self, observations: List[Observation]) -> bool:
        """
        LTX 관측으로 파티클 가중치를 계산하고, 필요하면 리샘플링.
        반환: 실제로 Update를 수행했는지 여부
        """
        obs = [o for o in observations if o.confidence >= self.p.min_confidence
               and self.map.pts[o.type].shape[0] > 0]
        if len(obs) < self.p.min_obs_for_update:
            return False   # 관측이 없으면 Prediction만 (파티클 유지)

        log_w = np.zeros(self.p.num_particles)
        c, s = np.cos(self.th), np.sin(self.th)

        for o in obs:
            # 파티클이 맞다면 이 랜드마크는 필드의 (wx, wy)에 있어야 함
            wx = self.x + o.x * c - o.y * s
            wy = self.y + o.x * s + o.y * c

            # 같은 type 지도 랜드마크 중 가장 가까운 것 (nearest neighbor)
            lm = self.map.pts[o.type]                       # (M, 2)
            dxm = wx[:, None] - lm[None, :, 0]              # (N, M)
            dym = wy[:, None] - lm[None, :, 1]
            dist = np.sqrt(dxm ** 2 + dym ** 2).min(axis=1) # (N,)

            # 거리에 비례한 표준편차 (먼 관측일수록 관대하게)
            rng_m = np.hypot(o.x, o.y)
            sigma = self.p.sigma_base + self.p.sigma_per_meter * rng_m

            lik = np.exp(-0.5 * (dist / sigma) ** 2)
            lik = np.where(dist > self.p.gate_dist, 0.0, lik)   # 게이트 밖은 매칭 실패

            # 오검출에 파티클이 전멸하지 않도록 바닥값을 두고 confidence로 섞음
            lik = o.confidence * lik + (1.0 - o.confidence) * self.p.outlier_floor
            log_w += np.log(lik + self.p.outlier_floor)

        # 수치 안정성을 위해 log-sum-exp
        log_w -= log_w.max()
        w = self.w * np.exp(log_w)
        total = w.sum()
        if not np.isfinite(total) or total <= 0:
            self._reset_weights()        # 전멸 시 가중치 초기화 (파티클은 유지)
            return False
        self.w = w / total

        self._maybe_resample()
        self.pose = self._estimate()
        return True

    # ---------------- [3] Resampling ----------------
    def neff(self) -> float:
        """유효 파티클 수. 작을수록 가중치가 소수 파티클에 쏠려 있음"""
        return 1.0 / np.sum(self.w ** 2)

    def _maybe_resample(self):
        n = self.p.num_particles
        if self.neff() < n * self.p.resample_neff_ratio:
            self.resample()

    def resample(self):
        """Systematic (low-variance) resampling + 소량 노이즈 + 선택적 랜덤 주입"""
        n = self.p.num_particles
        positions = (self.rng.random() + np.arange(n)) / n
        cumulative = np.cumsum(self.w)
        cumulative[-1] = 1.0
        idx = np.searchsorted(cumulative, positions)

        self.x = self.x[idx] + self.rng.normal(0, self.p.resample_noise_xy, n)
        self.y = self.y[idx] + self.rng.normal(0, self.p.resample_noise_xy, n)
        self.th = wrap_angle(self.th[idx] + self.rng.normal(0, self.p.resample_noise_yaw, n))

        # 복구용: 일부를 필드 전체에 랜덤 배치 (kidnapped 대비)
        k = int(n * self.p.random_inject_ratio)
        if k > 0:
            xmin, xmax, ymin, ymax = self.map.bounds()
            sel = self.rng.choice(n, k, replace=False)
            self.x[sel] = self.rng.uniform(xmin, xmax, k)
            self.y[sel] = self.rng.uniform(ymin, ymax, k)
            self.th[sel] = self.rng.uniform(-np.pi, np.pi, k)

        self._reset_weights()

    # ---------------- [4] 포즈 추정 ----------------
    def _estimate(self) -> np.ndarray:
        """가중 평균. yaw는 원형 평균(circular mean)"""
        x = np.sum(self.w * self.x)
        y = np.sum(self.w * self.y)
        th = np.arctan2(np.sum(self.w * np.sin(self.th)),
                        np.sum(self.w * np.cos(self.th)))
        return np.array([x, y, th])

    def spread(self) -> Tuple[float, float]:
        """파티클 퍼짐 (위치 표준편차 m, yaw 표준편차 rad). 수렴 판단용"""
        mean = self._estimate()
        var_xy = np.sum(self.w * ((self.x - mean[0]) ** 2 + (self.y - mean[1]) ** 2))
        dth = wrap_angle(self.th - mean[2])
        var_th = np.sum(self.w * dth ** 2)
        return float(np.sqrt(var_xy)), float(np.sqrt(var_th))

    # ---------------- 한 사이클 편의 함수 ----------------
    def step(self, dx: float, dy: float, yaw: Optional[float],
             observations: Optional[List[Observation]] = None) -> np.ndarray:
        """Prediction -> (관측 있으면) Update -> 포즈 반환"""
        self.predict(dx, dy, yaw=yaw)
        if observations:
            self.update(observations)
        return self.pose.copy()


# ----------------------------------------------------------------------------
# 시뮬레이션 데모: 가짜 로봇을 필드 안에서 걷게 하고 MCL이 따라가는지 확인
# ----------------------------------------------------------------------------
def make_trajectory(steps: int, start=(-3.0, -2.0, 0.0), v: float = 0.05):
    """필드 안에서 벽에 가까워지면 중앙을 향해 도는 정답 경로 생성"""
    x, y, th = start
    traj = []
    for _ in range(steps):
        # 벽에 가까우면 중앙 쪽으로 회전, 아니면 완만하게 곡선 주행
        if abs(x) > 3.6 or abs(y) > 2.2:
            to_center = np.arctan2(-y, -x)
            d_yaw = float(np.clip(wrap_angle(to_center - th), -0.08, 0.08))
        else:
            d_yaw = 0.015
        th = float(wrap_angle(th + d_yaw))
        x += v * np.cos(th)
        y += v * np.sin(th)
        traj.append((x, y, th, v, d_yaw))
    return traj


def simulate(steps: int = 400, seed: int = 1, verbose: bool = True,
             odom_scale: float = 0.93):
    rng = np.random.default_rng(seed)
    fmap = FieldMap()
    mcl = MonteCarloLocalization(fmap, MCLParams(num_particles=800), seed=seed)
    traj = make_trajectory(steps)

    start = (-3.0, -2.0, 0.0)
    # 초기 위치를 약간 틀리게 알려줌 (현실에서의 초기 포즈 오차)
    mcl.init_gaussian((start[0] + 0.2, start[1] - 0.15, start[2] + 0.1),
                      std=(0.4, 0.4, 0.2))

    max_view, fov = 3.5, np.deg2rad(70)   # 카메라 최대 거리, 시야각
    errs = []
    for k, (tx, ty, tth, v, _) in enumerate(traj):
        # 오도메트리: IK는 실제 이동량의 odom_scale 배로 잘못 보고, IMU yaw는 노이즈
        ik_dx = v * odom_scale + rng.normal(0, 0.003)
        ik_dy = rng.normal(0, 0.002)
        imu_yaw = tth + rng.normal(0, 0.005)

        # 관측: 시야 안의 랜드마크를 로봇 좌표로 변환하고 노이즈 추가
        observations = []
        c, s = np.cos(tth), np.sin(tth)
        for t, pts in fmap.pts.items():
            for lx, ly in pts:
                rx, ry = lx - tx, ly - ty
                fx, fy = rx * c + ry * s, -rx * s + ry * c
                dist = np.hypot(fx, fy)
                if fx > 0 and dist < max_view and abs(np.arctan2(fy, fx)) < fov / 2:
                    n = 0.03 + 0.02 * dist
                    observations.append(Observation(
                        t, fx + rng.normal(0, n), fy + rng.normal(0, n),
                        confidence=float(rng.uniform(0.7, 1.0))))
        # 가끔 오검출(없는 곳의 랜드마크) 섞기
        if rng.random() < 0.1:
            observations.append(Observation(
                int(rng.integers(0, 3)), rng.uniform(0.5, 3), rng.uniform(-1, 1), 0.5))

        est = mcl.step(ik_dx, ik_dy, imu_yaw, observations)
        errs.append(np.hypot(est[0] - tx, est[1] - ty))

        if verbose and (k % 50 == 0 or k == steps - 1):
            sp_xy, _ = mcl.spread()
            print(f"step {k:3d} | obs {len(observations):2d} | "
                  f"true=({tx:+.2f},{ty:+.2f}) est=({est[0]:+.2f},{est[1]:+.2f}) | "
                  f"err={errs[-1]:.3f} m | spread={sp_xy:.3f} m | Neff={mcl.neff():.0f}")

    errs = np.array(errs)
    if verbose:
        print(f"\n평균 위치 오차: {errs.mean():.3f} m, 마지막 100스텝 평균: {errs[-100:].mean():.3f} m")
    return errs


def simulate_odom_only(steps: int = 400, seed: int = 1, odom_scale: float = 0.93):
    """비교용: LTX 보정 없이 오도메트리(Prediction)만 쓸 때의 drift"""
    rng = np.random.default_rng(seed)
    traj = make_trajectory(steps)
    ex, ey = -3.0 + 0.2, -2.0 - 0.15
    errs = []
    for tx, ty, tth, v, _ in traj:
        yaw = tth + rng.normal(0, 0.005)
        ex += v * odom_scale * np.cos(yaw)
        ey += v * odom_scale * np.sin(yaw)
        errs.append(np.hypot(ex - tx, ey - ty))
    return np.array(errs)


if __name__ == "__main__":
    print("=== MCL (Prediction + LTX Update) ===")
    e_mcl = simulate()
    e_odo = simulate_odom_only()
    print(f"\n[비교] 오도메트리만 사용: 평균 {e_odo.mean():.3f} m, "
          f"마지막 100스텝 평균 {e_odo[-100:].mean():.3f} m")