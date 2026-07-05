"""BRL-PPC 控制器。

从原始单文件实验脚本中提取控制算法部分。
机器人通信、实时循环和数据保存由平台统一处理。
"""

import math
from collections import deque

import numpy as np

from .base import BaseController, ControlResult


def rbf(x, centers, width, count):
    diff = x[:, np.newaxis] - centers[:, :count]
    dist_sq = np.sum(diff**2, axis=0)
    return np.exp(-dist_sq / (width**2))


class BRLPPCController(BaseController):
    """BRL-PPC 自适应力矩控制器。"""

    name = "brl_ppc"

    def __init__(self, dt=0.001, torque_limit=50.0, seed=None):
        self.dt = float(dt)
        self.torque_limit = float(torque_limit)
        self.seed = seed
        self.rng = np.random.default_rng(seed)

        self.k1 = np.diag([5.0, 5.0])
        self.k2 = np.diag([10.0, 10.0])

        self.t_transition = 1.5
        self.mu_h = 0.3
        self.rho_0 = np.array([0.8, 0.8])
        self.rho_inf = np.array([0.05, 0.05])
        self.l_i = np.array([1.2, 1.2])
        self.delta_u = 0.95
        self.delta_l = 0.95

        self.la = 2.0
        self.sigma_a = 0.01
        self.b_a = 15.0
        self.na_initial = 500
        self.na_max = 3000
        self.input_dim_a = 8

        self.bls_n_closest = 4
        self.bls_threshold_rho = 0.5
        self.bls_beta_gamma = 0.3

        self.nc = 350
        self.lc = 5.0
        self.sigma_c = 0.01
        self.b_c = 3.0
        self.input_dim_c = 6

        self.tc = 0.01
        self.vartheta = 0.9
        self.cp = 0.1
        self.eps = 1e-5

        self.na_current = self.na_initial
        self.mu_a = None
        self.wa_hat = None
        self.mu_c = None
        self.wc_hat = None
        self.q_buffer = None
        self.wc_buffer = None
        self.e1_0_sign = None
        self.last_xr = None
        self.reference = None

    def set_reference(self, reference):
        self.reference = reference

    def reset(self, q0, dq0=None):
        if self.reference is not None:
            xr0, _, _ = self.reference.sample(0.0)
        else:
            xr0 = np.zeros_like(q0, dtype=float)
        self.e1_0_sign = np.sign(np.asarray(q0, dtype=float) - xr0)
        self.e1_0_sign[self.e1_0_sign == 0.0] = 1.0

        self.na_current = self.na_initial
        self.mu_a = (self.rng.random((self.input_dim_a, self.na_max)) - 0.5) * 20.0
        self.wa_hat = np.zeros((self.na_max, 2))

        self.mu_c = (self.rng.random((self.input_dim_c, self.nc)) - 0.5) * 20.0
        self.wc_hat = np.zeros((self.nc, 2))

        lag_steps = round(self.tc / self.dt)
        buffer_size = lag_steps + 2
        self.q_buffer = deque(maxlen=buffer_size)
        self.wc_buffer = deque(maxlen=buffer_size)
        for _ in range(buffer_size):
            self.q_buffer.append(np.asarray(q0, dtype=float).copy())
            self.wc_buffer.append(self.wc_hat.copy())

    def get_params(self):
        return {
            "dt": self.dt,
            "torque_limit": self.torque_limit,
            "seed": -1 if self.seed is None else self.seed,
            "k1": np.diag(self.k1),
            "k2": np.diag(self.k2),
            "t_transition": self.t_transition,
            "mu_h": self.mu_h,
            "rho_0": self.rho_0,
            "rho_inf": self.rho_inf,
            "l_i": self.l_i,
            "delta_u": self.delta_u,
            "delta_l": self.delta_l,
            "la": self.la,
            "sigma_a": self.sigma_a,
            "b_a": self.b_a,
            "na_initial": self.na_initial,
            "na_max": self.na_max,
            "input_dim_a": self.input_dim_a,
            "bls_n_closest": self.bls_n_closest,
            "bls_threshold_rho": self.bls_threshold_rho,
            "bls_beta_gamma": self.bls_beta_gamma,
            "nc": self.nc,
            "lc": self.lc,
            "sigma_c": self.sigma_c,
            "b_c": self.b_c,
            "input_dim_c": self.input_dim_c,
            "tc": self.tc,
            "vartheta": self.vartheta,
            "cp": self.cp,
            "eps": self.eps,
        }

    def compute(self, t, q, dq, xr, dxr, ddxr):
        self.last_xr = xr.copy()
        ppc = self._compute_ppc(t, q, dq, xr, dxr, ddxr)
        zeta = ppc["zeta"]
        alpha1 = ppc["alpha1"]
        upsilon = ppc["Upsilon"]
        w_val = ppc["w"]

        xa = np.concatenate([q, dq, dxr, ddxr])
        self._bls_expand(xa)
        sa = rbf(xa, self.mu_a, self.b_a, self.na_current)
        wa_active = self.wa_hat[: self.na_current, :]

        xc = np.concatenate([q, zeta, xr])
        sc = rbf(xc, self.mu_c, self.b_c, self.nc)

        t_lag = max(t - self.tc, 0.0)
        q_lag = self.q_buffer[0]
        wc_lag = self.wc_buffer[0]
        zeta_lag, xr_lag = self._zeta_lag(t_lag, q_lag, xr)
        xc_lag = np.concatenate([q_lag, zeta_lag, xr_lag])
        sc_lag = rbf(xc_lag, self.mu_c, self.b_c, self.nc)

        h_hat = self.wc_hat.T @ sc
        h_hat_lag = wc_lag.T @ sc_lag
        pc = np.zeros(2)
        cost_val = (self.tc / math.log(self.vartheta)) * (self.vartheta - 1.0)
        pc[np.abs(zeta) > self.cp] = cost_val
        ec = h_hat - self.vartheta * h_hat_lag + pc
        delta_sc = sc - self.vartheta * sc_lag
        dwc = -self.lc * (np.outer(delta_sc, ec) + self.sigma_c * self.wc_hat)

        e2 = dq - alpha1
        ea = e2 + h_hat
        dwa = -self.la * (np.outer(sa, ea) + self.sigma_a * wa_active)

        u_actor = wa_active.T @ sa
        torque = u_actor - w_val * upsilon @ zeta - self.k2 @ e2
        torque = np.clip(torque, -self.torque_limit, self.torque_limit)

        self.wc_hat += dwc * self.dt
        self.wa_hat[: self.na_current, :] += dwa * self.dt
        self.q_buffer.append(q.copy())
        self.wc_buffer.append(self.wc_hat.copy())

        return ControlResult(
            torque=torque,
            log={
                "error": ppc["e1"],
                "zeta": zeta,
                "alpha1": alpha1,
                "u_actor": u_actor,
                "e_bar": ppc["e_bar"],
                "e_underline": ppc["e_underline"],
                "na": np.array([self.na_current], dtype=float),
                "ppc_violated": np.array([float(ppc["violated"])]),
            },
        )

    def _compute_ppc(self, t, q, dq, xr, dxr, ddxr):
        e1 = q - xr
        sign = self.e1_0_sign
        eps = self.eps

        rho = (self.rho_0 - self.rho_inf) * np.exp(-self.l_i * t) + self.rho_inf
        d_rho = -self.l_i * (self.rho_0 - self.rho_inf) * np.exp(-self.l_i * t)

        e_bar = (self.delta_u + sign) * rho - sign * self.rho_inf
        d_e_bar = (self.delta_u + sign) * d_rho
        uw = (self.delta_l - sign) * rho + sign * self.rho_inf
        d_uw = (self.delta_l - sign) * d_rho
        e_underline = -uw
        d_e_underline = -d_uw

        if t < self.t_transition:
            w = math.sin(math.pi * t / (2.0 * self.t_transition))
            dw = (math.pi / (2.0 * self.t_transition)) * math.cos(
                math.pi * t / (2.0 * self.t_transition)
            )
            h_i_0 = (e_bar[0] - uw[0]) / 2.0
            denom = self.t_transition - t + eps
            h_val = h_i_0 * math.exp(-self.mu_h * self.t_transition * t / denom)
            h = np.array([h_val, h_val])
            dh = h * (-self.mu_h * self.t_transition**2 / (denom**2))
        else:
            w = 1.0
            dw = 0.0
            h = np.zeros(2)
            dh = np.zeros(2)

        epsilon = w * e1 + h
        margin = 1e-4
        violated = bool(
            np.any(epsilon <= e_underline + margin)
            or np.any(epsilon >= e_bar - margin)
        )
        epsilon = np.clip(epsilon, e_underline + margin, e_bar - margin)

        log_num = np.maximum(uw + epsilon, eps)
        log_denom = np.maximum(e_bar - epsilon, eps)
        zeta = np.log(log_num / log_denom)

        denom_r = (uw + epsilon) * (e_bar - epsilon) + eps
        r_i = (e_bar + uw) / denom_r
        upsilon = np.diag(r_i)
        inv_upsilon = np.diag(1.0 / (r_i + eps))

        c_i = d_e_underline / (e_underline + epsilon - eps) - d_e_bar / (
            e_bar - epsilon + eps
        )

        if t < 1e-6:
            alpha1 = dxr.copy()
        else:
            alpha1 = -inv_upsilon @ self.k1 @ (zeta / (w + eps)) + dxr

        d_epsilon = dw * e1 + w * (dq - dxr) + dh
        d_zeta = upsilon @ d_epsilon + c_i
        d_r_i = -(r_i**2) * (
            (-d_uw - d_epsilon) / (uw + epsilon + eps)
            - (d_e_bar - d_epsilon) / (e_bar - epsilon + eps)
        )
        d_upsilon = np.diag(d_r_i)
        d_zeta_over_w = (d_zeta * w - zeta * dw) / (w**2 + eps)

        term1 = inv_upsilon @ d_upsilon @ inv_upsilon @ self.k1 @ (zeta / (w + eps))
        term2 = -inv_upsilon @ self.k1 @ d_zeta_over_w
        d_alpha1 = term1 + term2 + ddxr

        return {
            "e1": e1,
            "zeta": zeta,
            "e_bar": e_bar,
            "e_underline": e_underline,
            "Upsilon": upsilon,
            "w": w,
            "alpha1": alpha1,
            "d_alpha1": d_alpha1,
            "violated": violated,
        }

    def _bls_expand(self, xa):
        if self.na_current >= self.na_max:
            return
        dist = np.linalg.norm(xa[:, np.newaxis] - self.mu_a[:, : self.na_current], axis=0)
        idx = np.argsort(dist)[: min(self.na_current, self.bls_n_closest)]
        q_center = np.mean(self.mu_a[:, idx], axis=1)
        chi = np.linalg.norm(xa - q_center)
        if chi >= self.bls_threshold_rho:
            self.mu_a[:, self.na_current] = q_center + self.bls_beta_gamma * (
                xa - q_center
            )
            self.na_current += 1

    def _zeta_lag(self, t_lag, q_lag, current_xr):
        sign = self.e1_0_sign
        if self.reference is not None:
            xr_lag, _, _ = self.reference.sample(t_lag)
        else:
            xr_lag = current_xr.copy()
        e1_lag = q_lag - xr_lag

        rho_lag = (self.rho_0 - self.rho_inf) * np.exp(-self.l_i * t_lag) + self.rho_inf
        e_bar_lag = (self.delta_u + sign) * rho_lag - sign * self.rho_inf
        uw_lag = (self.delta_l - sign) * rho_lag + sign * self.rho_inf

        if t_lag < self.t_transition:
            w_lag = math.sin(math.pi * t_lag / (2.0 * self.t_transition))
            h_i_0 = (e_bar_lag[0] - uw_lag[0]) / 2.0
            denom = self.t_transition - t_lag + self.eps
            h_val = h_i_0 * math.exp(-self.mu_h * self.t_transition * t_lag / denom)
            h_lag = np.array([h_val, h_val])
        else:
            w_lag = 1.0
            h_lag = np.zeros(2)

        eps_lag = w_lag * e1_lag + h_lag
        margin = 1e-4
        eps_lag = np.clip(eps_lag, -uw_lag + margin, e_bar_lag - margin)

        log_num = np.maximum(uw_lag + eps_lag, self.eps)
        log_denom = np.maximum(e_bar_lag - eps_lag, self.eps)
        return np.log(log_num / log_denom), xr_lag
