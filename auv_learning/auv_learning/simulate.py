"""Synthetic 2D trajectory + IMU generator.

Why synthetic first: perfect ground truth separates 'my pipeline is broken'
from 'my data is bad'. If training fails here, the bug is ours.

Frames (REP-103, flattened): world x east, y north; heading CCW from x.
All SI units.
"""
import numpy as np


def _smooth_noise(n, dt, cutoff_hz, rng):
    """Low-pass-filtered white noise: smooth random curves."""
    x = rng.standard_normal(n)
    alpha = 2 * np.pi * cutoff_hz * dt / (2 * np.pi * cutoff_hz * dt + 1)
    y = np.empty(n)
    y[0] = 0.0
    for k in range(1, n):
        y[k] = y[k - 1] + alpha * (x[k] - y[k - 1])
    return y


def generate_trajectory(duration_s, dt, seed,
                        accel_noise_std=0.05, accel_bias_std=0.05,
                        gyro_noise_std=0.005, gyro_bias_std=0.01):
    """Vehicle on smooth random curves with a noisy, biased IMU.

    Seeded rng: same seed, same dataset, forever (fix seeds wherever
    stochastic components exist). The constant per-run bias is deliberate:
    bias is what naive integration cannot survive and what the learned
    model must absorb. That asymmetry IS the thesis.
    """
    rng = np.random.default_rng(seed)
    n = int(round(duration_s / dt))
    t = np.arange(n) * dt

    # Smooth speed (0.3-1.5 m/s, always forward) and smooth yaw rate.
    speed = 0.9 + 0.6 * np.tanh(_smooth_noise(n, dt, 0.05, rng) * 2.0)
    yaw_rate = 0.4 * _smooth_noise(n, dt, 0.08, rng)

    heading = np.cumsum(yaw_rate) * dt
    vel = np.stack([speed * np.cos(heading), speed * np.sin(heading)], axis=1)
    pos = np.cumsum(vel, axis=0) * dt

    # True world accel by finite difference, rotated into the body frame.
    a_world = np.gradient(vel, dt, axis=0)
    c, s = np.cos(heading), np.sin(heading)
    accel_body = np.stack([ c * a_world[:, 0] + s * a_world[:, 1],
                           -s * a_world[:, 0] + c * a_world[:, 1]], axis=1)

    # IMU corruption: white noise + constant per-run bias.
    accel_meas = (accel_body
                  + rng.standard_normal((n, 2)) * accel_noise_std
                  + rng.standard_normal(2) * accel_bias_std)
    gyro_meas = (yaw_rate
                 + rng.standard_normal(n) * gyro_noise_std
                 + rng.standard_normal() * gyro_bias_std)

    return {'t': t, 'pos': pos, 'vel': vel, 'heading': heading,
            'accel_body': accel_meas, 'gyro_z': gyro_meas}


def sparse_fixes(t, pos, interval_s, noise_std_m, seed):
    """The surface-GPS analogue: noisy position at sparse instants.

    M << N is the entire premise: supervision is occasional and positional,
    never continuous and velocital. That is the gap in the prior work
    (Saksvik, Topini: trained against continuous DVL velocities).
    """
    rng = np.random.default_rng(seed)
    step = max(1, int(round(interval_s / (t[1] - t[0]))))
    idx = np.arange(0, len(t), step)
    fix_pos = pos[idx] + rng.standard_normal((len(idx), 2)) * noise_std_m
    return {'fix_t': t[idx], 'fix_pos': fix_pos, 'fix_idx': idx}