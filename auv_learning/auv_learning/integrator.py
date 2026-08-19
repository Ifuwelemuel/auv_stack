"""Differentiable dead reckoning. THE core artefact of the dissertation.

The contribution rests on one property: gradients flow from final positions
back to every IMU sample. Hence torch end-to-end and three bans inside this
module: no .detach(), no .item(), no numpy. Any of them severs the autograd
graph SILENTLY -- training runs, loss sits still, and nothing says why.
"""
import torch


def integrate(accel_body, gyro_z, dt, x0):
    """Dead-reckon body-frame IMU to world-frame positions.

    accel_body: (N,2) tensor, body-frame acceleration [m/s^2]
    gyro_z:     (N,)  tensor, yaw rate [rad/s]
    dt:         float, sample period [s]
    x0:         (5,)  tensor, [px, py, vx, vy, heading] initial state
    returns:    (N,2) tensor, world positions

    Numerics: rectangle-rule Euler via cumsum. Chosen because cumsum is one
    vectorised autograd op -- fast forward AND backward -- and at IMU rates
    (dt ~ 0.01 s) Euler's bias sits far below sensor noise. We state the
    choice honestly in the methodology rather than gold-plating it.
    """
    # Heading from integrated yaw rate.
    heading = x0[4] + torch.cumsum(gyro_z, dim=0) * dt          # (N,)

    # Rotate body accel to world: a_w = R(heading) @ a_b, component-wise
    # to stay vectorised (no per-step matmul).
    c, s = torch.cos(heading), torch.sin(heading)
    ax_w = c * accel_body[:, 0] - s * accel_body[:, 1]
    ay_w = s * accel_body[:, 0] + c * accel_body[:, 1]
    a_world = torch.stack([ax_w, ay_w], dim=1)                   # (N,2)

    # Integrate accel -> velocity -> position.
    vel = x0[2:4] + torch.cumsum(a_world, dim=0) * dt            # (N,2)
    pos = x0[0:2] + torch.cumsum(vel, dim=0) * dt                # (N,2)
    return pos