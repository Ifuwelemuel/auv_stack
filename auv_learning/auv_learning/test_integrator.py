"""Tests against ANALYTIC ground truth -- tests that catch real failure
modes, no coverage theatre. The gradient test is the dissertation's
central claim in executable form."""
import torch

from auv_learning.integrator import integrate
from auv_learning.losses import displacement_loss
from auv_learning.simulate import generate_trajectory, sparse_fixes

DT = 0.01
X0 = torch.zeros(5)


def test_zero_input_holds_position():
    """No motion in, no motion out."""
    n = 500
    pos = integrate(torch.zeros(n, 2), torch.zeros(n), DT, X0)
    assert torch.allclose(pos, torch.zeros(n, 2), atol=1e-9)


def test_constant_accel_half_a_t_squared():
    """Constant forward accel at heading 0 reproduces 0.5*a*t^2.
    Tolerance reflects rectangle-rule error, so this test also documents
    the integrator's error order."""
    n, a = 1000, 0.5
    accel = torch.zeros(n, 2)
    accel[:, 0] = a
    pos = integrate(accel, torch.zeros(n), DT, X0)
    T = n * DT
    assert abs(pos[-1, 0].item() - 0.5 * a * T ** 2) < a * T * DT * 2
    assert abs(pos[-1, 1].item()) < 1e-6


def test_constant_gyro_curves_the_path():
    """Constant turn rate + forward thrust must curve the path: y accumulates,
    unlike the straight-line case."""
    n, w = 1000, 0.3
    accel = torch.zeros(n, 2)
    accel[:, 0] = 1.0
    pos = integrate(accel, torch.full((n,), w), DT, X0)
    assert pos[-1, 1].abs().item() > 0.1


def test_gradients_flow_end_to_end():
    """THE claim: d(loss)/d(IMU sample) exists, is finite, is nonzero --
    sparse position fixes can supervise the entire integration chain."""
    data = generate_trajectory(30.0, DT, seed=42)
    fixes = sparse_fixes(data['t'], data['pos'], 5.0, 0.5, seed=43)
    accel = torch.tensor(data['accel_body'], dtype=torch.float32,
                         requires_grad=True)
    gyro = torch.tensor(data['gyro_z'], dtype=torch.float32,
                        requires_grad=True)
    traj = integrate(accel, gyro, DT, X0)
    loss = displacement_loss(
        traj,
        torch.tensor(data['t'], dtype=torch.float32),
        torch.tensor(fixes['fix_t'], dtype=torch.float32),
        torch.tensor(fixes['fix_pos'], dtype=torch.float32))
    loss.backward()
    for g in (accel.grad, gyro.grad):
        assert g is not None
        assert torch.isfinite(g).all()
        assert g.abs().sum() > 0