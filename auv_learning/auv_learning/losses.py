"""GPS-only supervision: loss on displacement between sparse fixes."""
import torch


def displacement_loss(traj, t, fix_t, fix_pos):
    """MSE between predicted and measured displacement fix[i] -> fix[i+1].

    Differences, not absolute positions, for two reasons (both go in the
    dissertation): (1) a GPS fix anchors WHERE you are only at instants;
    what dead reckoning must get right is how far and which way you moved
    between them. (2) Differencing cancels any constant offset (datum
    error, initial-position error), so the loss grades motion, not map
    alignment.

    Gather-indexing is differentiable w.r.t. traj values, so gradients
    flow from this loss through integrate() to the raw IMU stream.
    Assumes fix_t are members of t (true by construction in sparse_fixes).
    """
    idx = torch.searchsorted(t, fix_t)
    pred_at_fix = traj[idx]                        # (M,2)
    pred_disp = pred_at_fix[1:] - pred_at_fix[:-1]
    meas_disp = fix_pos[1:] - fix_pos[:-1]
    return torch.mean((pred_disp - meas_disp) ** 2)