# DOGS2 v1 software known issues

These notes track firmware behavior that still needs adjustment in the current
`v1` engineering sample.

1. Output-voltage convergence should be more aggressive when the error is large. The current control loop reaches the setpoint too slowly, and it likely needs a first-boot no-load calibration path.
2. Current measurement and UI display accuracy still need tuning. Averaging modes should be rechecked to find a better balance between update speed and reading stability.
3. The least-significant-digit display hysteresis still needs validation.
