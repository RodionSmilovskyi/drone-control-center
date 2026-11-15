import unittest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../services/drone-inference')))

from flight_controller import FlightController
from inference import handle_armed, handle_disarmed, handle_ai, MAX_ALTITUDE, MAX_XY_SHIFT


class TestInferenceAltitude(unittest.TestCase):
    def setUp(self):
        self.fc = FlightController()

    def test_mode_aux_channels_for_inav(self):
        obs = np.zeros(9, dtype=np.float64)
        self.assertEqual(handle_disarmed(obs, self.fc), [1500, 1500, 900, 1500, 1000, 1000])
        self.assertEqual(handle_armed(obs, self.fc), [1500, 1500, 900, 1500, 1800, 1000])

    def test_vertical_shift_relative_to_reference_math(self):
        # Starting on ground (ref_alt = 0.0m), desired_shift_z = +0.4 / 3.0
        obs_ground = np.array([0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0], dtype=np.float64)
        shift_z = 0.4 / MAX_ALTITUDE
        rc_climb = handle_ai(obs_ground, self.fc, dt=0.033, desired_shift_z=shift_z, desired_shift_x=0.0, desired_shift_y=0.0)
        self.assertAlmostEqual(self.fc.ref_alt_norm, 0.0, places=5)
        self.assertAlmostEqual(self.fc.ref_alt_norm + shift_z, 0.4 / MAX_ALTITUDE, places=5)
        self.assertGreater(rc_climb[2], int(self.fc.hover_throttle + self.fc.inav_thr_deadband))

        # Starting mid-air (ref_alt = 1.2m), negative desired_shift_z = -0.3 / 3.0 (-0.1) -> target 0.9m
        self.fc.reset()
        obs_midair = np.array([1.2, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0], dtype=np.float64)
        neg_shift_z = -0.3 / MAX_ALTITUDE
        for _ in range(10):
            rc = handle_ai(obs_midair, self.fc, dt=0.033, desired_shift_z=neg_shift_z, desired_shift_x=0.0, desired_shift_y=0.0)
        self.assertAlmostEqual(self.fc.ref_alt_norm, 1.2 / MAX_ALTITUDE, places=5)
        self.assertAlmostEqual(self.fc.throttle_pid.setpoint, 0.9 / MAX_ALTITUDE, places=4)
        self.assertLess(rc[2], int(self.fc.hover_throttle - self.fc.inav_thr_deadband))

    def test_handle_ai_steady_state_at_setpoint(self):
        # Simulate 9-float sensor obs at 0.4m within arrival tolerance -> 1500 PWM hold across all axes
        obs = np.array([0.4, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0, 3.0, -2.0], dtype=np.float64)

        handle_ai(obs, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)
        rc = handle_ai(obs, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)

        self.assertEqual(len(rc), 6)
        roll, pitch, throttle, yaw, aux1, aux2 = rc
        self.assertEqual(roll, 1500)
        self.assertEqual(pitch, 1500)
        self.assertEqual(throttle, 1500)
        self.assertEqual(yaw, 1500)
        self.assertEqual(aux1, 1800)
        self.assertEqual(aux2, 1800)

        # Still within arrival tolerances (<= 3cm Z and <= 3cm X/Y) -> holds at 1500 PWM
        obs_near = np.array([0.42, 1.5, 0.02, -0.02, 0.0, 0.0, 101.0, 0.0, 0.0], dtype=np.float64)
        rc_near = handle_ai(obs_near, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)
        self.assertEqual(rc_near[:4], [1500, 1500, 1500, 1500])

    def test_handle_ai_below_and_above_setpoint_deadband_offset(self):
        # At 0.2m, command +0.6m climb -> outside alt_hold_deadband (40) above 1500
        obs = np.array([0.2, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0], dtype=np.float64)
        self.fc.reset()
        shift_z = 0.6 / MAX_ALTITUDE
        rc_up = handle_ai(obs, self.fc, dt=0.033, desired_shift_z=shift_z, desired_shift_x=0.0, desired_shift_y=0.0)
        self.assertGreater(rc_up[2], 1500 + int(self.fc.inav_thr_deadband))

        # At 1.0m, command -0.6m descent -> outside alt_hold_deadband (40) below 1500
        obs_high = np.array([1.0, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0], dtype=np.float64)
        self.fc.reset()
        rc_down = handle_ai(obs_high, self.fc, dt=0.033, desired_shift_z=-shift_z, desired_shift_x=0.0, desired_shift_y=0.0)
        self.assertLess(rc_down[2], 1500 - int(self.fc.inav_thr_deadband))

    def test_climb_and_descent_velocity_bounds(self):
        # Extreme climb command clamps to 1500 + 120 = 1620
        self.fc.reset()
        self.fc.ref_alt_norm = 0.0
        high_action = np.array([2.5 / MAX_ALTITUDE, 0.0, 0.0, 0.0], dtype=np.float32)
        rc = self.fc.compute_rc_commands(high_action, current_state=0.05 / MAX_ALTITUDE, dt=0.033)
        self.assertEqual(rc[0], self.fc.hover_throttle + self.fc.max_climb_correction)
        self.assertLessEqual(rc[0], self.fc.max_throttle)

        # Extreme descent command clamps to 1500 - 120 = 1380
        self.fc.reset()
        self.fc.ref_alt_norm = 0.0
        low_action = np.array([0.1 / MAX_ALTITUDE, 0.0, 0.0, 0.0], dtype=np.float32)
        rc_desc = self.fc.compute_rc_commands(low_action, current_state=2.0 / MAX_ALTITUDE, dt=0.033)
        self.assertEqual(rc_desc[0], self.fc.hover_throttle + self.fc.max_descent_correction)
        self.assertGreaterEqual(rc_desc[0], self.fc.min_throttle)

    def test_roll_and_pitch_velocity_control_and_bounds(self):
        # Latch reference at origin while airborne at 0.4m
        self.fc.reset()
        obs_origin = np.array([0.4, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0], dtype=np.float64)
        handle_ai(obs_origin, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)

        # Drift in +X (0.3m) and -Y (-0.3m) -> clears pos_hold_deadband (25) with bounded velocity
        obs_drifted = np.array([0.4, 1.5, 0.3, -0.3, 0.0, 0.0, 101.0, 0.0, 0.0], dtype=np.float64)
        handle_ai(obs_drifted, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)
        rc = handle_ai(obs_drifted, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)
        roll, pitch, throttle, yaw, _, _ = rc
        self.assertLess(roll, 1500 - int(self.fc.inav_rp_deadband))
        self.assertGreater(pitch, 1500 + int(self.fc.inav_rp_deadband))
        self.assertEqual(yaw, 1500)

        # Extreme desired shift clamps Roll/Pitch to 1500 ± 80 ([1420, 1580])
        rc_extreme = handle_ai(obs_origin, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=1.0, desired_shift_y=-1.0)
        self.assertEqual(rc_extreme[0], 1500 + int(self.fc.max_rp_correction))
        self.assertEqual(rc_extreme[1], 1500 - int(self.fc.max_rp_correction))
        self.assertEqual(self.fc.max_rp_correction, 80.0)

    def test_level_liftoff_guard_below_ground_threshold(self):
        # When altitude is below 0.050m, Roll/Pitch/Yaw stay strictly at 1500 PWM even with non-zero shift
        self.fc.reset()
        obs_ground = np.array([0.005, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0], dtype=np.float64)
        rc = handle_ai(obs_ground, self.fc, dt=0.033, desired_shift_z=0.2, desired_shift_x=0.5, desired_shift_y=-0.5)
        self.assertEqual(rc[0], 1500)
        self.assertEqual(rc[1], 1500)
        self.assertEqual(rc[3], 1500)


if __name__ == "__main__":
    unittest.main()
