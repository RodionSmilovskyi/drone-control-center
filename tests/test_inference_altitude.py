import unittest
import numpy as np
import json
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../services/drone-inference')))

from flight_controller import FlightController
from inference import handle_ai, MAX_ALTITUDE, MAX_XY_SHIFT

class TestInferenceAltitude(unittest.TestCase):
    def setUp(self):
        self.fc = FlightController()

    def test_vertical_shift_relative_to_reference_math(self):
        # Starting on ground (ref_alt = 0.0m), desired_shift_z = +0.4 / 3.0
        obs_ground = np.array([0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        shift_z = 0.4 / MAX_ALTITUDE
        handle_ai(obs_ground, self.fc, dt=0.033, desired_shift_z=shift_z, desired_shift_x=0.0, desired_shift_y=0.0)
        self.assertAlmostEqual(self.fc.ref_alt_norm, 0.0, places=5)
        self.assertAlmostEqual(self.fc.ref_alt_norm + shift_z, 0.4 / MAX_ALTITUDE, places=5)

        # Starting mid-air (ref_alt = 1.2m), negative desired_shift_z = -0.3 / 3.0 (-0.1) -> target 0.9m
        self.fc.reset()
        obs_midair = np.array([1.2, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        neg_shift_z = -0.3 / MAX_ALTITUDE
        for _ in range(100):
            rc = handle_ai(obs_midair, self.fc, dt=0.033, desired_shift_z=neg_shift_z, desired_shift_x=0.0, desired_shift_y=0.0)
        self.assertAlmostEqual(self.fc.ref_alt_norm, 1.2 / MAX_ALTITUDE, places=5)
        self.assertAlmostEqual(self.fc.throttle_pid.setpoint, 0.9 / MAX_ALTITUDE, places=4)
        self.assertLess(rc[2], self.fc.hover_throttle)

    def test_handle_ai_steady_state_at_setpoint(self):
        # Simulate sensor obs at 0.4m with desired_shift_z = 0.0 (maintain current altitude) and 0 X/Y shift
        obs = np.array([0.4, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        
        # Prime first frame to latch reference and establish last_measurement
        handle_ai(obs, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)
        
        # At steady state at setpoint, PID error is 0 across all axes
        rc = handle_ai(obs, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)
        self.assertEqual(len(rc), 6)
        roll, pitch, throttle, yaw, aux1, aux2 = rc
        self.assertEqual(roll, 1500)
        self.assertEqual(pitch, 1500)
        self.assertEqual(yaw, 1500)
        self.assertEqual(aux1, 1800)
        self.assertEqual(aux2, 1800)
        self.assertEqual(throttle, self.fc.hover_throttle)

    def test_handle_ai_below_setpoint(self):
        # At 0.2m, command a positive vertical shift (+0.6m -> target 0.8m) -> should increase throttle above hover_throttle
        obs = np.array([0.2, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        self.fc.reset()
        shift_z = 0.6 / MAX_ALTITUDE
        handle_ai(obs, self.fc, dt=0.033, desired_shift_z=shift_z, desired_shift_x=0.0, desired_shift_y=0.0)
        rc = handle_ai(obs, self.fc, dt=0.033, desired_shift_z=shift_z, desired_shift_x=0.0, desired_shift_y=0.0)
        throttle = rc[2]
        self.assertGreater(throttle, self.fc.hover_throttle)

    def test_handle_ai_above_setpoint(self):
        # At 1.0m, command a negative vertical shift (-0.6m -> target 0.4m) -> should decrease throttle below hover_throttle
        obs = np.array([1.0, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        self.fc.reset()
        shift_z = -0.6 / MAX_ALTITUDE
        handle_ai(obs, self.fc, dt=0.033, desired_shift_z=shift_z, desired_shift_x=0.0, desired_shift_y=0.0)
        rc = handle_ai(obs, self.fc, dt=0.033, desired_shift_z=shift_z, desired_shift_x=0.0, desired_shift_y=0.0)
        throttle = rc[2]
        self.assertLess(throttle, self.fc.hover_throttle)

    def test_asymmetrical_climb_and_descent_bounds(self):
        # Extreme below setpoint: start at 0.0m, desired_shift_z = +2.5 / 3.0, current = 0.05m
        self.fc.reset()
        self.fc.ref_alt_norm = 0.0
        high_action = np.array([2.5 / MAX_ALTITUDE, 0.0, 0.0, 0.0], dtype=np.float32)
        for _ in range(200):
            rc = self.fc.compute_rc_commands(high_action, current_state=0.05 / MAX_ALTITUDE, dt=0.05)
        self.assertLessEqual(rc[0], self.fc.hover_throttle + self.fc.max_climb_correction)
        self.assertEqual(rc[0], self.fc.hover_throttle + self.fc.max_climb_correction)

        # Extreme above setpoint: start at 0.0m, desired_shift_z = +0.1 / 3.0, current = 1.5m
        self.fc.reset()
        self.fc.ref_alt_norm = 0.0
        low_action = np.array([0.1 / MAX_ALTITUDE, 0.0, 0.0, 0.0], dtype=np.float32)
        for _ in range(200):
            rc = self.fc.compute_rc_commands(low_action, current_state=1.5 / MAX_ALTITUDE, dt=0.05)
        self.assertGreaterEqual(rc[0], self.fc.hover_throttle + self.fc.max_descent_correction)
        self.assertEqual(rc[0], self.fc.hover_throttle + self.fc.max_descent_correction)

    def test_integral_accumulates_below_setpoint(self):
        # Start on ground (0.0m), target +0.5m, holding at 0.35m -> integral accumulates to trim hover throttle
        self.fc.reset()
        shift_z = 0.5 / MAX_ALTITUDE
        obs_ground = np.array([0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        handle_ai(obs_ground, self.fc, dt=0.033, desired_shift_z=shift_z)

        obs_air = np.array([0.35, 1.5, 0.0, 0.0, 0.0, 0.0, 101.0], dtype=np.float64)
        for _ in range(50):
            handle_ai(obs_air, self.fc, dt=0.033, desired_shift_z=shift_z)
        i_initial = self.fc.throttle_pid.integral
        for _ in range(10):
            handle_ai(obs_air, self.fc, dt=0.033, desired_shift_z=shift_z)
        i_after = self.fc.throttle_pid.integral
        self.assertGreater(i_after, i_initial, "Integral must accumulate when below setpoint to adapt to battery sag")

    def test_bidirectional_integral_trimming(self):
        # Verify integral can trim downward for fresh battery (above setpoint) but respects integral_min
        self.fc.reset()
        self.fc.ref_alt_norm = 0.0
        obs_high = np.array([0.6, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        for _ in range(60):
            handle_ai(obs_high, self.fc, dt=0.033, desired_shift_z=0.4 / MAX_ALTITUDE)
        self.assertGreaterEqual(self.fc.throttle_pid.integral, self.fc.throttle_pid.integral_min)
        self.assertLess(self.fc.throttle_pid.integral, 0.0)

        # Verify integral can trim upward for battery sag (below setpoint) up to integral_limit
        self.fc.reset()
        self.fc.ref_alt_norm = 0.0
        obs_low = np.array([0.2, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        for _ in range(60):
            handle_ai(obs_low, self.fc, dt=0.033, desired_shift_z=0.4 / MAX_ALTITUDE)
        self.assertLessEqual(self.fc.throttle_pid.integral, self.fc.throttle_pid.integral_limit)
        self.assertGreater(self.fc.throttle_pid.integral, 0.0)

    def test_roll_and_pitch_pid_control_and_indoor_tilt_cap(self):
        # Latch reference at origin while airborne at 0.4m
        self.fc.reset()
        obs_origin = np.array([0.4, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        handle_ai(obs_origin, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)

        # Simulate drift in +X (0.3m) and -Y (-0.3m) with desired_shift_x=0, desired_shift_y=0
        obs_drifted = np.array([0.4, 1.5, 0.3, -0.3, 0.0, 0.0, 101.0], dtype=np.float64)
        # Prime derivative and check steady PID direction
        handle_ai(obs_drifted, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)
        rc = handle_ai(obs_drifted, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)
        roll, pitch, throttle, yaw, _, _ = rc
        self.assertLess(roll, 1500, "Positive X drift should produce negative Roll correction (< 1500)")
        self.assertGreater(pitch, 1500, "Negative Y drift should produce positive Pitch correction (> 1500)")
        self.assertEqual(yaw, 1500, "Yaw should remain neutral at 1500")

        # Verify indoor 5-degree cap (±50 PWM -> [1450, 1550]) under extreme desired shift
        for _ in range(50):
            rc_extreme = handle_ai(obs_origin, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=1.0, desired_shift_y=-1.0)
        self.assertLessEqual(rc_extreme[0], 1500 + int(self.fc.max_rp_correction))
        self.assertGreaterEqual(rc_extreme[1], 1500 - int(self.fc.max_rp_correction))
        self.assertEqual(self.fc.max_rp_correction, 50.0)

    def test_level_liftoff_guard_below_ground_threshold(self):
        # When altitude is below ground_threshold (e.g. 0.005m), Roll/Pitch/Yaw must stay at 1500 even with non-zero shift
        self.fc.reset()
        obs_ground = np.array([0.005, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        rc = handle_ai(obs_ground, self.fc, dt=0.033, desired_shift_z=0.2, desired_shift_x=0.5, desired_shift_y=-0.5)
        self.assertEqual(rc[0], 1500)
        self.assertEqual(rc[1], 1500)
        self.assertEqual(rc[3], 1500)

if __name__ == "__main__":
    unittest.main()
