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

    def test_cascaded_outer_loop_velocity_clamping(self):
        # Verify outer loop clamps target velocity to +/- max_velocity (0.07 m/s) under large position steps
        self.fc.reset()
        obs = np.array([0.4, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        # Large positive X shift step (+0.50m)
        handle_ai(obs, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.50, desired_shift_y=-0.50)
        self.assertAlmostEqual(self.fc.roll_controller.target_velocity, 0.07, places=4)
        self.assertAlmostEqual(self.fc.pitch_controller.target_velocity, -0.07, places=4)

    def test_cascaded_return_braking_anti_windup(self):
        # Verify that when returning toward origin, the inner trim integral unwinds any wall-unsticking push
        # proportional to distance traveled toward 0 and NEVER winds up in the opposing (braking) direction.
        self.fc.reset()
        obs_origin = np.array([0.4, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        handle_ai(obs_origin, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)

        # 1. Drone is stalled at x = +0.16m (vx = 0) -> accumulates negative integral to push left toward 0
        obs_stalled = np.array([0.4, 1.5, 0.16, 0.0, 0.0, 0.0, 101.0], dtype=np.float64)
        for _ in range(30):
            handle_ai(obs_stalled, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)
        self.assertLess(self.fc.roll_controller.vel_pid.integral, -0.05)

        # 2. Drone now returns toward 0 (x = +0.08m, vx = -0.35 m/s -> norm -0.07)
        # Integral should unwind cleanly to 0, and even over 40 frames of fast return braking, must NOT wind up positive (> 0.01)
        obs_fast_return = np.array([0.4, 1.5, 0.08, 0.0, -0.07, 0.0, 102.0], dtype=np.float64)
        for _ in range(40):
            handle_ai(obs_fast_return, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)
        self.assertAlmostEqual(self.fc.roll_controller.vel_pid.integral, 0.0, places=4)

    def test_cascaded_arbitrary_spatial_shift_waypoint(self):
        # User requirement: move to +0.1z, +0.3x, -0.1y and stay there
        self.fc.reset()
        # Drone latches airborne at alt=0.30m, shift_x=0.0, shift_y=0.0
        obs_start = np.array([0.30, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        shift_z = 0.10 / MAX_ALTITUDE
        shift_x = 0.30 / MAX_XY_SHIFT
        shift_y = -0.10 / MAX_XY_SHIFT
        handle_ai(obs_start, self.fc, dt=0.033, desired_shift_z=shift_z, desired_shift_x=shift_x, desired_shift_y=shift_y)
        
        # Verify target setpoints
        self.assertAlmostEqual(self.fc.ref_shift_x_norm, 0.0, places=4)
        self.assertAlmostEqual(self.fc.ref_shift_y_norm, 0.0, places=4)
        self.assertAlmostEqual(self.fc.roll_controller.target_position, 0.30, places=4)
        self.assertAlmostEqual(self.fc.pitch_controller.target_position, -0.10, places=4)

        # In mid-flight when reaching the waypoint (+0.3x, -0.1y at 0.40m alt with 0 velocity):
        obs_arrived = np.array([0.40, 1.5, 0.30, -0.10, 0.0, 0.0, 105.0], dtype=np.float64)
        for _ in range(50):
            rc = handle_ai(obs_arrived, self.fc, dt=0.033, desired_shift_z=shift_z, desired_shift_x=shift_x, desired_shift_y=shift_y)
        
        # At target waypoint at rest, target velocity is 0 and corrections remain neutral (1500)
        self.assertAlmostEqual(self.fc.roll_controller.target_velocity, 0.0, places=4)
        self.assertAlmostEqual(self.fc.pitch_controller.target_velocity, 0.0, places=4)
        self.assertEqual(rc[0], 1500)  # Roll neutral
        self.assertEqual(rc[1], 1500)  # Pitch neutral

    def test_cascaded_inner_loop_return_braking(self):
        # Verify that when returning toward origin faster than target approach velocity,
        # the velocity loop reverses tilt to apply pre-emptive braking BEFORE crossing origin.
        self.fc.reset()
        obs_origin = np.array([0.4, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        handle_ai(obs_origin, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)

        # Drone is at +0.10m (right of origin), returning toward 0 at vx = -0.25 m/s (normalized vx = -0.25 / 5.0 = -0.05).
        # Position error is 0 - 0.10 = -0.10m -> outer loop target velocity is -0.10 m/s.
        # But measured return velocity is -0.25 m/s, so velocity error is -0.10 - (-0.25) = +0.15 m/s > 0.
        # Even though drone is to the right of origin (x = +0.10m), it must command Roll > 1500 (tilt right) to BRAKE!
        obs_returning_fast = np.array([0.4, 1.5, 0.10, 0.0, -0.05, 0.0, 101.0], dtype=np.float64)
        # Prime last_measurement for derivative
        handle_ai(obs_returning_fast, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)
        rc = handle_ai(obs_returning_fast, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)
        roll = rc[0]
        self.assertGreater(roll, 1500, "Should apply opposing braking tilt (Roll > 1500) when return speed exceeds target speed")

    def test_cascaded_trim_integral_learning(self):
        # Verify that steady-state velocity drift accumulates inner integral to learn trim offset
        self.fc.reset()
        obs_origin = np.array([0.4, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        handle_ai(obs_origin, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)

        # Drift to right with vx = +0.04 m/s (normalized vx = +0.04 / 5.0 = 0.008) at target position 0.0
        obs_drift = np.array([0.4, 1.5, 0.0, 0.0, 0.008, 0.0, 101.0], dtype=np.float64)
        for _ in range(40):
            handle_ai(obs_drift, self.fc, dt=0.033, desired_shift_z=0.0, desired_shift_x=0.0, desired_shift_y=0.0)
        
        # Integral should be negative (learning leftward roll trim to oppose rightward drift)
        self.assertLess(self.fc.roll_controller.vel_pid.integral, 0.0)
        self.assertGreaterEqual(self.fc.roll_controller.vel_pid.integral, self.fc.roll_controller.vel_pid.integral_min)

    def test_cascaded_reset_clears_controllers(self):
        # Verify fc.reset() cleanly clears cascaded controllers
        obs = np.array([0.4, 1.5, 0.2, -0.2, 0.02, -0.02, 100.0], dtype=np.float64)
        for _ in range(10):
            handle_ai(obs, self.fc, dt=0.033, desired_shift_z=0.1, desired_shift_x=0.2, desired_shift_y=-0.2)
        
        self.assertNotEqual(self.fc.roll_controller.position_error, 0.0)
        self.fc.reset()
        self.assertEqual(self.fc.roll_controller.target_position, 0.0)
        self.assertEqual(self.fc.roll_controller.target_velocity, 0.0)
        self.assertEqual(self.fc.roll_controller.position_error, 0.0)
        self.assertEqual(self.fc.roll_controller.vel_pid.integral, 0.0)
        self.assertEqual(self.fc.pitch_controller.target_position, 0.0)
        self.assertEqual(self.fc.pitch_controller.target_velocity, 0.0)
        self.assertEqual(self.fc.pitch_controller.position_error, 0.0)
        self.assertEqual(self.fc.pitch_controller.vel_pid.integral, 0.0)

if __name__ == "__main__":
    unittest.main()
