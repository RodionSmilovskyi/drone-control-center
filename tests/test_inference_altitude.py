import unittest
import numpy as np
import json
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../services/drone-inference')))

from flight_controller import FlightController
from inference import handle_ai, MAX_ALTITUDE

class TestInferenceAltitude(unittest.TestCase):
    def setUp(self):
        self.fc = FlightController()

    def test_altitude_normalization_math(self):
        # Test target_alt = 0.4m
        target_alt = 0.4
        target_alt_norm = target_alt / MAX_ALTITUDE  # 0.4 / 3.0 = 0.13333...
        action_alt = (target_alt_norm * 2.0) - 1.0   # in [-1, 1]

        # Re-derive desired_alt_norm inside flight controller
        desired_alt_norm = (action_alt + 1.0) / 2.0
        self.assertAlmostEqual(desired_alt_norm, target_alt_norm, places=5)
        self.assertAlmostEqual(desired_alt_norm * MAX_ALTITUDE, target_alt, places=5)

    def test_handle_ai_dynamic_alt(self):
        # Simulate sensor obs with altitude = 0.4m (at setpoint)
        obs = np.array([0.4, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        
        # Prime first frame to establish last_measurement
        handle_ai(obs, self.fc, dt=0.033, target_alt=0.4)
        
        # At steady state at setpoint, PID error is 0, throttle should equal hover_throttle
        rc = handle_ai(obs, self.fc, dt=0.033, target_alt=0.4)
        self.assertEqual(len(rc), 6)
        roll, pitch, throttle, yaw, aux1, aux2 = rc
        self.assertEqual(roll, 1500)
        self.assertEqual(pitch, 1500)
        self.assertEqual(yaw, 1500)
        self.assertEqual(aux1, 1800)
        self.assertEqual(aux2, 1800)
        self.assertEqual(throttle, self.fc.hover_throttle)

    def test_handle_ai_below_setpoint(self):
        # Below target (target=0.8m, current=0.2m) -> should increase throttle above hover_throttle
        obs = np.array([0.2, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        self.fc.reset()
        handle_ai(obs, self.fc, dt=0.033, target_alt=0.8)
        rc = handle_ai(obs, self.fc, dt=0.033, target_alt=0.8)
        throttle = rc[2]
        self.assertGreater(throttle, self.fc.hover_throttle)

    def test_handle_ai_above_setpoint(self):
        # Above target (target=0.4m, current=1.0m) -> should decrease throttle below hover_throttle
        obs = np.array([1.0, 1.5, 0.0, 0.0, 0.0, 0.0, 100.0], dtype=np.float64)
        self.fc.reset()
        handle_ai(obs, self.fc, dt=0.033, target_alt=0.4)
        rc = handle_ai(obs, self.fc, dt=0.033, target_alt=0.4)
        throttle = rc[2]
        self.assertLess(throttle, self.fc.hover_throttle)

    def test_json_parsing_logic(self):
        payload = json.dumps({"mode": "ai", "target_alt": 0.65})
        data = json.loads(payload)
        self.assertEqual(data["mode"], "ai")
        self.assertEqual(data["target_alt"], 0.65)

if __name__ == "__main__":
    unittest.main()
