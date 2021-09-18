#!/usr/bin/env python3
import unittest
import numpy as np

from cereal import log
import cereal.messaging as messaging
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import RW, LongitudinalMpc


class FakePubMaster():
  def send(self, s, data):
    assert data


def run_following_distance_simulation(v_lead, t_end=100.0):
  dt = 0.05
  t = 0.

  x_lead = 140.0

  x_ego = 0.0
  v_ego = v_lead
  a_ego = 0.0

  mpc = LongitudinalMpc()

  while t < t_end:
    # Setup CarState
    CS = messaging.new_message('carState')
    CS.carState.vEgo = float(v_ego)
    CS.carState.aEgo = float(a_ego)

    # Setup model packet
    radarstate = messaging.new_message('radarState')
    lead = log.RadarState.LeadData.new_message()
    lead.modelProb = .75
    lead.dRel = x_lead - x_ego
    lead.vLead = v_lead
    lead.aLeadK = 0.0
    lead.status = True
    radarstate.radarState.leadOne = lead

    # Run MPC
    mpc.set_cur_state(v_ego, a_ego)
    mpc.update(CS.carState, radarstate.radarState, 50)

    # Choose slowest of two solutions
    v_ego, a_ego = float(mpc.v_solution[5]), float(mpc.a_solution[5])

    # Update state
    x_lead += v_lead * dt
    x_ego += v_ego * dt
    t += dt

  return x_lead - x_ego


class TestFollowingDistance(unittest.TestCase):
  def test_following_distanc(self):
    for speed in np.arange(0, 50, 5):
      print(f'Testing {speed} m/s')
      v_lead = float(speed)

      simulation_steady_state = run_following_distance_simulation(v_lead)
      correct_steady_state = RW(v_lead, v_lead) + 4.0

      self.assertAlmostEqual(simulation_steady_state, correct_steady_state, delta=(correct_steady_state*.05 + .3))


if __name__ == "__main__":
  unittest.main()
