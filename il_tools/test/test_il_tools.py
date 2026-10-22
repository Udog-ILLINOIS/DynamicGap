#!/usr/bin/env python3
"""Offline IL tools: bag checks (plan 1.6) and dataset building (plan 1.7) on synthetic sessions.
Needs rosbag and the manual_gap_msgs Python messages (source the workspace)."""

import gzip
import json
import math
import os
import shutil
import sys
import tempfile
import unittest

import rosbag
import rospy
import yaml
from manual_gap_msgs.msg import ManualGapCandidate, ManualGapSample

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import il_bags  # noqa: E402
import il_dataset  # noqa: E402

TOPIC = "/rto/manual_gap_sample"


def candidate(idx, left, right, terminal, feasible=True, cursor=None):
    c = ManualGapCandidate()
    c.traj_idx = idx
    c.left_model_id, c.right_model_id = left, right
    c.feasible = feasible
    c.shown = feasible
    if feasible:
        c.terminal_pose.position.x, c.terminal_pose.position.y = terminal
        c.terminal_pose.orientation.w = 1.0
    c.cursor_dist = math.hypot(terminal[0] - cursor[0], terminal[1] - cursor[1]) if (cursor and feasible) else float("nan")
    c.cursor_bearing_diff = 0.0 if (cursor and feasible) else float("nan")
    return c


def sample(seq, t, episode, cursor=(3.0, 0.0), gaps=((1, 2, (3.0, 1.0)), (3, 4, (3.0, -1.0))),
           select=True, live=True, reset_window=False, goal=(10.0, 0.0), scan_min=2.0):
    s = ManualGapSample()
    s.schema_version = s.SCHEMA_VERSION
    s.header.stamp = rospy.Time.from_sec(t)
    s.header.frame_id = "rto/odom"
    s.wall_stamp = rospy.Time.from_sec(1000.0 + t)
    s.tick_seq = seq
    s.episode_id = episode
    s.in_reset_window = reset_window
    s.scan.ranges = [scan_min, 4.0]
    s.goal_robot_frame.x, s.goal_robot_frame.y = goal
    s.cursor.header.stamp = rospy.Time.from_sec(t)
    s.cursor.tool_active = True
    s.cursor.pick_ok = live
    s.cursor.last_move_wall = rospy.Time.from_sec(1000.0 + t)
    s.cursor_odom.x, s.cursor_odom.y = cursor
    s.cursor_valid = True
    s.cursor_live = live
    s.candidates = [candidate(i, l, r, term, cursor=cursor) for i, (l, r, term) in enumerate(gaps)]
    if select:
        best = il_bags.nearest_feasible(s)
        s.source = s.SOURCE_GAP_CURSOR
        s.selected_idx = best
        s.selected_left_model_id, s.selected_right_model_id = gaps[best][0], gaps[best][1]
    else:
        s.source = s.SOURCE_NONE
        s.selected_idx = -1
    return s


class SessionTestCase(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        with open(os.path.join(self.dir, "params.yaml"), "w") as f:
            yaml.safe_dump({"task_generator_server": {"timeout": 60.0, "goal_radius": 1.0}}, f)
        with open(os.path.join(self.dir, "episodes.csv"), "w") as f:
            f.write("event,episode,wall_time,sim_time,start,goal\nreset,0,0,0,,\nreset,1,0,0,,\n")

    def tearDown(self):
        shutil.rmtree(self.dir)

    def write(self, samples, name="session_0.bag"):
        with rosbag.Bag(os.path.join(self.dir, name), "w") as bag:
            for s in samples:
                bag.write(TOPIC, s, s.header.stamp)

    def good_samples(self):
        samples = [sample(i, 0.1 * i, 0) for i in range(20)]
        # episode 1: cursor points at the lower gap; the last tick is at the goal
        samples += [sample(20 + i, 2.0 + 0.1 * i, 1, cursor=(3.0, -1.2), goal=(0.5, 0.0) if i == 19 else (5.0, 0.0))
                    for i in range(20)]
        return samples


class CheckTest(SessionTestCase):

    def checks(self):
        checks, report = il_bags.check_session(self.dir)
        return {name: ok for name, ok, _ in checks}, report

    def test_good_session_passes(self):
        self.write(self.good_samples())
        checks, report = self.checks()
        self.assertTrue(all(checks.values()), checks)
        self.assertEqual(report["usable_ticks"], 40)
        self.assertEqual(report["episodes"], 2)
        self.assertEqual(report["results"], {"ABORTED": 1, "GOAL_REACHED": 1})

    def test_split_bags_are_read_in_order(self):
        samples = self.good_samples()
        self.write(samples[:15], "session_0.bag")
        self.write(samples[15:], "session_1.bag")
        checks, _ = self.checks()
        self.assertTrue(checks["tick_seq contiguous"])

    def test_tick_gap_detected(self):
        samples = self.good_samples()
        del samples[5]
        self.write(samples)
        self.assertFalse(self.checks()[0]["tick_seq contiguous"])

    def test_wrong_label_detected(self):
        samples = self.good_samples()
        samples[3].selected_idx = 1 - samples[3].selected_idx
        self.write(samples)
        self.assertFalse(self.checks()[0]["selection reproducible"])

    def test_clock_mismatch_detected(self):
        samples = self.good_samples()
        samples[7].cursor.header.stamp = rospy.Time.from_sec(1.7e9)  # wall clock stamp
        self.write(samples)
        self.assertFalse(self.checks()[0]["one clock"])

    def test_active_bag_detected(self):
        self.write(self.good_samples())
        open(os.path.join(self.dir, "session_1.bag.active"), "w").close()
        self.assertFalse(self.checks()[0]["no .bag.active files"])

    def test_episode_without_samples_detected(self):
        with open(os.path.join(self.dir, "episodes.csv"), "a") as f:
            f.write("reset,2,0,0,,\nreset,3,0,0,,\n")
        self.write(self.good_samples())
        # episode 3 may still be running; episode 2 has no samples at all
        self.assertFalse(self.checks()[0]["every episode has an outcome"])

    def test_collision_flagged(self):
        samples = self.good_samples()
        samples[30].scan.ranges = [0.15, 4.0]
        outcomes = il_bags.episode_outcomes(samples, il_bags.load_params(self.dir))
        self.assertFalse(outcomes[0]["collision"])
        self.assertTrue(outcomes[1]["collision"])


class DatasetTest(SessionTestCase):

    def build(self, **opts):
        out = os.path.join(self.dir, "out")
        summary = il_dataset.build_dataset([self.dir], out, dict({"splits": {"train": 1.0}}, **opts))
        with gzip.open(os.path.join(out, "train.jsonl.gz"), "rt") as f:
            examples = [json.loads(line) for line in f]
        return examples, summary["sessions"][os.path.basename(self.dir)]["counts"]

    def test_examples_and_labels(self):
        self.write(self.good_samples())
        examples, counts = self.build()
        self.assertEqual(counts["kept"], 40)
        self.assertTrue(all(e["obs_tick"] == e["label_tick"] for e in examples))
        self.assertEqual({e["label"] for e in examples if e["episode"] == 0}, {0})
        self.assertEqual({e["label"] for e in examples if e["episode"] == 1}, {1})

    def test_lag_pairs_earlier_observation_by_identity(self):
        samples = self.good_samples()
        # in episode 0 the candidate order flips half way: identity, not index, must be used
        for s in samples[10:20]:
            s.candidates = list(reversed(s.candidates))
            s.selected_idx = il_bags.nearest_feasible(s)
        self.write(samples)
        examples, counts = self.build(lag_s=0.3)
        # first three ticks of each episode have no tick 0.3 s earlier
        self.assertEqual(counts["no_obs_tick"], 6)
        ex = [e for e in examples if e["label_tick"] == 11][0]
        self.assertEqual(ex["obs_tick"], 8)
        self.assertEqual(ex["label"], 0)  # gap (1, 2) is still index 0 at tick 8

    def test_identity_lost_dropped(self):
        samples = self.good_samples()
        samples[11].candidates[0].left_model_id = 99  # gap (1, 2) split at tick 11
        self.write(samples)
        # only tick 13 is paired with tick 11
        _, counts = self.build(lag_s=0.15)
        self.assertEqual(counts["identity_lost"], 1)

    def test_unusable_ticks_and_thresholds(self):
        samples = self.good_samples()
        samples[2].cursor_live = False
        samples[3].in_reset_window = True
        samples[4] = sample(4, 0.4, 0, select=False)
        self.write(samples)
        _, counts = self.build(max_cursor_dist=0.5)
        self.assertEqual(counts["usable"], 37)
        # episode 0 cursor is 1 m from both terminals; episode 1 is 0.2 m from its pick
        self.assertEqual(counts["threshold"], 17)

    def test_collision_episodes_dropped_or_kept(self):
        samples = self.good_samples()
        samples[30].scan.ranges = [0.15, 4.0]
        self.write(samples)
        _, counts = self.build()
        self.assertEqual(counts["collision_episode"], 20)
        shutil.rmtree(os.path.join(self.dir, "out"))
        _, counts = self.build(drop_collision_episodes=False)
        self.assertEqual(counts["kept"], 40)

    def test_split_is_by_episode_and_deterministic(self):
        splits = {"train": 0.5, "val": 0.5}
        a = [il_dataset.episode_split("s", e, splits, "seed") for e in range(50)]
        b = [il_dataset.episode_split("s", e, splits, "seed") for e in range(50)]
        self.assertEqual(a, b)
        self.assertEqual(set(a), {"train", "val"})


if __name__ == "__main__":
    unittest.main()
