"""
Offline reading and checking of imitation learning sessions recorded with
arena_bringup/launch/record_il.launch (see docs/IL_DATA_COLLECTION_PLAN.md,
sections 1.6 and 1.7).

A session folder holds session_*.bag, params.yaml, versions.yaml, episodes.csv
and notes.txt. Only rosbag and the generated manual_gap_msgs Python messages
are needed; the planner does not have to be built.
"""

import csv
import glob
import math
import os
import re

import yaml

SAMPLE_TOPIC_SUFFIX = "/manual_gap_sample"

# planner/task defaults used when params.yaml does not have them
DEFAULT_GOAL_RADIUS = 1.0
DEFAULT_TIMEOUT = 180.0
GOAL_CHECK_PERIOD = 0.5  # s, task generator goal check
ROBOT_RADIUS = 0.2  # m, rto
COLLISION_EPS = 0.01  # m, as arena_evaluation metrics

SOURCE_NONE, SOURCE_GAP_CURSOR, SOURCE_CLICK_TO_MOVE, SOURCE_MODEL = 0, 1, 2, 3
STATUS_PLANNED = 0


def bag_files(session_dir):
    """session_0.bag, session_1.bag, ... in split order."""
    def index(path):
        match = re.search(r"_(\d+)\.bag$", path)
        return int(match.group(1)) if match else -1
    return sorted(glob.glob(os.path.join(session_dir, "*.bag")), key=index)


def active_bag_files(session_dir):
    """Bags rosbag never closed (crash, kill -9)."""
    return sorted(glob.glob(os.path.join(session_dir, "*.bag.active")))


def read_messages(session_dir, topics=None):
    """(topic, msg, bag time) of every bag of the session, in order."""
    import rosbag
    for path in bag_files(session_dir):
        with rosbag.Bag(path) as bag:
            for topic, msg, t in bag.read_messages(topics=topics):
                yield topic, msg, t


def sample_topic(session_dir, namespace=None):
    if namespace:
        return "/" + namespace.strip("/") + SAMPLE_TOPIC_SUFFIX
    import rosbag
    for path in bag_files(session_dir):
        with rosbag.Bag(path) as bag:
            for topic in bag.get_type_and_topic_info().topics:
                if topic.endswith(SAMPLE_TOPIC_SUFFIX):
                    return topic
    return None


def load_samples(session_dir, namespace=None):
    """ManualGapSample messages of a session, in recording order."""
    topic = sample_topic(session_dir, namespace)
    if topic is None:
        return []
    return [msg for _, msg, _ in read_messages(session_dir, [topic])]


def load_params(session_dir):
    path = os.path.join(session_dir, "params.yaml")
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def load_episode_events(session_dir):
    """Rows of episodes.csv: event (begin/reset), episode, wall_time, sim_time, start, goal."""
    path = os.path.join(session_dir, "episodes.csv")
    if not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def task_settings(params):
    server = params.get("task_generator_server", {}) or {}
    timeout = server.get("timeout", DEFAULT_TIMEOUT)
    return {
        "goal_radius": float(server.get("goal_radius", DEFAULT_GOAL_RADIUS)),
        "timeout": math.inf if timeout is None or timeout < 0 else float(timeout),
    }


# ---------------------------------------------------------------- labels


def nearest_feasible(sample):
    """Index of the feasible candidate whose terminal pose is nearest the
    recorded cursor point (the planner's rule), -1 if none."""
    best, best_dist = -1, math.inf
    for i, c in enumerate(sample.candidates):
        if not c.feasible:
            continue
        dist = math.hypot(c.terminal_pose.position.x - sample.cursor_odom.x,
                          c.terminal_pose.position.y - sample.cursor_odom.y)
        if dist < best_dist:
            best, best_dist = i, dist
    return best


def is_usable_label(sample):
    """Plan 1.4: the human was choosing on this tick."""
    return (sample.source == SOURCE_GAP_CURSOR and sample.cursor_live
            and not sample.in_reset_window and sample.selected_idx >= 0)


def identity(candidate):
    return (candidate.left_model_id, candidate.right_model_id)


def selected_identity(sample):
    return (sample.selected_left_model_id, sample.selected_right_model_id)


def stamp(t):
    return t.secs + t.nsecs * 1e-9


# ---------------------------------------------------------------- episodes


def robot_radius(params, samples):
    """robot_radius of the recorded robot namespace, else the rto default."""
    for s in samples:
        namespace = s.header.frame_id.split("/")[0]
        value = (params.get(namespace, {}) or {}).get("robot_radius")
        if value is not None:
            return float(value)
        break
    return ROBOT_RADIUS


def episode_outcomes(samples, params):
    """Outcome per episode id from the samples: GOAL_REACHED if the last pose
    is within the goal radius (plus the distance covered in one goal check),
    TIMEOUT if the episode lasted the task timeout, else ABORTED; collision
    if the scan came within the robot radius."""
    settings = task_settings(params)
    radius = robot_radius(params, samples)
    episodes = {}
    for s in samples:
        if s.episode_id < 0 or s.in_reset_window:
            continue
        episodes.setdefault(s.episode_id, []).append(s)

    outcomes = {}
    for episode, ticks in sorted(episodes.items()):
        first, last = ticks[0], ticks[-1]
        goal = last.goal_robot_frame
        speed = math.hypot(last.robot_vel.linear.x, last.robot_vel.linear.y)
        distance = math.hypot(goal.x, goal.y)
        duration = stamp(last.header.stamp) - stamp(first.header.stamp)
        min_range = min((min(s.scan.ranges) for s in ticks if s.scan.ranges), default=math.inf)
        if distance < settings["goal_radius"] + speed * GOAL_CHECK_PERIOD:
            result = "GOAL_REACHED"
        elif duration >= settings["timeout"] - 1.0:
            result = "TIMEOUT"
        else:
            result = "ABORTED"
        outcomes[episode] = {
            "result": result,
            "collision": min_range <= radius + COLLISION_EPS,
            "ticks": len(ticks),
            "duration": round(duration, 2),
            "final_goal_distance": round(distance, 2),
        }
    return outcomes


# ---------------------------------------------------------------- checks


def check_session(session_dir, namespace=None, samples=None):
    """Automatic checks of plan 1.6. Returns (checks, report): checks is a list
    of (name, ok, detail)."""
    samples = load_samples(session_dir, namespace) if samples is None else samples
    params = load_params(session_dir)
    checks = []

    def check(name, ok, detail=""):
        checks.append((name, bool(ok), detail))

    check("bags present", bag_files(session_dir), "{} bag(s)".format(len(bag_files(session_dir))))
    active = active_bag_files(session_dir)
    check("no .bag.active files", not active, ", ".join(os.path.basename(a) for a in active))
    check("samples present", samples, "{} samples".format(len(samples)))

    # tick_seq contiguous (a planner restart starts again at 0)
    gaps, restarts = [], 0
    for prev, cur in zip(samples, samples[1:]):
        if cur.tick_seq == prev.tick_seq + 1:
            continue
        if cur.tick_seq < prev.tick_seq:
            restarts += 1
        else:
            gaps.append((prev.tick_seq, cur.tick_seq))
    check("tick_seq contiguous", not gaps,
          "{} gap(s){}; {} planner restart(s)".format(len(gaps), " e.g. {}".format(gaps[:3]) if gaps else "", restarts))

    # recomputed nearest endpoint reproduces the recorded choice
    cursor_ticks = [s for s in samples if s.source == SOURCE_GAP_CURSOR]
    mismatched = [s.tick_seq for s in cursor_ticks if nearest_feasible(s) != s.selected_idx]
    check("selection reproducible", not mismatched,
          "{} of {} cursor ticks differ{}".format(len(mismatched), len(cursor_ticks),
                                                   " e.g. tick {}".format(mismatched[:3]) if mismatched else ""))

    # one clock: cursor stamps (RViz) and sample stamps (planner) agree
    off_clock = [s.tick_seq for s in samples
                 if s.cursor.header.stamp.secs and abs(stamp(s.cursor.header.stamp) - stamp(s.header.stamp)) > 5.0]
    backwards = [cur.tick_seq for prev, cur in zip(samples, samples[1:])
                 if cur.tick_seq == prev.tick_seq + 1 and stamp(cur.header.stamp) < stamp(prev.header.stamp)
                 and cur.episode_id == prev.episode_id]
    check("one clock", not off_clock and not backwards,
          "{} tick(s) with cursor clock off by > 5 s, {} backwards stamp(s) within an episode".format(len(off_clock), len(backwards)))

    # every episode has an outcome
    outcomes = episode_outcomes(samples, params)
    events = load_episode_events(session_dir)
    started = sorted({int(e["episode"]) for e in events if e.get("event") == "reset"})
    missing = [e for e in started if e not in outcomes]
    # the last episode may still be running when recording stops
    if missing and started and missing[-1] == started[-1]:
        missing = missing[:-1]
    check("every episode has an outcome", outcomes and not missing,
          "{} episode(s); without samples: {}".format(len(outcomes), missing))

    report = session_report(samples, outcomes)
    return checks, report


def session_report(samples, outcomes):
    """Counts per session, in decisions rather than ticks (plan 1.7)."""
    usable = [s for s in samples if is_usable_label(s)]
    decisions, previous, last_move = 0, None, None
    for s in usable:
        ident = selected_identity(s)
        moved = last_move is not None and stamp(s.cursor.last_move_wall) != last_move
        if previous is not None and ident != previous and moved:
            decisions += 1
        previous, last_move = ident, stamp(s.cursor.last_move_wall)

    results = {}
    for o in outcomes.values():
        results[o["result"]] = results.get(o["result"], 0) + 1

    return {
        "ticks": len(samples),
        "usable_ticks": len(usable),
        "decisions": decisions,
        "ticks_with_choice": sum(1 for s in usable if sum(c.feasible for c in s.candidates) > 1),
        "cursor_ticks": sum(1 for s in samples if s.source == SOURCE_GAP_CURSOR),
        "click_to_move_ticks": sum(1 for s in samples if s.source == SOURCE_CLICK_TO_MOVE),
        "hold_still_ticks": sum(1 for s in samples if s.hold_still),
        "episodes": len(outcomes),
        "results": results,
        "collision_episodes": sum(1 for o in outcomes.values() if o["collision"]),
    }
