"""Attributing an event to the right lap, and reading our position off the log.

Both are arithmetic on times that come from three different clocks, and neither
is visible in the dashboard once it is wrong -- a recovery silently counted on
lap 3 instead of lap 4 looks exactly like a correct one. Hence tests.

The position logic in particular has no live coverage yet: no run so far has
drawn a single speed-limit event, so the only thing exercising rank_at is here.

    python3 -m pytest dashboard/test_lap_detail.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import lap_detail  # noqa: E402


# --- reading AWSIM's own log ---

PLAYER_LOG = """\
[VehicleTuning] sensors.gnss.positionNoiseMean: 0 -> 0
[NpcLap] C2 lap 1: 73.34s
Lap completed: 87.37s, total laps: 1
[NpcLap] C3 lap 1: 90.00s
Lap completed: 130.19s, total laps: 2
[NpcLap] C2 lap 2: 80.00s
"""


def test_player_log_reports_lap_times_and_they_accumulate_into_crossings():
    ego, npcs = lap_detail.parse_player_log(PLAYER_LOG)
    assert ego == [87.37, 217.56]              # 87.37, then +130.19
    assert npcs['C2'] == [73.34, 153.34]
    assert npcs['C3'] == [90.0]


def test_a_log_without_npcs_yields_no_opponents():
    ego, npcs = lap_detail.parse_player_log("Lap completed: 37.9s, total laps: 1\n")
    assert ego == [37.9]
    assert npcs == {}


# --- position ---

def test_position_is_first_while_nobody_has_lapped_yet():
    ego, npcs = lap_detail.parse_player_log(PLAYER_LOG)
    assert lap_detail.rank_at(50.0, ego, npcs) == 1


def test_an_npc_that_crossed_first_is_ahead_of_us():
    # At t=80 C2 has 1 lap and we have none.
    ego, npcs = lap_detail.parse_player_log(PLAYER_LOG)
    assert lap_detail.rank_at(80.0, ego, npcs) == 2


def test_on_equal_laps_whoever_got_there_first_is_ahead():
    # At t=100 all three of us are on 1 lap. C2 crossed at 73.3, before our 87.4,
    # so it is in front; C3 crossed at 90.0, after us, so it is behind. Second.
    ego, npcs = lap_detail.parse_player_log(PLAYER_LOG)
    assert lap_detail.rank_at(100.0, ego, npcs) == 2


def test_position_needs_opponents_to_mean_anything():
    assert lap_detail.rank_at(100.0, [37.0, 74.0], {}) is None


# --- lap attribution ---

CROSSINGS = [100.0, 200.0, 300.0]


def test_an_event_lands_on_the_lap_it_happened_in():
    assert lap_detail.lap_of(50.0, CROSSINGS) == 1
    assert lap_detail.lap_of(150.0, CROSSINGS) == 2
    assert lap_detail.lap_of(250.0, CROSSINGS) == 3


def test_an_event_after_the_last_crossing_is_on_the_lap_never_finished():
    assert lap_detail.lap_of(350.0, CROSSINGS) == 4


def test_a_crossing_the_judge_refused_to_count_does_not_open_a_new_lap():
    """AWSIM logs the line crossing, then declines to count it at the timeout.

    Player.log then has more crossings than there are completed laps, and
    without the clamp a 6-lap race grows a lap 7 out of nothing.
    """
    assert lap_detail.lap_of(350.0, CROSSINGS, completed=2) == 3
    assert lap_detail.lap_of(250.0, CROSSINGS, completed=2) == 3


def test_ros_stamps_are_converted_through_the_lap_both_clocks_saw():
    # The log stamped lap 1 at ROS 1000.0; in race time that moment was 40.0 s.
    to_race = lap_detail.ros_to_race_time([1000.0, 1040.0], [40.0, 40.0])
    assert abs(to_race(1000.0) - 40.0) < 1e-9
    assert abs(to_race(1020.0) - 60.0) < 1e-9


def test_no_completed_lap_means_no_conversion_is_possible():
    assert lap_detail.ros_to_race_time([], [40.0]) is None
    assert lap_detail.ros_to_race_time([1000.0], []) is None


# --- the assembled rows ---

def test_penalties_recoveries_and_position_land_on_the_right_rows():
    ego, npcs = lap_detail.parse_player_log(PLAYER_LOG)
    rows = lap_detail.build(
        lap_times=[87.37, 130.19],
        penalty_events=[
            {'kind': 'crash', 'lap': 1, 'race_time': 45.0, 'duration': 14.0},
            {'kind': 'wall', 'lap': 2, 'race_time': 150.0, 'duration': 8.0},
            # A speed-limit penalty, taken while C2 was a lap up on us.
            {'kind': 'over', 'lap': 2, 'race_time': 160.0, 'duration': 5.0},
        ],
        # Lap 1 was stamped at ROS 1087.37, i.e. race time 87.37.
        recovery_stamps=[1050.0, 1150.0, 1160.0],
        guard_slow_stamps=[],
        lap_stamps=[1087.37, 1217.56],
        ego_crossings=ego,
        npc_crossings=npcs,
    )
    by_lap = {r['lap']: r for r in rows}

    assert by_lap[1]['crash'] == 1 and by_lap[1]['penalty_s'] == 14.0
    assert by_lap[1]['recoveries'] == 1
    assert by_lap[2]['wall'] == 1 and by_lap[2]['over'] == 1
    assert by_lap[2]['recoveries'] == 2
    assert by_lap[2]['penalty_s'] == 13.0
    # At t=160 C2 is a lap up on us; C3 is on our lap but crossed after us. Second.
    assert by_lap[2]['limit_rank'] == 2
    assert by_lap[1]['limit_rank'] is None, "no limit event on lap 1"


def test_rows_exist_for_every_completed_lap_even_a_quiet_one():
    rows = lap_detail.build(lap_times=[37.9, 37.5], penalty_events=[],
                            recovery_stamps=[], guard_slow_stamps=[],
                            lap_stamps=[1037.9, 1075.4])
    assert [r['lap'] for r in rows] == [1, 2]
    assert all(r['crash'] == 0 and r['recoveries'] == 0 for r in rows)
    assert [r['time'] for r in rows] == [37.9, 37.5]


# --- why it did not finish ---

def test_a_finished_run_has_nothing_blocking_it():
    assert lap_detail.blocker(True, 6, 6, [], False) is None


def test_never_moving_is_reported_as_such():
    assert lap_detail.blocker(False, 0, 6, [], True) == 'never completed a lap'


def test_the_blocker_names_the_costs_that_were_actually_measured():
    rows = [{'lap': 1, 'time': 90.0, 'crash': 2, 'wall': 1, 'over': 0,
             'recoveries': 4, 'guard_slow': 0, 'penalty_s': 60.0, 'limit_rank': None}]
    text = lap_detail.blocker(False, 5, 6, rows, True)
    assert '60 s of penalties' in text
    assert '2 kart, 1 wall' in text
    assert '4 recoveries' in text
    assert '1 lap still to run' in text, "one lap, not '1 laps'"
