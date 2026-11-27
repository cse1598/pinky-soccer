"""Completed rallies must not time out while the robots are returning home."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace


def test_goal_clears_rally_deadline_before_return_to_start():
    path=Path(__file__).resolve().parents[3]/'scripts/verify_attack_match.py'
    spec=importlib.util.spec_from_file_location('attack_trial_under_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    reward={n:dict(total=0.,goals=0,conceded=0,outs=0) for n in ('yellow','blue')}
    trial=SimpleNamespace(count={},play_started=10.,reward=reward,events=[],
                          now=lambda:84.,log=lambda *args:None)
    module.Match.event(trial,SimpleNamespace(sequence=1,event='GOAL',goal_color='blue'))
    assert trial.play_started is None
    assert trial.reward['yellow']['goals']==1
    trial.play_started=100.
    # The latched old event must not clear a newer rally's timer.
    module.Match.event(trial,SimpleNamespace(sequence=1,event='GOAL',goal_color='blue'))
    assert trial.play_started==100.
