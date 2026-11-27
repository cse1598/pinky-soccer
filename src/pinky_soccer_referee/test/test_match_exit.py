"""An uncertain crossing can stop the game without inventing a goal."""
import sys
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from referee_core import Rules


def test_reacquired_exit_stops_without_scoring():
    f=SimpleNamespace(length=1.66,width=1.06)
    goals={-1:('yellow',-.2,.2),1:('blue',-.2,.2)}
    r=Rules(stop_on_reacquired_exit=True)
    r.update((0,0),f,.02135,goals)
    assert r.update(None,f,.02135,goals) is None
    assert r.update((.86,0),f,.02135,goals) is None
    assert r.update((.86,0),f,.02135,goals) is None
    assert r.update((.86,0),f,.02135,goals)==('UNVERIFIED_EXIT','')
    assert r.scores=={'yellow':0,'blue':0}
    assert r.update((.86,0),f,.02135,goals) is None


def test_unknown_ball_never_becomes_an_out():
    r=Rules(stop_on_reacquired_exit=True)
    f=SimpleNamespace(length=1.66,width=1.06)
    for _ in range(20):assert r.update(None,f,.02135,{}) is None
    assert not r.last_event
