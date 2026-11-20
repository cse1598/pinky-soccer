"""Field geometry and map generation checks against the shared RCJ spec."""

from pathlib import Path

import numpy as np
from pinky_soccer_field.field_geometry import Field, load_spec
from pinky_soccer_field.generate_field_map import (FREE, lidar_height_from_urdf, OCCUPIED,
                                                   rasterize, write_map)
import pytest
import yaml

SPEC = Path(__file__).resolve().parents[2] / 'pinky_soccer_sim' / 'config' / 'rcj_field.yaml'


@pytest.fixture(scope='module')
def spec():
    return load_spec(SPEC)


def test_goal_mouth_at_half_playing_length(spec):
    field = Field(spec)
    for color, sign in (('blue', 1), ('yellow', -1)):
        (x0, y0), (x1, y1) = field.goal_mouth(color)
        assert x0 == x1 == pytest.approx(sign * spec['playing_length'] / 2)
        assert y1 - y0 == pytest.approx(spec['goal_width'])
    assert field.goal_center('blue') == pytest.approx((1.095, 0.0))
    assert field.goal_back('blue')[0][0] == pytest.approx(1.095 + 0.074)


def test_inner_walls_size(spec):
    walls = Field(spec).inner_walls
    assert walls.width == pytest.approx(2.43)
    assert walls.height == pytest.approx(1.82)
    assert (walls.xmin + walls.xmax, walls.ymin + walls.ymax) == pytest.approx((0, 0))


def test_markings(spec):
    field = Field(spec)
    area = field.penalty_area('yellow')
    assert (area.xmin, area.xmax) == pytest.approx((-1.095, -0.845))
    assert (area.ymin, area.ymax) == pytest.approx((-0.40, 0.40))
    assert field.center_circle[2] == pytest.approx(0.30)
    spots = field.neutral_spots
    assert len(spots) == 5 and spots[0] == (0.0, 0.0)
    assert sorted(abs(x) for x, _ in spots[1:]) == pytest.approx([0.645] * 4)


def test_goal_color_flips_attack_direction(spec):
    yellow, blue = Field(spec, 'yellow'), Field(spec, 'blue')
    assert yellow.attack_sign == 1 and blue.attack_sign == -1
    assert yellow.own_goal == blue.opp_goal
    assert yellow.opp_goal == blue.own_goal
    assert yellow.own_goal[0] == pytest.approx(-yellow.opp_goal[0])
    with pytest.raises(ValueError):
        Field(spec, 'red')


def test_ball_in_goal(spec):
    field = Field(spec)
    assert field.ball_in_goal(1.10, 0.0, 'blue')
    assert not field.ball_in_goal(1.09, 0.0, 'blue')
    assert not field.ball_in_goal(1.10, 0.31, 'blue')
    assert field.ball_in_goal(-1.10, 0.1, 'yellow')


def test_rejects_invalid_specs(tmp_path, spec):
    bad = dict(spec, goal_width=spec['playing_width'] + 0.1)
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError):
        load_spec(path)
    path.write_text(yaml.safe_dump(dict(spec, wall_height=-1)))
    with pytest.raises(ValueError):
        load_spec(path)
    path.write_text(yaml.safe_dump(dict(spec, arc_segments=16)))
    with pytest.raises(ValueError):
        load_spec(path)


def test_same_validation_as_sim_generator(tmp_path, spec):
    sim = pytest.importorskip('pinky_soccer_sim.rcj_field')
    cases = [spec, dict(spec, goal_depth=spec['outer_strip']), dict(spec, ramp_base=0.5),
             dict(spec, penalty_radius=spec['penalty_depth']), dict(spec, friction=0)]
    for i, case in enumerate(cases):
        path = tmp_path / f'case{i}.yaml'
        path.write_text(yaml.safe_dump(case))
        ours = theirs = None
        try:
            load_spec(path)
        except ValueError:
            ours = 'reject'
        try:
            sim.load_spec(path)
        except ValueError:
            theirs = 'reject'
        assert ours == theirs, case


def test_lidar_sees_walls_only(spec):
    field = Field(spec)
    names = {o.name for o in field.obstacles_seen_at(0.125)}
    assert names == {'end_wall_+', 'end_wall_-', 'side_wall_+', 'side_wall_-'}
    low = {o.name for o in field.obstacles_seen_at(0.05)}
    assert 'blue_goal_back' in low and 'yellow_goal_roof' in low


def test_map_origin_round_trip(tmp_path, spec):
    field = Field(spec)
    res = 0.01
    grid, origin, _ = rasterize(field, res, 0.125)
    # Field centre must be the map origin: the grid is symmetric around it.
    assert origin[0] == pytest.approx(-grid.shape[1] * res / 2)
    assert origin[1] == pytest.approx(-grid.shape[0] * res / 2)

    def to_cell(x, y):
        return int((y - origin[1]) / res), int((x - origin[0]) / res)

    def to_xy(j, i):
        return origin[0] + (i + 0.5) * res, origin[1] + (j + 0.5) * res
    for x, y in [(0.0, 0.0), (1.0, -0.5), (-1.2, 0.9)]:
        j, i = to_cell(x, y)
        bx, by = to_xy(j, i)
        assert abs(bx - x) <= res and abs(by - y) <= res
    assert grid[to_cell(0.0, 0.0)] == FREE
    assert grid[to_cell(field.inner_walls.xmax + 0.005, 0.0)] == OCCUPIED
    assert grid[to_cell(0.0, field.inner_walls.ymin - 0.005)] == OCCUPIED
    assert grid[to_cell(field.half_length + 0.05, 0.0)] == FREE   # goal is below the LiDAR
    assert np.array_equal(grid, grid[::-1, :]) and np.array_equal(grid, grid[:, ::-1])

    pgm, yaml_path = write_map(grid, origin, res, tmp_path)
    meta = yaml.safe_load(yaml_path.read_text())
    assert meta['origin'][:2] == pytest.approx(list(origin), abs=1e-6)
    header = pgm.read_bytes()[:20].split(b'\n')
    assert header[0] == b'P5' and header[1] == f'{grid.shape[1]} {grid.shape[0]}'.encode()


def test_lidar_height_from_urdf():
    urdf = """<robot name="r">
      <link name="base_footprint"/><link name="base_link"/><link name="m"/>
      <link name="rplidar_link"/>
      <joint name="a" type="fixed"><parent link="base_footprint"/><child link="base_link"/>
        <origin xyz="0 0 0.028" rpy="0 0 0"/></joint>
      <joint name="b" type="fixed"><parent link="base_link"/><child link="m"/>
        <origin xyz="-0.017 0 0.067" rpy="0 0 0"/></joint>
      <joint name="c" type="fixed"><parent link="m"/><child link="rplidar_link"/>
        <origin xyz="0 0 0.030" rpy="0 0 3.14159"/></joint>
    </robot>"""
    assert lidar_height_from_urdf(urdf) == pytest.approx(0.125)
