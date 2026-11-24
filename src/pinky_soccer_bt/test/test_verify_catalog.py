"""The verifier catalog lists exactly the runnable behaviours with the game's port values."""

import os
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml

WORLD = {'playing', 'pose', 'ball', 'opponents', 'own_goal', 'opp_goal'}


@pytest.fixture(scope='module')
def models():
    out = subprocess.run([os.environ['BT_NODE'], '--print-models', os.environ['TREE_FILE']],
                         check=True, capture_output=True, text=True, timeout=30).stdout
    head, _, tail = out.rpartition('<!-- runnable:')
    runnable = set(tail.replace('-->', '').split())
    root = ET.fromstring(head[head.index('<root'):])
    nodes = {}
    for el in root.find('TreeNodesModel'):
        nodes[el.get('ID')] = {
            'kind': el.tag,
            'ports': {p.get('name'): p.get('default') for p in el.findall('input_port')}}
    return nodes, runnable


@pytest.fixture(scope='module')
def catalog():
    with open(os.environ['CATALOG'], encoding='utf-8') as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def tree():
    return ET.parse(os.environ['TREE_FILE']).getroot()


def tree_value(tree, node_id, port):
    """Return the literal port value the game tree uses for node_id, or None."""
    for el in tree.iter(node_id):
        value = el.get(port)
        if value is not None and not value.startswith('{'):
            return value
    return None


def test_items_are_the_runnable_behaviours(models, catalog):
    _, runnable = models
    assert {item['id'] for item in catalog['items']} == runnable


def test_action_ports_and_defaults(models, catalog, tree):
    nodes, _ = models
    for item in catalog['items']:
        if item['kind'] == 'subtree':
            assert tree.find(f"BehaviorTree[@ID='{item['id']}']") is not None, item['id']
            assert not item['ports']
            continue
        model = nodes[item['id']]
        assert model['kind'] == 'Action'
        settable = {k for k in model['ports'] if k not in WORLD}
        assert {p['name'] for p in item['ports']} == settable, item['id']
        # GoToPose/SpinInPlace read the root pose directly, so inputs may list more.
        assert set(item['inputs']) >= {k for k in model['ports'] if k in WORLD}, item['id']
        for port in item['ports']:
            used = tree_value(tree, item['id'], port['name']) or model['ports'][port['name']]
            if used is not None:
                assert float(port['default']) == pytest.approx(float(used)), \
                    (item['id'], port['name'])
            assert port['min'] <= port['default'] <= port['max']
        for key in (item.get('pose_input') or {}).values():
            assert key in settable


def test_probe_matches_game_thresholds(models, catalog, tree):
    nodes, _ = models
    for probe in catalog['probe']:
        model = nodes[probe['target']]
        assert model['kind'] == 'Condition'
        for name, value in probe['ports'].items():
            assert name in model['ports'] and name not in WORLD
            used = tree_value(tree, probe['target'], name) or model['ports'][name]
            assert float(value) == pytest.approx(float(used)), (probe['target'], name)
        assert tree.find(f".//{probe['target']}") is not None
    labels = {p['label'] for p in catalog['probe']}
    for item in catalog['items']:
        done = item.get('done_when')
        assert done is None or done == 'goal_scored' or done in labels
