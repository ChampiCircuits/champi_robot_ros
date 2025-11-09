from champi_brain.world_state.worldState import WorldState
from champi_brain.enums import NutsBox, Color, ElementState
import unittest

class TestProcessObservation(unittest.TestCase):

    def test_create_when_empty(self):
        ws = WorldState([], [], matching_distance_threshold=0.2, max_missing=2)
        detections = [
            NutsBox(id='d1', x=0.1, y=0.2, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED),
            NutsBox(id='d2', x=1.0, y=1.1, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED)
        ]
        ws.process_observation(detections)
        # two unknown objects should be created
        self.assertEqual(len(ws.elements), 2)
        keys = sorted(ws.elements.keys())
        self.assertTrue(keys[0].startswith('unknown_'))
        self.assertTrue(keys[1].startswith('unknown_'))
        self.assertAlmostEqual(list(ws.elements.values())[0].x, 0.1)
        self.assertAlmostEqual(list(ws.elements.values())[1].x, 1.0)

    def test_match_and_create_unknown(self):
        ws = WorldState([
            NutsBox(id='box1', x=0.0, y=0.0, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED),
            NutsBox(id='box2', x=1.0, y=0.0, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED)
        ], 
        [],
        matching_distance_threshold=0.2,
        max_missing=2)
        # one detection close to box1, one far away -> should update box1 and create unknown
        detections = [
            NutsBox(id='det1', x=0.05, y=0.02, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED),
            NutsBox(id='det2', x=5.0, y=5.0, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED)
        ]
        ws.process_observation(detections)
        # box1 moved
        self.assertAlmostEqual(ws.elements['box1'].x, 0.05, places=3)
        self.assertAlmostEqual(ws.elements['box1'].y, 0.02, places=3)
        # unknown created for far detection
        unknowns = [k for k in ws.elements.keys() if k.startswith('unknown_')]
        self.assertEqual(len(unknowns), 1)
        unk = ws.elements[unknowns[0]]
        self.assertAlmostEqual(unk.x, 5.0)
        self.assertAlmostEqual(unk.y, 5.0)

    def test_unmatched_detection_increments_missing_and_removal(self):
        ws = WorldState([
            NutsBox(id='box1', x=0.0, y=0.0, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED),
            NutsBox(id='box2', x=1.0, y=0.0, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED)
        ], 
        [],
        matching_distance_threshold=0.2,
        max_missing=2)
        # repeatedly receive no detections -> objects should be removed after exceeding max_missing
        for _ in range(ws.max_missing + 1):
            ws.process_observation([])
        self.assertEqual(len(ws.elements), 0)

    def test_distance_threshold_prevents_match(self):
        ws = WorldState([
            NutsBox(id='box1', x=0.0, y=0.0, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED)
        ], 
        [],
        matching_distance_threshold=0.2,
        max_missing=2)
        # detection farther than threshold (default 0.2)
        ws.process_observation([
            NutsBox(id='det1', x=0.3, y=0.0, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED)
        ])
        # box1 should not be updated (remains at 0.0) and an unknown should be created
        self.assertAlmostEqual(ws.elements['box1'].x, 0.0)
        unknowns = [k for k in ws.elements.keys() if k.startswith('unknown_')]
        self.assertEqual(len(unknowns), 1)

    def test_clustered_four_boxes(self):
        # Four boxes placed closely; detections are nearby and within matching threshold
        initial = [
            NutsBox(id='b1', x=0.0, y=0.0, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED),
            NutsBox(id='b2', x=0.05, y=0.0, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED),
            NutsBox(id='b3', x=0.0, y=0.05, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED),
            NutsBox(id='b4', x=0.05, y=0.05, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED),
        ]
        ws = WorldState(initial, [], matching_distance_threshold=0.1, max_missing=2)
        detections = [
            NutsBox(id='d1', x=0.01, y=-0.01, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED),
            NutsBox(id='d2', x=0.06, y=0.01, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED),
            NutsBox(id='d3', x=-0.01, y=0.06, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED),
            NutsBox(id='d4', x=0.04, y=0.04, theta_deg=0.0, state=ElementState.ON_TABLE, color=Color.NOT_INITIALIZED),
        ]
        ws.process_observation(detections)
        # All original boxes should be updated to detection positions (no unknowns created)
        for bid, det in zip(['b1','b2','b3','b4'], detections):
            self.assertAlmostEqual(ws.elements[bid].x, det.x, places=3)
            self.assertAlmostEqual(ws.elements[bid].y, det.y, places=3)
        unknowns = [k for k in ws.elements.keys() if k.startswith('unknown_')]
        self.assertEqual(len(unknowns), 0)

if __name__ == '__main__':
    unittest.main()
