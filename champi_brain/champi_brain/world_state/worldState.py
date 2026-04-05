from typing import List, Dict
import yaml
import numpy as np
from scipy.optimize import linear_sum_assignment
from champi_brain.enums import ZoneType, Color, ElementState, NutsBox, Zone
from ament_index_python.packages import get_package_share_directory


class WorldState:
    def __init__(self, elements: List[NutsBox], init_zones: List[Zone], matching_distance_threshold:float, max_missing: int):
        """
        elements: list of NutsBox objects representing the initial known state
        init_zones: list of Zone objects representing predefined zones
        matching_distance_threshold: maximum distance to consider a detection matching an existing object (in meters)
        max_missing: number of consecutive misses before removing an object
        """
        self.elements: Dict[str, NutsBox] = {e.id: e for e in elements}
        self.zones: List[Zone] = init_zones
        self.matching_distance_threshold: float = matching_distance_threshold
        self.max_missing: int = max_missing

    def get_elements_by_state(self, state: ElementState) -> List[NutsBox]:
        return [e for e in self.elements.values() if e.state == state]

    def _create_object(self, det: NutsBox) -> str:
        new_id = f"unknown_{len(self.elements)}"
        self.elements[new_id] = NutsBox(
            id=new_id,
            x=det.x,
            y=det.y,
            theta_deg=det.theta_deg,
            state=det.state,
            color=det.color
        )
        return new_id

    def _remove_lost_objects(self) -> None:
        to_remove = [obj_id for obj_id, el in self.elements.items() if el.missing_count > self.max_missing]
        for obj_id in to_remove:
            del self.elements[obj_id]

    def process_observation(self, detections:List[NutsBox]) -> None:
        """
        Update tracked objects from a new set of detections using a global assignment (Hungarian) algorithm.
        detections: list of NutsBox-like objects with x,y
        """

        # If no existing objects, create one per detection
        if len(self.elements) == 0:
            for det in detections:
                self._create_object(det)
            return

        obj_list = list(self.elements.values())
        obj_ids = list(self.elements.keys())

        if len(detections) == 0:
            # increment missing counters for all objects
            for obj_id in obj_ids:
                self.elements[obj_id].missing_count += 1
            self._remove_lost_objects()
            return

        obj_positions = np.array([[el.x, el.y] for el in obj_list])
        det_positions = np.array([[det.x, det.y] for det in detections])

        # If one side has size 0 handled above
        cost_matrix = np.linalg.norm(obj_positions[:, np.newaxis] - det_positions[np.newaxis, :], axis=2)

        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        matched_detections = set()
        matched_objects = set()

        # Apply associations
        for r, c in zip(row_ind, col_ind):
            distance = cost_matrix[r, c]
            obj_id = obj_ids[r]
            if distance < self.matching_distance_threshold:
                det = detections[c]
                # update object
                self.elements[obj_id].missing_count = 0 # reset missing counter
                self.elements[obj_id].x = det.x
                self.elements[obj_id].y = det.y
                self.elements[obj_id].theta_deg = det.theta_deg
                self.elements[obj_id].state = det.state
                if isinstance(self.elements[obj_id], NutsBox):
                    self.elements[obj_id].color = det.color

                matched_detections.add(c)
                matched_objects.add(obj_id)
            else:
                # not matched due to distance, will be considered missing
                pass

        # Increment missing counters for unmatched objects
        for i, obj_id in enumerate(obj_ids):
            if obj_id not in matched_objects:
                self.elements[obj_id].missing_count += 1

        # Create objects for unmatched detections
        for i, det in enumerate(detections):
            if i not in matched_detections:
                self._create_object(det)

        # Remove lost objects
        self._remove_lost_objects()

    @staticmethod
    def from_yaml(path: str) -> tuple[List[NutsBox], List[Zone]]:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        raw_elements = data.get('elements', data.get('boxes', []))
        init_elements = []
        for element in raw_elements:
            init_elements.append(NutsBox(
                id=element['id'],
                x=element['x'],
                y=element['y'],
                theta_deg=element.get('theta_deg', 0.0),
                color=Color(element.get('color', Color.NOT_INITIALIZED.value)),
                state=ElementState(element.get('state', 'on_table'))
            ))

        raw_zones = data.get('zones', [])
        init_zones = []
        for zone in raw_zones:
            init_zones.append(Zone(
                id=zone['id'],
                x=zone['x'],
                y=zone['y'],
                width=zone['width'],
                height=zone['height'],
                zone_type=ZoneType(zone.get('type', 'not_in_a_zone')),
                # color from string to enum value or None
                color=Color(zone.get('color', None)) if 'color' in zone else None
            ))

        return init_elements, init_zones



if __name__ == "__main__":
    from champi_brain.world_state.utils import print_all_elements_with_rich, print_all_zones_with_rich

    init_elements, init_zones = WorldState.from_yaml(get_package_share_directory('champi_brain') + "/config/initial_world_state.yaml")
    world = WorldState(init_elements, init_zones, matching_distance_threshold=0.3, max_missing=2)

    print_all_zones_with_rich(world, "All zones")
    print_all_elements_with_rich(world, "All elements at start")

    # Simulate observations
    observations = [
        NutsBox(id="det1", x=1.10, y=0.1, theta_deg=0, state=ElementState.ON_TABLE, color=Color.BLUE),
        NutsBox(id="det2", x=1.5, y=0.8, theta_deg=0, state=ElementState.ON_TABLE, color=Color.BLUE),
        NutsBox(id="det3", x=2.0, y=2.0, theta_deg=0, state=ElementState.ON_TABLE, color=Color.YELLOW)   # new unknown
    ]
    world.process_observation(observations)
    
    print_all_elements_with_rich(world,"All elements after observation")