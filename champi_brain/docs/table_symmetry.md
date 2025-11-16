# Table Symmetry Management

## Principle

The game table has **vertical axis symmetry** around x=1.5m (table width = 3m).

- **Yellow team**: starts on the left side
- **Blue team**: starts on the right side

Each element on the table has a corresponding symmetric counterpart.

## Symmetry Transformation

### Position Transformation
For vertical axis symmetry (left-right inversion):
```
x_blue = 3.0 - x_yellow
y_blue = y_yellow  (unchanged)
```

### Angle Transformation
The angle transformation formula is: `theta_blue = (180 - theta_yellow) % 360`

This preserves vertical directions (up/down) while inverting horizontal directions (left/right):

| Direction | Yellow Angle | Blue Angle | Description |
|-----------|--------------|------------|-------------|
| Right →   | 0°           | 180° ←     | Horizontal inversion |
| Up ↑      | 90°          | 90° ↑      | Vertical preserved |
| Left ←    | 180°         | 0° →       | Horizontal inversion |
| Down ↓    | 270°         | 270° ↓     | Vertical preserved |

### Examples
- `elements_1` (yellow): x=1.1, angle=90° → `elements_4` (blue): x=1.9, angle=90°
- `elements_2` (yellow): x=0.775, angle=270° → `elements_5` (blue): x=2.225, angle=270°
- `elements_3` (yellow): x=0.075, angle=0° → `elements_6` (blue): x=2.925, angle=180°

## YAML Configuration

In `initial_world_state_2025.yaml`, each element has a `mirror_id` field referencing its symmetric counterpart:

```yaml
elements:
  - id: elements_1
    type: elements
    x: 1.1
    y: 0.95
    theta_deg: 90.0
    mirror_id: elements_4    # Its symmetric counterpart for blue team

  - id: elements_4
    type: elements
    x: 1.9                   # Symmetric of 1.1 around x=1.5
    y: 0.95
    theta_deg: 90.0          # Same vertical orientation
    mirror_id: elements_1    # Its symmetric counterpart for yellow team
```

## Code Usage

### 1. In Strategy (DSL)

**Always write the strategy for the yellow team!** Automatic transformation happens via `Position.transform_for_blue()`.

```python
# In strategies/__strat_main_2025.py
def create_main_strategy(points_per_action):
    return (
        Strategy()
        .set_init_pose(1.18, 0.16, 30.0)  # Yellow position
        
        # Action to elements_1 (yellow side)
        .move_to("elements_1")  # Use yellow name
    )
```

### 2. In State Machine (Target Resolution)

The State Machine **automatically converts** the `named_target` based on team color:

```python
# Extract from state_machine.py
from champi_brain.world_state.symmetry import get_element_id_for_color

base_target_name = action.named_target  # e.g., "elements_1"
team_color = Color.BLUE if self.strategy_config.color == 'BLUE' else Color.YELLOW
target_name = get_element_id_for_color(base_target_name, team_color)
# If team_color == BLUE: target_name becomes "elements_4"
# If team_color == YELLOW: target_name stays "elements_1"
```

### 3. SymmetryMapper API

```python
from champi_brain.world_state.symmetry import get_element_id_for_color, get_zone_id_for_color
from champi_brain.enums import Color

# Get the correct ID for a given color
element_id = get_element_id_for_color("elements_1", Color.BLUE)
# Returns: "elements_4"

zone_id = get_zone_id_for_color("depart_jaune", Color.BLUE)
# Returns: "depart_bleu"
```

## Complete Workflow

1. **Strategy writing**: Always use yellow names (`elements_1`, `elements_2`, etc.)
2. **Config loading**: `WorldState.from_yaml()` automatically initializes the `SymmetryMapper`
3. **Execution**:
   - If yellow team → uses IDs as-is
   - If blue team → automatically uses `mirror_id`


## Adding New Elements

When adding a new element, remember to:

1. Add the element and its symmetric counterpart in the YAML
2. Assign `mirror_id` in both directions
3. Calculate symmetric position: `x_blue = 3.0 - x_yellow`
4. Calculate symmetric angle: `theta_blue = (180 - theta_yellow) % 360`

Example:
```yaml
- id: new_element_yellow
  x: 0.5
  y: 1.0
  theta_deg: 0
  mirror_id: new_element_blue

- id: new_element_blue
  x: 2.5  # = 3.0 - 0.5
  y: 1.0
  theta_deg: 180  # = (180 - 0) % 360
  mirror_id: new_element_yellow
```