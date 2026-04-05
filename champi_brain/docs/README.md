# Champi Brain Documentation

*A brainy mushroom that plans spore-tacular strategies* 🍄✨

Welcome to the Champi Brain documentation! This package contains the high-level strategy and state machine logic for the Champi robot.

## 📚 Documentation Index

### Core Concepts
- **[Table Symmetry Management](table_symmetry.md)** - How the robot handles yellow vs blue team symmetry

### Architecture
- **State Machine** - Manages robot behavior and action execution
- **Strategy DSL** - Domain-specific language for defining robot strategies
- **Match Controller** - Handles match timing and scoring
- **World State** - Tracks game elements and their positions

## 🚀 Quick Start

### Writing a Strategy

Strategies are always written for the **yellow team**. The system automatically transforms them for the blue team:

```python
from champi_brain.strategy_dsl import Strategy, Position, Offset

def create_main_strategy(points_per_action):
    return (
        Strategy()

        .move_to("elements_1")  # Named target from world state
        .do_things()
        .add_points(10)
    )
```

### Understanding Symmetry

The table has vertical axis symmetry. When playing as blue team:
- Positions are mirrored horizontally: `x_blue = 3.0 - x_yellow`
- Vertical directions (up/down) are preserved
- Horizontal directions (left/right) are inverted

See [Table Symmetry Management](table_symmetry.md) for details.

## 📁 Package Structure

```
champi_brain/
├── champi_brain/           # Main package
│   ├── state_machine.py    # State machine logic
│   ├── strategy_dsl.py     # Strategy definition DSL
│   ├── match_controller.py # Match timing and scoring
│   ├── enums.py           # Common enumerations
│   ├── action_executor/   # Action execution
│   └── world_state/       # World state management
│       ├── worldState.py  # World state class
│       └── symmetry.py    # Symmetry mapping
├── strategies/            # Strategy files
├── scripts/              # ROS nodes
├── config/               # Configuration files
├── launch/               # Launch files
└── docs/                 # Documentation (you are here!)
```

## 🔧 Development

### Running Tests
```bash
# Run all tests
cd ~/champi_ws/src/champi_robot_ros/champi_brain
python3 -m unittest discover -s test -v
```

## 📖 Additional Resources

- [Where to find the strategies](../strategies/)
- [Where to find the initial world states](../config/)
