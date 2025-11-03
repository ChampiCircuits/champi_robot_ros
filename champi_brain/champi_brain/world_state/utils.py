from rich.console import Console
from rich.text import Text
from rich.console import Console
from rich.table import Table
from champi_brain.world_state.worldState import WorldState
from champi_brain.enums import ZoneType


def print_all_elements_with_rich(world: WorldState, title: str = "") -> None:
    console = Console()

    table = Table(title=title, show_header=True, header_style="bold blue")

    table.add_column("ID", justify="left")
    table.add_column("X", justify="right")
    table.add_column("Y", justify="right")
    table.add_column("State", justify="left")
    table.add_column("Missing#", justify="right")
    table.add_column("Couleur", justify="left")

    for e in world.elements.values():
        color_value = e.color.value if e.color else "N/A"
        color_style = color_value.lower() if e.color and e.color.name != "NOT_INITIALIZED" else "white"
        missing = getattr(e, "missing_count", "N/A")

        # Créer une cellule colorée pour "Couleur"
        colored_cell = Text(color_value, style=color_style)

        # Ajouter la ligne sans styliser toute la ligne
        table.add_row(
            str(e.id),
            f"{e.x:.2f}",
            f"{e.y:.2f}",
            e.state.name,
            str(missing),
            colored_cell
        )

    console.print(table)


def print_all_zones_with_rich(world: WorldState, title: str="") -> None:
    console = Console()
    
    table = Table(title=title, show_header=True, header_style="bold magenta")

    table.add_column("ID", justify="left")
    table.add_column("X", justify="right")
    table.add_column("Y", justify="right")
    table.add_column("Width", justify="right")
    table.add_column("Height", justify="right")
    table.add_column("Type", justify="left")
    table.add_column("Couleur", justify="left")

    for z in world.zones:
        color_str = z.color.value if z.color else "N/A"
        color_style = z.color.value.lower() if z.color else "white"
        zone_style = "red" if z.type in [ZoneType.SECURE_ZONE, ZoneType.NOT_IN_A_ZONE] else "green" if z.type == ZoneType.PLACEMENT_ZONE else "white"

        colored_cell = Text(color_str, style=color_style)
        zone_cell = Text(z.type.name, style=zone_style)

        table.add_row(
            str(z.id),
            f"{z.x:.2f}",
            f"{z.y:.2f}",
            f"{z.width:.2f}",
            f"{z.height:.2f}",
            zone_cell,
            colored_cell
        )

    console.print(table)

