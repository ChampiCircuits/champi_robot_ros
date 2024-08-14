TABLE_WIDTH_PX = 9640+1700
TABLE_HEIGHT_PX = 5855+1700


def real_to_px(pose: tuple) -> tuple:
    x, y, theta = pose
    x_px = TABLE_WIDTH_PX - y * TABLE_WIDTH_PX / 3.0
    y_px = TABLE_HEIGHT_PX - x * TABLE_HEIGHT_PX / 2.0
    x_px, y_px = int(x_px), int(y_px)
    return (x_px, y_px, theta)

def px_to_real(pose_px: tuple) -> tuple:
    x_px, y_px, theta = pose_px
    y = 3.0 - x_px * 3.0 / TABLE_WIDTH_PX
    x = 2.0 - y_px * 2.0 / TABLE_HEIGHT_PX
    return (x, y, theta)

def id_to_coords(id):
    match id:
        case "B1":
            x=1700//2
            y=1700//2
        case "B2":
            x=850
            y=5855+1700//2
        case "B3":
            x=9640+1700//2
            y=2925+1700//2
        case "Y1":
            x=1700//2
            y=2925+1700//2
        case "Y2":
            x=9640+1700//2
            y=1700//2
        case "Y3":
            x=9640+1700//2
            y=5855+1700//2
    return (x, y)

def action_type_to_color(action_type):
    match action_type:
        case "none":
            return "black"
        case "take_plants":
            return "green"
        case "release_plants":
            return "red"
        case _:
            return "purple"