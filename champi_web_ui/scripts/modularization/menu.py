from nicegui import ui


def menu() -> None:
    ui.link('Launchs', '/').classes(replace='text-white')
    ui.link('Start Match', '/match').classes(replace='text-white')
    ui.link('Debug Actions', '/debug').classes(replace='text-white')
    ui.link('Diagnostics', '/diagnostics').classes(replace='text-white')
    ui.link('IP', '/ip').classes(replace='text-white')
    ui.link('Usages', '/usages').classes(replace='text-white')
