import theme
from message import message

from nicegui import ui, events

def create() -> None:
    @ui.page('/in_match')
    def page_in_match():
        with theme.frame('In Match Page'):
            ui.label('Score :').classes('text-h4 text-grey-8')
            ui.label('0 Points').classes('text-h4 text-black-8')

            ui.separator()

            ui.label('Time left :').classes('text-h4 text-grey-8')
            ui.label('100 seconds').classes('text-h4 text-black-8')
            ui.label('TODO')
            # TODO link with the real time/points