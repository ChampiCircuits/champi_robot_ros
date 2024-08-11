import theme
from message import message

from nicegui import ui


def create() -> None:
    @ui.page('/')
    def page_a():
        with theme.frame('Launch Page'):
            message('Launchs')

