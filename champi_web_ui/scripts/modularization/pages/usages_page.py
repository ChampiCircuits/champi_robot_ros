import theme

from nicegui import ui
import psutil
from datetime import datetime

#################################################
#################### PAGE #######################
#################################################

def create() -> None:
    @ui.page('/usages')
    def page_usages():
        with theme.frame('Usages'):
            with ui.element("div").style("width: 90%"):
                ui.label('Usages').classes('text-h4 text-grey-8')

                # limited to 60s
                line_plot = ui.line_plot(n=2, limit=60, figsize=(10, 5), update_every=1) \
                    .with_legend(['cpu', 'ram'], loc='upper center', ncol=2)

                cpu = []
                ram = []

                def update_line_plot() -> None:
                    cpu_percent = psutil.cpu_percent()
                    ram_percent = psutil.virtual_memory().percent
                    cpu.append(cpu_percent)
                    ram.append(ram_percent)
                    now = datetime.now()
                    now = datetime.now().second
                    line_plot.push([now], [[cpu_percent], [ram_percent]])

                ui.timer(1.0, update_line_plot)

#################################################
#################### UTILS ######################
#################################################
