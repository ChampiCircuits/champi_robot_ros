import theme
from message import message

from nicegui import ui, events

zone_has_been_chosen = False

container = None
stepper = None
interactive_image_table = None

src = 'champi_web_ui/scripts/modularization/resources/table_2024.png'

svg_zones_overlay = '''
                <rect id="B1" x="0" y="0" width="1700" height="1700" stroke="#4E84A2" fill="#4E84A2" pointer-events="all" cursor="pointer" />
                <rect id="Y1" x="0" y="2925" width="1700" height="1700" stroke="#E2BB4E" fill="#E2BB4E" pointer-events="all" cursor="pointer" />
                <rect id="B2" x="0" y="5855" width="1700" height="1700" stroke="#4E84A2" fill="#4E84A2" pointer-events="all" cursor="pointer" />
                
                <rect id="Y2" x="9640" y="0" width="1700" height="1700" stroke="#E2BB4E" fill="#E2BB4E" pointer-events="all" cursor="pointer" />
                <rect id="B3" x="9640" y="2925" width="1700" height="1700" stroke="#4E84A2" fill="#4E84A2" pointer-events="all" cursor="pointer" />
                <rect id="Y3" x="9640" y="5855" width="1700" height="1700" stroke="#E2BB4E" fill="#E2BB4E" pointer-events="all" cursor="pointer" />
            '''
svg_plants_overlay = '''
                <circle id="P1" cx="3785" cy="2645" r="470" fill="green" stroke="green" pointer-events="all" cursor="pointer" />
                <circle id="P2" cx="5675" cy="1895" r="470" fill="green" stroke="green" pointer-events="all" cursor="pointer" />
                <circle id="P3" cx="7560" cy="2645" r="470" fill="green" stroke="green" pointer-events="all" cursor="pointer" />

                <circle id="P4" cx="3785" cy="4910" r="470" fill="green" stroke="green" pointer-events="all" cursor="pointer" />
                <circle id="P5" cx="5675" cy="5660" r="470" fill="green" stroke="green" pointer-events="all" cursor="pointer" />
                <circle id="P6" cx="7560" cy="4910" r="470" fill="green" stroke="green" pointer-events="all" cursor="pointer" />
            '''

def ready_to_launch_match():
    container.clear()
    with container:
        ui.image(src).style('width:75%')
    ui.navigate.to('/in_match')

def zone_chosen(args: events.GenericEventArguments):
    id = args.args['element_id']
    print(id)
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

    interactive_image_table.content = svg_zones_overlay + '''<circle id="P3" cx="{x}" cy="{y}" r="470" fill="black" stroke="black" />'''.format(x=x,y=y)
    global zone_has_been_chosen
    zone_has_been_chosen = True

def create() -> None:
    @ui.page('/match')
    def page_a():
        with theme.frame('Match Page'):
            global interactive_image_table
            global container
            global stepper

            container = ui.column().classes('w-full; items-center').style('padding-top:50px')
            with container:        
                message('Match')
                stepper = ui.stepper().props('vertical').classes('')
                with stepper:
                    with ui.step('Positionning the robot'):
                        ui.label('Robot')
                        with ui.stepper_navigation():
                            ui.button('Next', on_click=stepper.next)

                    with ui.step('Choose start position'):
                        interactive_image_table = ui.interactive_image(src, content=svg_zones_overlay).on('svg:pointerdown', zone_chosen).style('width:50%')
                        def next_if_zone_chosen():
                            if zone_has_been_chosen:
                                stepper.next()
                            else:
                                ui.notify('Please choose a zone...')

                        with ui.stepper_navigation():
                            ui.button('Next', on_click=next_if_zone_chosen)

                    with ui.step('Choose strategy'):
                        radio_strategy_selection = ui.radio(['Homologation', 'Soft','Aggressive', 'OnlyHalfTable'], value='Homologation')
                        with ui.stepper_navigation():
                            ui.button('Next', on_click=stepper.next)

                    with ui.step('Launching nodes'):
                        ui.label('Launch nodes')
                        ui.label('TODO') # TODO
                        print("TODO launch correct nodes")
                        with ui.stepper_navigation():
                            ui.button('Next', on_click=stepper.next)
                            ui.button('Back', on_click=stepper.previous).props('flat')

                    with ui.step('Checking tirette'):
                        ui.label('tirette TODO')
                        print("TODO check tirette") # TODO
                        with ui.stepper_navigation():
                            ui.button('Next', on_click=stepper.next)
                            ui.button('Back', on_click=stepper.previous).props('flat')

                    with ui.step('Deactivating BAU'):
                        ui.label('Please check the BAU')
                        print("TODO auto check the BAU") # TODO
                        with ui.stepper_navigation():
                            ui.button('Done', on_click=ready_to_launch_match)
                            ui.button('Back', on_click=stepper.previous).props('flat')


