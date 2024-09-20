import theme

from nicegui import ui, events
from std_msgs.msg import Int64, Int64MultiArray
import json
from enum import Enum

from node import init_ros_node
from utils import real_to_px, id_to_coords, action_type_to_color

STRATEGIES_FILE_PATH = 'champi_web_ui/scripts/modularization/resources/strategies.json'
TABLE_IMAGE_PATH = 'champi_web_ui/scripts/modularization/resources/table_2024.png'

# strategies
strategies_dics = []
is_creating_a_strategy = False
new_strategy_name = ""

def save_json_strategies():
    with open(STRATEGIES_FILE_PATH, 'w') as fp:
        json.dump(strategies_dics, fp, indent=4)


def open_json_strategies():
    global strategies_dics
    with open(STRATEGIES_FILE_PATH, 'r') as fp:
        strategies_dics = json.load(fp)

open_json_strategies()



interactive_image_table = None

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

svg_current_strat_overlay = ''''''

#################################################
#################### PAGE #######################
#################################################

class ToggleButtonAddStrat(ui.button):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._state = False
        self.on('click', self.toggle)
        self.props(f'color=pink')

    def toggle(self) -> None:
        """Toggle the button state."""
        global is_creating_a_strategy
        self._state = not self._state
        if self._state: # DEBUT DE LA STRAT
            self.props(f'color=red')
            self.set_text("Terminer")
            is_creating_a_strategy = True

        else: # FIN DE LA STRAT
            if new_strategy_name == "":
                ui.notify("Choisir un nom...")
                return
            self.props(f'color=pink')
            self.set_text("Ajouter une strategie")
            is_creating_a_strategy = False
            strategies_dics.append({'name':new_strategy_name, 'points':0}) # TODO points
            save_json_strategies()
            display_strats.refresh()

        self.update()

    def update(self) -> None:
        super().update()

def update_table_display(str_additional=""):
    for strat_ui in stratUIs_list:
        if strat_ui.is_being_seen:
            interactive_image_table.content = svg_current_strat_overlay + str_additional
            return
    interactive_image_table.content = svg_zones_overlay + svg_plants_overlay + svg_current_strat_overlay + str_additional


def zone_chosen(args: events.GenericEventArguments):
    if not is_creating_a_strategy:
        return
    
    id = args.args['element_id']

    x,y = id_to_coords(id)
    update_table_display('''<circle id="P3" cx="{x}" cy="{y}" r="470" fill="black" stroke="black" />'''.format(x=x,y=y))

class StratUI():
    def __init__(self, strat_dic):
        self.strat_dic = strat_dic

        ui.label(self.strat_dic['name'])
        ui.label(self.strat_dic['points'])
        self.see_button = ui.button("Voir", on_click=self.see_button_clicked).props('color=blue')
        self.edit_button = ui.button("Modifier", on_click=self.edit_button_clicked).props('color=orange')
        self.del_button = ui.button("Supprimer", on_click=self.delete_button_clicked).props('color=red')

        self.is_being_seen = False

    def see_button_clicked(self):
        if not 'path_with_actions' in self.strat_dic.keys():
            ui.notify('Aucune action pour cette stratégie...')
            return
        
        self.is_being_seen = not self.is_being_seen
        global svg_current_strat_overlay
        
        # CHANGE STYLE
        if not self.is_being_seen:
            self.see_button.props('color=blue')
            svg_current_strat_overlay = ""
            update_table_display()
            return
        
        self.see_button.props('color=purple')

        # DISPLAY ACTIONS AND PATH
        last_px_coords = None
        for action in self.strat_dic['path_with_actions']:
            x,y,theta = real_to_px((action['x'], action['y'], 0))
            action_type = action["action"]

            svg_current_strat_overlay += '''<circle id="ps" cx={x} cy={y} r="250" fill="{color}" pointer-events="all" cursor="pointer" />'''.format(x=x, y=y, color=action_type_to_color(action_type))
            
            if last_px_coords is not None:
                svg_current_strat_overlay += '''<line x1={x1} y1={y1} x2={x2} y2={y2} style="stroke:black;stroke-width:50"/>'''.format(x1=last_px_coords[0],y1=last_px_coords[1], x2=x, y2=y)
            last_px_coords = (x,y,theta)
            
        update_table_display()

    def edit_button_clicked(self):
        print("Modifier : "+self.strat_dic['name']) # TODO

    def delete_button_clicked(self):
        ui.notify("Suppression de : "+self.strat_dic['name']+"...")
        for i in range(len(strategies_dics)):
            if strategies_dics[i]['name'] == self.strat_dic['name']:
                del strategies_dics[i]
                break
        
        save_json_strategies()
        display_strats.refresh()
        

stratUIs_list = []

@ui.refreshable
def display_strats():
    for strat_dic in strategies_dics:
        stratUIs_list.append(StratUI(strat_dic=strat_dic))



@ui.refreshable
def create() -> None:
    @ui.page('/strategies')
    def page_a():
        with theme.frame('Strategies'):
            with ui.grid(columns=2).style('width: 100%'):
                with ui.column():
                    with ui.element("div"):
                        global interactive_image_table
                        interactive_image_table = ui.interactive_image(TABLE_IMAGE_PATH, content=svg_zones_overlay+svg_plants_overlay).on('svg:pointerdown', zone_chosen).style('width:100%')
                    with ui.row():
                        ToggleButtonAddStrat("Ajouter une strategie")
                        ui.input(label="Nom", placeholder="Nom de la strategie...")\
                            .bind_value_to(globals(), 'new_strategy_name')\
                            .bind_visibility_from(globals(), 'is_creating_a_strategy')

                with ui.column():
                    ui.label("Strategies disponibles").classes('text-h4 text-grey-8')
                    with ui.grid(columns=5):
                        ui.label('Nom').classes('text-h6')
                        ui.label('Points').classes('text-h6')
                        ui.label('')
                        ui.label('')
                        ui.label('')

                        display_strats()



#################################################
#################### UTILS ######################
#################################################

