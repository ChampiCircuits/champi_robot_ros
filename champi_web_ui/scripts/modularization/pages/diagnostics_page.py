import theme
from message import message

from nicegui import ui
from diagnostic_msgs.msg import DiagnosticArray
from node import init_ros_node

#################################################
#################### PAGE #######################
#################################################

@ui.refreshable
def create() -> None:
    @ui.page('/diagnostics')
    def page_diagnostics():
        with theme.frame('Diagnostics'):
            message('Diagnostics')
            global diagnostics_dic
            diagnostics_dic = {} # bc each time the page is loaded, it calls create(), deletes the content of the aggrid, so we need to reset global vars
            global grid
            grid = ui.aggrid({
                # 'defaultColDef': {'wrapText':True, 'autoHeight':True},
                'defaultColDef': {},
                'columnDefs': [
                    {'headerName': 'Level', 'field': 'level',
                     'cellClassRules': {
                        'bg-green':'x == 0',
                        'bg-orange':'x == 1',
                        'bg-red':'x == 2',
                        'bg-grey':'x == 3',},
                     'width':'60px'},
                    {'headerName': 'Name', 'field': 'name','width':'200px'},
                    {'headerName': 'Message', 'field': 'message', 'wrapText':True, 'autoHeight': True,'flex':1}
                ],
                'rowData': [
                    # {'level': '', 'name': '', 'message': ''},
                    # {'level': '', 'name': '', 'message': ''},
                    # {'level': '', 'name': '', 'message': ''},
                ],
            }).style('width: 100%; height:800px')
            grid.options['rowHeight'] = 60
            

#################################################
#################### UTILS ######################
#################################################

def update_diags(msg: DiagnosticArray):
    # print("up"):
    global grid
    if grid is None: #Page not created yet
        # print("no grid")
        return

    stamp = msg.header.stamp
    for status in msg.status:
        level = status.level # byte : \x00=OK, \x01=WARN, \x02=ERROR, \x03=STALE
        name = status.name
        message = status.message # can be empty
        hardware_id = status.hardware_id # seems to be always none ?? TODO

        if not name in diagnostics_dic: # we create the index if not already created
        # print(grid) le truc bizarre c'est que grid.options a l'air d'etre reinit quand on va sur la page
        # if not name in grid.options['rowData'][:][1]:
            diagnostics_dic[name] = [len(diagnostics_dic), level, message, hardware_id]
            grid.options['rowData'].append({'level': '', 'name': '', 'message': ''}) # add a line
        else:
            diagnostics_dic[name] = [diagnostics_dic[name][0], level, message, hardware_id]

    for name, diag in diagnostics_dic.items():
        # print(grid.options['rowData']) # rowdata est vide quand on change de page
        # print()
        if grid.options['rowData'] != []:
            grid.options['rowData'][diag[0]]['level'] = int.from_bytes(diag[1],"big")
            grid.options['rowData'][diag[0]]['name'] = name
            grid.options['rowData'][diag[0]]['message'] = diag[2]
        # print("#")
    grid.update()
        


def level_to_color(level):
    if level == "\x00":
        return "green"
    elif level == "\x01":
        return "orange"
    elif level == "\x02":
        return "red"
    elif level == "\x03":
        return "grey"
        

grid = None
# bind the line number in the grid with the name of the diagnostic, and the level, message, hardware_id
diagnostics_dic = {} # diagnostics_dic[name] = [line, level, msg, hardware_id]

ros_node = init_ros_node()
ros_node.create_subscription(DiagnosticArray, '/diagnostics', update_diags, 10)
