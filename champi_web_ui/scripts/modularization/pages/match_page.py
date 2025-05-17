import theme

from nicegui import ui, events
from std_msgs.msg import String

from node import init_ros_node
import os, time


zone_has_been_chosen = False
chosen_zone = None

container = None
stepper = None
interactive_image_table = None

last_odom_time_label, tirette_label, e_stop_label = None, None, None
radio_strategy_selection = None

src = 'champi_web_ui/scripts/modularization/resources/table_2025.png'

def ready_to_launch_match():
    ros_node.ready_to_start_match = True
    on_strategy_selected()
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

    global zone_has_been_chosen, chosen_zone
    zone_has_been_chosen = True
    chosen_zone = id


def get_available_strategies():
    strategies_path = os.path.expanduser('~/champi_ws/src/champi_robot_ros/champi_brain/scripts/strategies')
    return [f for f in os.listdir(strategies_path) if f.endswith('.yaml') and os.path.isfile(os.path.join(strategies_path, f))]


def create() -> None:
    @ui.page('/')
    def page_a():
        with theme.frame('Match Page'):
            global interactive_image_table
            global container
            global stepper

            with ui.grid(columns=2).style('width: 100%;padding-top:50px'):
                with ui.column():
                    ui.button("Reset", on_click=reset_all)
                with ui.column():
                    container = ui.column().classes('w-full; items-center')
                    with container:
                        ui.label('Match').classes('text-h4 text-grey-8')
                        stepper = ui.stepper().props('vertical').classes('')
                        with stepper:
                            with ui.step('Positionner le robot sur la table'):
                                with ui.stepper_navigation():
                                    ui.button('Robot placé ✓', on_click=stepper.next)

                            # with ui.step('Choisir la position de départ'):
                            #     interactive_image_table = ui.interactive_image(src, content=svg_zones_overlay).on('svg:pointerdown', zone_chosen).style('width:50%')
                            #     def next_if_zone_chosen():
                            #         if zone_has_been_chosen:
                            #             msg = String()
                            #             msg.data = chosen_zone
                            #             ros_node.zone_pub.publish(msg)
                            #             print("pub ID")
                            #             stepper.next()
                            #         else:
                            #             ui.notify('Choisir une zone...')

                                # with ui.stepper_navigation():
                                #     ui.button('Suivant', on_click=next_if_zone_chosen)
                                #     ui.button('Retour', on_click=stepper.previous).props('flat')


                            with ui.step('Check rapide des nodes'):
                                global last_odom_time_label
                                last_odom_time_label = ui.label('Dernier msg otos: ??')
                                ui.timer(1.0, update_label_odom)

                                with ui.stepper_navigation():
                                    ui.button('Suivant', on_click=stepper.next)
                                    ui.button('Retour', on_click=stepper.previous).props('flat')

                            with ui.step('Vérification tirette/BAU'):
                                global tirette_label, e_stop_label
                                tirette_label = ui.label('La tirette est: ??')
                                e_stop_label = ui.label('Le BAU est: ??')
                                ui.timer(1.0, update_label_tirette_bau)
                                with ui.stepper_navigation():
                                    ui.button('Suivant', on_click=stepper.next)
                                    ui.button('Retour', on_click=stepper.previous).props('flat')

                            with ui.step('Choisir la strategie'):
                                available_strategies = get_available_strategies()
                                global radio_strategy_selection
                                radio_strategy_selection = ui.radio(available_strategies, value='test_strat.yaml')
                                with ui.stepper_navigation():
                                    btn_next = ui.button('Prêt !! 😬', on_click=ready_to_launch_match)
                                    btn_next.bind_enabled_from(radio_strategy_selection, 'value')
                                    ui.button('Retour', on_click=stepper.previous).props('flat')


def update_label_odom():
    if ros_node.last_odom_otos_time is None:
        last_odom_time_label.text = '🛑Dernier msg OTOS il y a ??'
    else:
        delta_t = time.time()-ros_node.last_odom_otos_time
        if delta_t < 0.1:
            last_odom_time_label.text = f'✅Dernier msg otos il y a {delta_t:.2f}s'
        else:
            last_odom_time_label.text = f'🛑Dernier msg otos il y a {delta_t:.2f}s'

def update_label_tirette_bau():
    if ros_node.last_stm_state is None:
        tirette_status = '??'
        e_stop_status = '??'
        tirette_emoji = '🛑'
        e_stop_emoji = '🛑'
    else:
        # Tirette
        if ros_node.last_stm_state.tirette_released:
            tirette_status = 'Relâchée'
            tirette_emoji = '🛑'
        else:
            tirette_status = 'Enfoncée'
            tirette_emoji = '✅'

        # Arrêt d'urgence
        if ros_node.last_stm_state.e_stop_pressed:
            e_stop_status = 'Appuyé'
            e_stop_emoji = '🛑'
        else:
            e_stop_status = 'Relâché'
            e_stop_emoji = '✅'

    tirette_label.text = f'{tirette_emoji} Tirette: {tirette_status}'
    e_stop_label.text = f'{e_stop_emoji} Arrêt d\'urgence: {e_stop_status}'

def on_strategy_selected():
    # send the chosen strategy to the node
    ros_node.pub_strategy(radio_strategy_selection.value)
    stepper.next()

def reset_all():
    ros_node.reset_all()

ros_node = init_ros_node()
# ros_node.zone_pub = ros_node.create_publisher(String, '/start_zone', 10)
