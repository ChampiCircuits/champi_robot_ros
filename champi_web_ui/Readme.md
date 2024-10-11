
## TODO LIST
- pb avec le joystick que dans un sens
- finir la gestion des strategies
- sub CAN_state et nb plants pour update affichage
- faire un msg pour pub match start (Empty msg)
- verif etat des process lancés par la page launch
- comment importer CAN_MSGS depuis champi_brain.utils
- always none, la val ID du diagnostic


## Strategies

Les strategies sont stockées au format json. On stocke :
- name
- points
- path_with_actions

path_with_actions est une liste des positions et actions associées
Ex:
[
    {
        'name':'P1',
        'x':0.5,
        'y':0.5,
        'action':'take_plants'
    },
    {
        'name':'B1',
        'x':0.1,
        'y':0.1,
        'action':'release_plants',
        'n_to_release':6
    }
]


action peut être parmi : 'take_plants', 'release_plants' (associé avec un nombre 'n_to_release'), 'none' (pour des waypoints définis manuellement)