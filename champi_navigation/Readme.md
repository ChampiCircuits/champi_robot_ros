# Notes sur ChampiPath et son utilisation

CLIENT
- on a le client de l'action (BRAIN) qui veut faire une action (ex aller chercher des plantes)
- donc le chemin qu'on veut faire c'est :

              pose actuelle
                    |
                    v

            cercle des plantes (point A)

                    |
    (traversée du cercle à vitesse réduite)
                    |
                    v

            autre côté du cercle (point B)


Donc BRAIN envoie comme ChampiPath { SEGMENT[A,B, faible vitesse...], }


SERVEUR
- de l'autre côté, le serveur (path_planner_node) recoit le path
- valide l'action et tout #TODO détailler
- ajoute au début un premier SEGMENT[current_pose, A...]
- enregistre le path


Dans sa boucle d'update:
    Tant que goal non atteint :
        - compute le ChampiPath exact depuis pose actuelle JUSQU'AU BOUT (en calculant le path de chaque segment + concaténation)
        - publish le ChampiPath exact sur /plan
        - feedback #TODO détailler
    Puis passe au second goal (point B), et ainsi de suite
    Quand dernier goal atteint, notifie le client d'action

PATH CONTROLLER
- recoit un ChampiPath sur /plan
- suit le path en prenant en compte
    - lookAtPoint & angle du robot
    - vitesse du segment
    - vitesse au waypoint
    - tolérance au waypoint