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
    - lookAtPoint & angle du robot --> si on doLookAtPoint alors l'angle d'arrivée du segment n'est plus pris en compte
    - vitesse du segment
    - vitesse au waypoint
    - tolérance au waypoint

CHAMPI_PATH:
le param forcing_type a pour valeurs possibles : 


 si l'adversaire est sur le path            
      "avoid"     on contourne                           
      "wait       on attend à la position de départ
      "force"     on commence le path jusqu'à ce que la distance de collision arrête le robot
 si l'adversaire est sur le goal
      "avoid"     prend le même effet que wait     
      "wait"      on attend à la position de départ
      "force"     on commence le path jusqu'à ce que la distance de collision arrête le robot



## Notes

[
    msg.pose.pose.position.x,
    msg.pose.pose.position.y,
    2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
]
