
# BRAIN
- le nom de la strat choisie doit être paramétrable dans le yaml
- le mode debug de la sm
- Faire que l'etat wait for ros_init n'attend pas que le service set_pose soit dispo.

# WEB INTERFACE
- voir ce qu'il faut faire pour l'UI


# ASK ANDRE





Quand ROS recoit sur le topic /ctrl/actuators
--> je passe actuators_requests a REQUESTED

Quand la STM voit un actuators_requests REQUESTED et actuators_states NOTHING
--> actionne & passe actuators_states a DONE

Quand ROS voit un actuators_states DONE
--> passe actuators_requests a NOTHING et actuators_states a NOTHING