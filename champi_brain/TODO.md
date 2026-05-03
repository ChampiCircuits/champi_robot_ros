# TODOs - Package champi_brain

FEATURES
- améliorer la verif de ros initialized ? checker par ex que tous les noeuds sont vivants
- utiliser le feedback de la nav + rejet de goal
- ajouter des diags partout

BUGS
- des fois le brain crashe au démarrage, je crois s'il n'a pas encore recu d'odom
- end_speed est pas bien supportée par le path control

IDEES
- le retry avant/arrière devrait plutot être géré dans la sm ? ✨✨
- la simu des actionneurs doit être faite dans le node de simu ✨✨✨
