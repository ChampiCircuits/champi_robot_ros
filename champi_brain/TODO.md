# TODOs - Package champi_brain

FEATURES
- garder detect_platform ? faut plutot faire passer l'info par le world state d'ailleurs et qu'après les movements se basent sur la pos en world state
- améliorer la verif de ros initialized ? checker par ex que tous les noeuds sont vivants
- quand une action termine (par ex un actuateur, il faut appeler un callback plutot que d'attendre activement)
- utiliser le feedback de la nav + rejet de goal
- ajouter des diags partout

BUGS
- des fois le brain crashe au démarrage, je crois s'il n'a pas encore recu d'odom
- end_speed est pas bien supportée par le path control

IDEES
- implémenter un état d'init pose automatique ? ✨✨
- le retry avant/arrière devrait plutot être géré dans la sm ? ✨✨
- la simu des actionneurs doit être faite dans le node de simu ✨✨✨
