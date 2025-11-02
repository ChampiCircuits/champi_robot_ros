# TODOs - Package champi_brain

> **Date de génération**: 2 novembre 2025  
> **Branch**: feature/action-planner

---

## 🎯 Configuration & Paramètres

- end_speed est pas bien supportée par le path control
- des fois le brain crashe au démarrage, je crois s'il n'a pas encore recu d'odom
- le probleme de la pose init qui est toujours pas réglé
- l'ajout des points gagnés ca doit être un fichier de config 

### 1. Charger les paramètres de mouvement depuis la config ROS
**Fichier**: `strategy_dsl.py` (ligne 56)  
**Priorité**: 🟡 Moyenne  
**Contexte**:
```python
# TODO ces params par défaut devront être lus dans le champi.config.yaml via ros
@dataclass
class MotionParams:
    """Motion parameters with default values"""
    speed: float = 1.0          # m/s
    end_speed: float = 0.0      # m/s
    accel_linear: float = 0.5   # m/s²
    accel_angular: float = 6.0  # rad/s²
    use_dynamic_layer: bool = False
```
**Description**: Les paramètres par défaut de mouvement sont actuellement codés en dur dans la dataclass. Ils devraient être chargés depuis le fichier de configuration YAML du robot via les paramètres ROS.

**Action**: Créer un système pour lire `champi.config.yaml` et initialiser `MotionParams` avec ces valeurs.

---

## 🤖 State Machine - Initialisation

### 2. Clarifier pourquoi stop_requested = True au reset
**Fichier**: `state_machine.py` (ligne 120)  
**Priorité**: 🟢 Basse  
**Contexte**:
```python
def reset(self):
    self.reset_flags()
    self.stop_requested = True # TODO why true ???
    self.come_home_requested = False
    self.wait_to_come_home_requested = False
```
**Description**: Le flag `stop_requested` est mis à `True` lors du reset, mais la raison n'est pas claire. Est-ce intentionnel ? Devrait-il être à `False` ?

**Action**: Documenter la raison ou corriger si c'est une erreur.

---

### 3. Implémenter la transition vers moveToInitPose
**Fichier**: `state_machine.py` (ligne 62)  
**Priorité**: 🔴 Haute (feature désactivée)  
**Contexte**:
```python
self.sm.add_transition('init_next', 'init_waitForUserChooseConfig', 'init_waitForTirette', conditions='user_has_chosen_config')
# self.sm.add_transition('init_next', 'init_moveToInitPose', 'init_waitForTirette', conditions='goal_reached') # TODO
self.sm.add_transition('init_end', 'init_waitForTirette', 'idle', conditions='tirette_released')
```
**Description**: La transition vers l'état `moveToInitPose` est commentée. Cet état permettrait au robot de se positionner automatiquement à sa pose initiale pendant la phase de préparation.

**Lié à**: TODO #4 - InitPoseState

**Action**: 
- Implémenter complètement `InitPoseState`
- Décommenter la transition
- Tester le positionnement automatique avec ArUco

---

## 📍 States - Positionnement

### 4. Implémenter InitPoseState avec détection ArUco
**Fichier**: `states.py` (lignes 16-20)  
**Priorité**: 🔴 Haute (feature désactivée)  
**Contexte**:
```python
# class InitPoseState(ChampiState): # TODO quand on voudra init la pose du robot pendant le temps de prep automatiquement
#     def enter(self, event_data):
#         super().enter(event_data)
#         # TODO attendre un tag aruco OK
#         x = self.sm.init_pose[0]
#         y = self.sm.init_pose[1]
#         theta_deg = self.sm.init_pose[2]
#         theta_rad = theta_deg * 3.14159 / 180.0
#         get_logger(self.name+'_state').info(f"Start moving to INIT pose: x={x}, y={y}, theta={theta_deg}°")
```
**Description**: État commenté qui permettrait au robot de se positionner automatiquement à sa pose initiale en utilisant les marqueurs ArUco pour la localisation précise.

**Action**:
- Décommenter et implémenter la classe
- Ajouter la détection ArUco pour confirmer la position
- Implémenter le mouvement vers la pose initiale
- Gérer les erreurs de détection/positionnement

---

### 5. Mettre à jour le world state depuis detectPlatform
**Fichier**: `states.py` (lignes 57-59)  
**Priorité**: 🟡 Moyenne  
**Contexte**:
```python
# TODO, pour l'instant cette année on utilise plus de détection interne
# mais sinon faudra mettre à jour le world state depuis ici
# et que le move sache qu'il doit se baser sur cette détection
```
**Description**: Actuellement, la détection de plateforme ne met pas à jour un "world state" centralisé. La position détectée est stockée localement mais n'est pas propagée dans un système de représentation globale de l'environnement.

**Action**:
- Créer/utiliser un world state centralisé
- Mettre à jour les positions des éléments détectés
- Permettre aux actions de mouvement d'utiliser ces positions détectées

---

### 6. Clarifier le calcul de center_platform_dist
**Fichier**: `states.py` (ligne 85)  
**Priorité**: 🟢 Basse  
**Contexte**:
```python
center_platform_dist = self.sm.itf.latest_platform_dist + half_platform_width # TODO - ??
get_logger(self.name).info(f'Distance to platform width middle is {center_platform_dist}m')
```
**Description**: Le calcul de la distance au centre de la plateforme utilise une formule qui mérite clarification. Pourquoi additionne-t-on la demi-largeur ? Le commentaire interrogatif suggère que ce calcul doit être vérifié.

**Action**: 
- Vérifier la géométrie du calcul
- Documenter clairement la formule
- Ajouter un schéma si nécessaire

---

### 7. Ré-implémenter MoveForPlatformState ou trouver une alternative
**Fichier**: `states.py` (ligne 98)  
**Priorité**: 🟡 Moyenne  
**Contexte**:
```python
# class MoveForPlatformState(MoveState): # TODO on utilise plus move for platform, donc trouver un moyen de toujours prendre en compte l'offset détecté
#     def enter(self, event_data):
#         # super().enter(event_data)
#         # here x y theta are offsets
#         x_offset = event_data.kwargs.get('x', None)
```
**Description**: L'état `MoveForPlatformState` est commenté. Il permettait de se déplacer vers une plateforme détectée en appliquant des offsets. Actuellement, il faut trouver une autre façon de prendre en compte les offsets détectés dynamiquement.

**Solution actuelle**: Le système d'offset dans le DSL permet maintenant de gérer cela au niveau de la state machine.

**Action**: 
- Documenter que cette fonctionnalité est remplacée par le système d'offset du DSL
- Supprimer le code commenté
- OU ré-implémenter si nécessaire avec la nouvelle architecture

---

## 🔧 Interface ROS - state_machine_node.py

### 8. Nettoyer les paramètres de simulation
**Fichier**: `state_machine_node.py` (ligne 66)  
**Priorité**: 🟡 Moyenne  
**Contexte**:
```python
if use_above_default_strategy_param and self.sim_param: # TODOOOOOO
    self.get_logger().warn('>> State machine in SIM mode --> loading DEFAULT strategy...')
    self.champi_sm.color = 'YELLOW'
    self.champi_sm.strategy, self.champi_sm.init_pose, self.champi_sm.home_pose = load_strategy(...)
    self.sim_user_choose_strat_and_pose() # TODO remove
```
**Description**: Le code de gestion de simulation avec des paramètres par défaut a plusieurs TODOs et semble nécessiter un nettoyage. Le "TODOOOOOO" avec plusieurs 'O' suggère une urgence ou un problème connu.

**Action**:
- Clarifier la logique de simulation vs mode réel
- Nettoyer les paramètres
- Documenter le comportement attendu

---

### 9. Retirer sim_user_choose_strat_and_pose()
**Fichiers**: `state_machine_node.py` (lignes 73, 164)  
**Priorité**: 🟢 Basse  
**Contexte**:
```python
self.sim_user_choose_strat_and_pose() # TODO remove
```
**Description**: Cette fonction de simulation pour choisir stratégie et pose devrait être retirée. Elle apparaît à deux endroits dans le code.

**Action**: 
- Évaluer si la fonction est encore utilisée
- Si oui, trouver une alternative
- Si non, la supprimer complètement

---

### 10. Améliorer la vérification ros_initialized
**Fichier**: `state_machine_node.py` (ligne 113)  
**Priorité**: 🟢 Basse  
**Contexte**:
```python
self.champi_sm.ros_initialized = True # TODO more things ?
self.itf_initialized = True
```
**Description**: La question "more things ?" suggère que l'initialisation ROS pourrait nécessiter des vérifications supplémentaires avant de marquer le système comme initialisé.

**Action**:
- Lister toutes les dépendances ROS nécessaires
- Vérifier qu'elles sont toutes initialisées
- Documenter les conditions d'initialisation complète

---

### 11. Implémenter la vérification des actuateurs terminés
**Fichier**: `state_machine_node.py` (ligne 179)  
**Priorité**: 🟡 Moyenne  
**Contexte**:
```python
def actuators_finished_callback(self, msg):
    array = msg.data # 9 elements
    self.get_logger().debug(f'Actuators finished: {array}') # TODO check for which one but should be ok without
    self.champi_sm.end_of_actuator_state = True
```
**Description**: Le callback reçoit un tableau de 9 éléments indiquant quels actuateurs ont terminé, mais actuellement on ne vérifie pas quel actuateur spécifique a terminé. On marque juste l'état comme terminé globalement.

**Action**:
- Implémenter une vérification par actuateur si nécessaire
- OU documenter pourquoi la vérification globale suffit

---

### 12. Gérer le feedback de navigation
**Fichier**: `state_machine_node.py` (ligne 265)  
**Priorité**: 🟡 Moyenne  
**Contexte**:
```python
def feedback_callback(self, feedback_msg):
    # self.get_logger().debug(f'Feedback received! path_compute_result:{...}, ETA: {round(feedback_msg.feedback.eta, 2)}s')
    # TODO prendre en compte
    pass
```
**Description**: Le feedback de l'action de navigation (ETA, résultat du calcul de chemin) est reçu mais pas traité. Il pourrait être utilisé pour afficher l'avancement ou détecter des problèmes.

**Action**:
- Décider quelles informations du feedback sont utiles
- Implémenter l'affichage/traitement de ces informations
- Gérer les cas d'erreur remontés par le feedback

---

### 13. Gérer le rejet de goal
**Fichier**: `state_machine_node.py` (ligne 274)  
**Priorité**: 🔴 Haute  
**Contexte**:
```python
def goal_response_callback(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
        self.get_logger().info('Goal rejected :(') # TODO
        return
```
**Description**: Quand un goal de navigation est rejeté, on log juste un message mais on ne gère pas l'erreur (notification à la state machine, retry, etc.).

**Action**:
- Implémenter la gestion d'erreur (cancel action, notify state machine)
- Ajouter un mécanisme de retry si pertinent
- Documenter les causes possibles de rejet

---

### 14. Compléter get_result_callback
**Fichier**: `state_machine_node.py` (ligne 285)  
**Priorité**: 🟡 Moyenne  
**Contexte**:
```python
def get_result_callback(self, future): # TODO
    result = future.result().result
    self.get_logger().info(f'Action result: {result.success}, {result.message}')
    if result.success:
        self.goal_reached_callback()
    else:
        self.get_logger().error(f'Move failed: {result.message}')
        if 'move' in self.champi_sm.state:
            ...
```
**Description**: Le callback de résultat d'action a un TODO général, suggérant qu'il pourrait être amélioré ou complété.

**Action**:
- Examiner tous les cas possibles de résultat
- S'assurer que tous sont gérés correctement
- Documenter le comportement pour chaque type de résultat

---

### 15. Remplacer le quick fix de end_speed
**Fichier**: `state_machine_node.py` (ligne 334)  
**Priorité**: 🟡 Moyenne  
**Contexte**:
```python
goal.pose = goal_pose

# goal.end_speed = end_speed
goal.end_speed = 0. # TODO quick fix

goal.max_linear_speed = motion_params.speed
```
**Description**: La vitesse de fin (`end_speed`) est forcée à 0 comme "quick fix" au lieu d'utiliser la valeur du paramètre de mouvement. Cette valeur devrait venir des `motion_params`.

**Action**:
- Utiliser `motion_params.end_speed` au lieu de forcer à 0
- Tester que cela fonctionne correctement
- Supprimer le commentaire "quick fix"

---

## 🎮 Stratégies

### 16. Utiliser le world state pour les éléments au lieu de positions
**Fichier**: `__strat_main_2025.py` (ligne 18)  
**Priorité**: 🔴 Haute (architecture)  
**Contexte**:
```python
# TODO il faudra que les takes et tout on donne pas une position, mais le nom de l'élément dans le world state
```
**Description**: Actuellement, les actions de prise/dépose d'éléments utilisent des positions absolues. Il faudrait utiliser un système de world state où on référence les éléments par leur nom/ID plutôt que par position.

**Exemple actuel**:
```python
.take_elements_sequence(1.1, 0.95, 0.0, "elements_1")
```

**Exemple souhaité**:
```python
.take_element("platform_1")
.put_element("construction_zone_1")
```

**Action**:
- Concevoir un système de world state avec éléments nommés
- Modifier le DSL pour accepter des références d'éléments
- Mettre à jour les stratégies pour utiliser ce système

---

## 🎨 Visualisation

### 17. Améliorer la gestion de la rotation -90°
**Fichier**: `strat_displayer_node.py` (ligne 210)  
**Priorité**: 🟢 Basse  
**Contexte**:
```python
if has_target:
    # Use target as reference position
    x, y = action.target.x, action.target.y
    theta_deg = action.target.theta_deg - 90 # TODO 90 should be better handled
```
**Description**: Le displayer applique une rotation de -90° pour l'affichage, mais cette valeur "magique" devrait être mieux gérée (constante, paramètre, documentation).

**Action**:
- Créer une constante pour cette rotation
- Documenter pourquoi cette rotation est nécessaire
- OU corriger l'architecture pour ne pas nécessiter cette rotation

---

## 📊 Résumé par Priorité

### 🔴 Haute Priorité (5)
1. #3 - Implémenter la transition vers moveToInitPose
2. #4 - Implémenter InitPoseState avec détection ArUco
3. #13 - Gérer le rejet de goal
4. #16 - Utiliser le world state pour les éléments

### 🟡 Moyenne Priorité (9)
1. #1 - Charger les paramètres de mouvement depuis la config ROS
2. #5 - Mettre à jour le world state depuis detectPlatform
3. #7 - Ré-implémenter MoveForPlatformState ou alternative
4. #8 - Nettoyer les paramètres de simulation
5. #11 - Implémenter la vérification des actuateurs
6. #12 - Gérer le feedback de navigation
7. #14 - Compléter get_result_callback
8. #15 - Remplacer le quick fix de end_speed

### 🟢 Basse Priorité (5)
1. #2 - Clarifier pourquoi stop_requested = True
2. #6 - Clarifier le calcul de center_platform_dist
3. #9 - Retirer sim_user_choose_strat_and_pose()
4. #10 - Améliorer la vérification ros_initialized
5. #17 - Améliorer la gestion de la rotation -90°

---

## 📝 Notes

- **Architecture World State**: Plusieurs TODOs (#5, #16) pointent vers le besoin d'un système de world state centralisé
- **Code de simulation**: Plusieurs TODOs (#8, #9) concernent le nettoyage du code de simulation
- **Gestion d'erreurs**: Plusieurs TODOs (#11, #12, #13, #14) concernent l'amélioration de la gestion d'erreurs

---

**Dernière mise à jour**: 2 novembre 2025
