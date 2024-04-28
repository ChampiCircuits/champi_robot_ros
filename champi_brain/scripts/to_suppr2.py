import itertools

MAX_SPEED = 100.0  # Vitesse maximale du robot (mm/s)
TOTAL_TIME = 110  # Temps total disponible pour réaliser des actions (s)

# Définir les actions possibles avec leurs attributs : position (en mm), temps, points
# l'aire de jeu fait 2000mm de large et 3000mm de haut
# l'origine est le coin en bas à gauche
actions = [
    {"name":"plantes1","position": (1000-300, 1500-500),    "time": 10, "points": 0},
    {"name":"plantes2","position": (1000-500, 1500),        "time": 10, "points": 0},
    # {"name":"plantes3","position": (1000-300, 1500+500),    "time": 10, "points": 0},
    {"name":"plantes4","position": (1000+300, 1500-500),    "time": 10, "points": 0},
    # {"name":"plantes6","position": (1000+500, 1500),        "time": 10, "points": 0},
    # {"name":"plantes5","position": (1000+300, 1500+500),    "time": 10, "points": 0},

    {"name":"poserplantes1","position": (450/2, 500/2),     "time": 10, "points": 3*6},
    {"name":"poserplantes2","position": (2000-450/2, 500/2),"time": 10, "points": 3*6},
    # {"name":"poserplantes3","position": (2000/2, 3000-500/2),"time": 10, "points": 3*6},


    {"name":"panneau1","position": (50, 1500-225),          "time": 10, "points": 5},
    # {"name":"panneau2","position": (50, 1500),              "time": 10, "points": 5},
    # {"name":"panneau3","position": (50, 1500+225),          "time": 10, "points": 5},

    # {"name":"panneau4","position": (50, 275),               "time": 10, "points": 5},
    # {"name":"panneau5","position": (50, 275+225),           "time": 10, "points": 5},
    # {"name":"panneau6","position": (50, 275+225+225),       "time": 10, "points": 5},

    # {"name":"panneau7","position": (50, 3000-275),          "time": 10, "points": 5},
    # {"name":"panneau8","position": (50, 3000-275-225),      "time": 10, "points": 5},
    # {"name":"panneau9","position": (50, 3000-275-225-225),  "time": 10, "points": 5},
    # Ajoutez d'autres actions si nécessaire
]

START_POS = (450/2, 500/2)  # Position de départ obligatoire au début
END_POS = (2000-450/2, 500/2)  # Position de retour obligatoire à la fin

def calculate_distance(pos1, pos2):
    """Calculer la distance euclidienne entre deux positions."""
    return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**0.5

def calculate_time(distance):
    """Calculer le temps nécessaire pour se déplacer à une certaine distance."""
    return distance / MAX_SPEED

def calculate_score(sequence, current_pos):
    """Calculer le score total d'une séquence d'actions."""
    current_time = 0
    total_points = 0

    # nb de groupes de plantes en possession
    nb_groupes = 0
    
    for action in sequence:
        # si on pose des plantes
        if action["name"].startswith("poserplantes"):
            if nb_groupes > 0:
                nb_groupes -= 1
            else: # on ne peut pas poser des plantes si on n'en a pas
                continue
        # si on ramasse des plantes
        if action["name"].startswith("plantes") and nb_groupes < 3:
            nb_groupes += 1

        time_to_reach_action = calculate_time(calculate_distance(current_pos, action["position"]))
        projected_return_time = calculate_time(calculate_distance(action["position"], END_POS))

        if current_time + action["time"] + time_to_reach_action + projected_return_time > TOTAL_TIME:
            # Si ajouter cette action dépasse le temps total, arrêter
            break
        
        current_time += action["time"] + time_to_reach_action
        total_points += action["points"]
        current_pos = action["position"]
        
    return total_points

# Générer toutes les combinaisons d'actions possibles
all_combinations = []
for r in range(1, len(actions) + 1):
    all_combinations.extend(itertools.combinations(actions, r))

# Sélectionner la séquence d'actions qui maximise le score
best_sequence = []
best_score = 0
for combination in all_combinations:
    # print("\n\n#####################")
    # str_combination = ""
    # for action in combination:
    #     str_combination += action["name"] + " "
    # print("Combinaison:", str_combination)

    score = calculate_score(list(combination), START_POS)
    if score > best_score:
        best_score = score
        best_sequence = list(combination)
    elif score == best_score:
        # si le score est le même, on prend la séquence la plus courte en temps
        time_combination = sum([action["time"] + calculate_time(calculate_distance(START_POS, action["position"])) for action in combination])
        time_best_sequence = sum([action["time"] + calculate_time(calculate_distance(START_POS, action["position"])) for action in best_sequence])
        if time_combination < time_best_sequence:
            best_score = score
            best_sequence = list(combination)


print("La séquence d'actions optimale est :")
time_in_game = 0
for action in best_sequence:
    time_in_game += action["time"]
    print("Action:", action["name"], "; Points:", action["points"], "; Temps actuel:", time_in_game,"s.")

print("Score total de la séquence :", calculate_score(best_sequence, START_POS))
print("Pour aller à la position END_POS, il faut", calculate_time(calculate_distance(best_sequence[-1]["position"], END_POS)), "secondes.")
print("Temps total de la séquence :", time_in_game + calculate_time(calculate_distance(best_sequence[-1]["position"], END_POS)))



import pandas as pd


# Convert combinations to a more readable format (e.g., list of names)
combinations_readable = []
for combination in all_combinations:
    combination_names = [action['name'] for action in combination]
    combinations_readable.append(combination_names)

# la dernire colonne est le score
combinations_readable = [([action['name'] for action in combination], calculate_score(combination, START_POS)) for combination in all_combinations]
# Sort combinations by score
combinations_readable = sorted(combinations_readable, key=lambda x: x[1], reverse=True)

# Create a DataFrame to display the combinations
df_combinations = pd.DataFrame(combinations_readable)

# Show dataframe
print(df_combinations)

# print best
print("Best sequence:")
for action in best_sequence:
    print(action["name"], end=" ")
print()

# save to csv
df_combinations.to_csv("combinations.csv", index=False)

# open csv
import os
os.system("combinations.csv")


exit()


import pygame
import sys

# Initialize pygame
pygame.init()

# Define the dimensions of the playground
playground_width = 2000  # mm, will be scaled down for display
playground_height = 3000  # mm, will be scaled down for display
scale_factor = 0.1  # Scale factor to fit the playground on the screen

# Set up the display
screen_width = int(playground_width * scale_factor)
screen_height = int(playground_height * scale_factor)
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption('Playground')

# Define colors
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
GREY = (128, 128, 128)

# Function to draw the playground
def draw_playground():
    # Fill the background
    screen.fill(WHITE)
    
    # Draw each action as a circle on the playground
    for action in actions:
        # Scale the position
        scaled_position = (int(action["position"][0] * scale_factor), 
                           3000*scale_factor-int(action["position"][1] * scale_factor))
        # pour les plantes
        if action["name"].startswith("plantes"):
            pygame.draw.circle(screen, GREEN, scaled_position, 250/2*scale_factor)
        if action["name"].startswith("panneau"):
            pygame.draw.circle(screen, GREY, scaled_position, 100/2*scale_factor)

    # start pos yellow, end pos magenta
    pygame.draw.circle(screen, (255, 255, 0), (int(START_POS[0] * scale_factor), 3000*scale_factor-int(START_POS[1] * scale_factor)), 10)
    pygame.draw.circle(screen, (255, 0, 255), (int(END_POS[0] * scale_factor), 3000*scale_factor-int(END_POS[1] * scale_factor)), 10)


    # trajectory
    current_pos = START_POS
    for action in best_sequence:
        # Scale the position
        scaled_position = (int(action["position"][0] * scale_factor), 
                           3000*scale_factor-int(action["position"][1] * scale_factor))
        pygame.draw.line(screen, (0, 0, 0), (int(current_pos[0] * scale_factor), 3000*scale_factor-int(current_pos[1] * scale_factor)), scaled_position, 2)
        current_pos = action["position"]
    # add line to end pos
    pygame.draw.line(screen, (0, 0, 0), (int(current_pos[0] * scale_factor), 3000*scale_factor-int(current_pos[1] * scale_factor)), (int(END_POS[0] * scale_factor), 3000*scale_factor-int(END_POS[1] * scale_factor)), 2)



    # Update the display
    pygame.display.flip()

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Draw the playground
    draw_playground()

    # Limit frames per second
    pygame.time.Clock().tick(60)

# Quit the game
pygame.quit()
sys.exit()
