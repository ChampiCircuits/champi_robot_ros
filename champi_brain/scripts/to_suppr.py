import numpy as np
import matplotlib.pyplot as plt
from sklearn import svm

def filter_coords(dist, coords, research_zone_center):
    # on garde les coords qui sont à une distance inférieure à dist de la zone de recherche
    new_coords = []
    filtered_out = []
    for coord in coords:
        if np.sqrt((coord[0] - research_zone_center[0])**2 + (coord[1] - research_zone_center[1])**2) < dist:
            new_coords.append(coord)
        else:
            filtered_out.append(coord)
    return new_coords, filtered_out

def plan_trajectory(coords, coord_robot, research_zone_center):
    coords = np.array(coords)
    coord_robot = np.array(coord_robot)

    coords, filtered_out = filter_coords(dist=20, coords=coords, research_zone_center=research_zone_center)

    # Calculer le milieu des six points
    midpoint = np.mean(coords, axis=0)

    # Calculer l'angle entre le robot et le milieu des points
    angle_robot_midpoint = np.arctan2(midpoint[1] - coord_robot[1], midpoint[0] - coord_robot[0])
    
    best_angle_dist = 100000 # best angle c'est l'angle qui est le plus proche de l'angle entre le robot et le milieu des points
    best_angle = 0
    found = False
    for i in range(-180,180):
        # on calcule la droite à l'angle i
        angle = np.radians(i)
        slope = np.tan(angle)
        intercept = midpoint[1] - slope * midpoint[0]

        # on regarde combien de points sont au dessus et en dessous de la droite
        above = []
        below = []
        for coord in coords:
            if coord[1] > slope * coord[0] + intercept:
                above.append(coord)
            else:
                below.append(coord)

        # on regarde si il y a bien trois points de chaque côté
        nb_to_get_each_side = len(coords) // 2
        print("nb to get each side: ", nb_to_get_each_side)
        if len(above) >= nb_to_get_each_side and len(below) >= nb_to_get_each_side:
            # on regarde si c'est le meilleur angle
            # print("angle: ", np.degrees(angle), "nb above/below: ", len(above), len(below),"angle", abs(angle - angle_robot_midpoint))
            if abs(angle - angle_robot_midpoint) < best_angle_dist:
                best_angle = angle
                # print("\n\n####MEILLEUR ANGLE: ", np.degrees(best_angle))
                found = True
                best_angle_dist = abs(angle - angle_robot_midpoint)

    if not found:
        print("Pas d'angle trouvé")
        quit()
    
    # on calcule la droite à l'angle best_angle
    slope = np.tan(best_angle)
    intercept = midpoint[1] - slope * midpoint[0]

    # on regarde combien de points sont au dessus et en dessous de la droite
    group1 = []
    group2 = []
    for coord in coords:
        if coord[1] > slope * coord[0] + intercept:
            group1.append(coord)
        else:
            group2.append(coord)

    group1 = np.array(group1)
    group2 = np.array(group2)

      
    # SVM pour maximiser les marges entre les deux groupes
    # On utilise un modèle linéaire car on a une droite de séparation
    X = np.vstack([group1, group2])
    y = np.array([0] * len(group1) + [1] * len(group2))
    clf = svm.SVC(kernel='linear')
    clf.fit(X, y)
    slope2 = -clf.coef_[0][0] / clf.coef_[0][1]
    intercept2 = -clf.intercept_ / clf.coef_[0][1]
    intercept2 = intercept2[0]

       

    return group1, group2, slope, intercept, slope2, intercept2, filtered_out

def plot_groups_and_line(group1, group2, slope, intercept, coord_robot, slope2, intercept2, start_proj, end_proj, start, end,filtered_out):
    all_points = np.vstack([group1, group2, np.array([coord_robot]), filtered_out, np.array([start])])

    # Déterminer les limites pour centrer la fenêtre
    buffer = 5  # Marge ajoutée pour s'assurer que tous les points sont clairement visibles
    x_min, y_min = np.min(all_points, axis=0) - buffer
    x_max, y_max = np.max(all_points, axis=0) + buffer

    plt.xlim(x_min, x_max)
    plt.ylim(y_min, y_max)
    plt.axis('square')
    # noir, cercle
    plt.scatter(start[0], start[1], color='black', label='Start', marker='o', s=150)
    plt.scatter(end[0], end[1], color='black', label='End', marker='o', s=150)

    plt.scatter(start_proj[0], start_proj[1], color='black', label='Start proj', marker='x',s=100)
    plt.scatter(end_proj[0], end_proj[1], color='black', label='End proj', marker='x',s=100)


    # Placer les points de chaque groupe
    plt.scatter(group1[:, 0], group1[:, 1], color='red', label='Groupe 1')
    plt.scatter(group2[:, 0], group2[:, 1], color='blue', label='Groupe 2')
    plt.scatter(coord_robot[0], coord_robot[1], color='green', s=1000)

    plt.scatter(np.array(filtered_out)[:, 0], np.array(filtered_out)[:, 1], color='grey', label='Filtered out', marker='x', s=100)


    # Calculer les x et y pour la ligne de séparation
    x = np.linspace(min(np.min(group1[:, 0]), np.min(group2[:, 0])), max(np.max(group1[:, 0]), np.max(group2[:, 0])), 10)
    print(min(np.min(group1[:, 0]), np.min(group2[:, 0])))
    print(max(np.max(group1[:, 0]), np.max(group2[:, 0])))
    print(intercept)
    y = slope * x + intercept

    # Ajouter la ligne au plot
    plt.plot(x, y, 'k--', color="grey", label='Ligne de séparation')

    # Calculer les x et y pour la ligne de séparation 2
    x = np.linspace(min(np.min(group1[:, 0]), np.min(group2[:, 0])), max(np.max(group1[:, 0]), np.max(group2[:, 0])), 10)
    y = slope2 * x + intercept2

    # Ajouter la ligne au plot
    plt.plot(x, y, 'k-', label='SVM')


    #segment
    plt.plot([start_proj[0], end_proj[0]], [start_proj[1], end_proj[1]], color='yellow', label='Trajectory')
    

    # Ajouter des détails au graphique
    plt.title('viz')
    plt.xlabel('Coordonnée X')
    plt.ylabel('Coordonnée Y')
    plt.legend()
    plt.grid(True)
    plt.show()

def project_point_on_line(x1, y1, slope, intercept):
    m, b = slope, intercept  # Pente et ordonnée à l'origine de la droite
    print("m: ", m, "b: ", b)
    print("x1: ", x1, "y1: ", y1)
    point = np.array([x1, y1])
    # Calcul du vecteur directeur de la droite
    vecteur_directeur = np.array([1, m])
    
    # Calcul du vecteur de la droite
    vecteur_droite = np.array([0, b])

    # Calcul de la projection orthogonale du point sur la droite
    projection = vecteur_droite + np.dot(point - vecteur_droite, vecteur_directeur) / np.dot(vecteur_directeur, vecteur_directeur) * vecteur_directeur
    print("Projection du point sur la droite: ", projection)
    # return np.asarray([18,28.65])
    return projection


def generate_start_end_points(group1, group2, slope2, intercept2):
    # start c'est la projection le point le plus proche du robot sur la droite de séparation
    # end c'est la projection du point le plus loin du robot sur la droite de séparation

    start = [0,0]
    end = [0,0]

    max_dist = 0
    min_dist = 100000
    all_points = np.vstack([group1, group2])

    for point in all_points:
        # dist entre point et robot
        dist = np.sqrt((point[0] - coord_robot[0])**2 + (point[1] - coord_robot[1])**2)
        if dist > max_dist:
            max_dist = dist
            end = point
        if dist < min_dist:
            min_dist = dist
            start = point

    # projection des points sur la droite de séparation
    start_proj = project_point_on_line(start[0], start[1], slope2, intercept2)
    end_proj = project_point_on_line(end[0], end[1], slope2, intercept2)

    # on a besoin de décaler le start en direction du robot d'une distance d
    # on a besoin de décaler le end en direction inverse du robot d'une distance d
    d = 5
    # vecteur dir de la droite
    n = np.array([1, slope2])
    n = n / np.linalg.norm(n)
    # pour savoir si on doit ajouter ou soustraire d a start
    if np.dot(n, start_proj - coord_robot) > 0:
        start_proj = start_proj - d * n
    else:
        start_proj = start_proj + d * n
    
    # pour savoir si on doit ajouter ou soustraire d a end
    if np.dot(n, end_proj - coord_robot) > 0:
        end_proj = end_proj + d * n
    else:
        end_proj = end_proj - d * n
    
    return start_proj, end_proj, start, end

def generate_plants_coords(n, center_x, center_y):
    # on génère des coords sur un cercle de rayon 10 autour d'un point random
    # on ajoute du bruit sur les coords
    # les n angles sont répartis uniformément + un peu de bruit

    centre_plantes = [center_x, center_y]
    centre_plantes += np.random.uniform(-10, 10, 2)

    coords = []
    for i in range(n):
        angle = 2 * np.pi * i / n + np.random.uniform(-0.2, 0.2)
        r = np.random.uniform(0, 1) + 5
        x = centre_plantes[0] + r * np.cos(angle)
        y = centre_plantes[1] + r * np.sin(angle)
        coords.append([x,y])
    return coords

def add_away_coords(coords, n, center_x, center_y):
    # on génère d'autres detections de plantes, mais plus loin que les coords

    centres_plantes = [center_x+np.random.uniform(0, 1), center_y-np.random.uniform(40, 45)]
    coords_away = []
    for i in range(n):
        angle = 2 * np.pi * i / n + np.random.uniform(-0.2, 0.2)
        r = np.random.uniform(10, 20)
        x = centres_plantes[0] + r * np.cos(angle)
        y = centres_plantes[1] + r * np.sin(angle)
        coords_away.append([x,y])

    return coords + coords_away


# Exemple d'utilisation:
# coords = [[12, 10], [10, 20], [11, 30], [26, 9], [27, 21], [25, 29]]
research_zone_center = [0,0]
coords = generate_plants_coords(6,research_zone_center[0],research_zone_center[1])
coords = add_away_coords(coords,6,research_zone_center[0],research_zone_center[1])
# coord_robot = [10,30]
# le robot est quelque part sur un carré de rayon 30 autour de 0,0
coord_robot = [0,0]
coord_robot += np.random.uniform(-30, 30, 2)

group1, group2, slope, intercept, slope2, intercept2,filtered_out = plan_trajectory(coords, coord_robot, research_zone_center)
start_proj, end_proj, start, end = generate_start_end_points(group1, group2, slope2, intercept2)
plot_groups_and_line(group1, group2, slope, intercept, coord_robot, slope2, intercept2, start_proj, end_proj, start, end,filtered_out)
