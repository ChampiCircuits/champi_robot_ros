## Étapes de réalisation

1. **Initialisation du projet** : Création des dossiers pour le frontend web, le backend, et le firmware ESP32 de base.
2. **Développement du Frontend (Interface Web)** : 
   - Intégration de l'image de la table (3x2m) dans un Canvas interactif.
   - Outils de dessin : ajout, modification et suppression de points de passages (waypoints).
3. **Logique de Visualisation & Simulation** :
   - Ajout d'une *timeline* permettant d'animer les déplacements des PAMIs.
   - Système de détection de collisions visuelle (superposition des PAMIs ou zones de danger).
4. **Développement du Backend (API & Sauvegarde)** :
   - Création d'une API pour sauvegarder (ex: format JSON) et charger les différentes configurations de trajectoires.
5. **Générateur de code (Backend)** :
   - Script qui prend les trajectoires JSON en entrée et génère un fichier d'en-tête (ex: `generated_trajectory.h`) en C/C++ exploitable par l'ESP32.
6. **Intégration du Flashage (Backend)** :
   - Interfaçage avec le toolchain ESP32 (ex: PlatformIO CLI / esptool) pour compiler le code firmware (qui inclut le fichier généré) et le flasher en USB (ou OTA).

## Structure des fichiers et du code

```text
esp32_PAMIs/
├── PLAN.md
├── firmware/                        # Le projet embarqué commun
│   ├── platformio.ini               # Configuration PlatformIO pour ESP32
│   ├── include/
│   │   └── generated_trajectory.h   # Fichier QUI SERA GÉNÉRÉ par l'interface web
│   └── src/
│       ├── main.cpp                 # Code principal (identique pour tous les PAMIs)
│       └── motion.cpp               # Logique d'asservissement et de suivi de trajectoire
└── pami_web_configurator/           # L'outil de configuration
    ├── backend/                     # Serveur (ex: Python FastAPI ou Node.js)
    │   ├── main.py                  # API REST
    │   ├── builder.py               # Appels système (Subprocess) vers PlatformIO pour compiler/flasher
    │   ├── generator.py             # Traduction des trajets JSON -> generated_trajectory.h
    │   └── data/                    # Sauvegarde des configurations et projets
    └── frontend/                    # Application UI (ex: React, Vue, ou Vanilla JS)
        ├── public/
        │   └── table_map.png        # L'image 3x2m de la table
        └── src/
            ├── components/
            │   ├── TableCanvas.js   # Rendu 2D, gestion des clics (waypoints)
            │   ├── Timeline.js      # Contrôle de l'animation de la simulation
            │   └── ConfigPanel.js   # Formulaires de configuration (vitesse, paramètres PAMI)
            └── App.js
```
