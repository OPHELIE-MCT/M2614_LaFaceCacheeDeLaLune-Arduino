# Workflow de developpement

## Structure actuelle

Le projet n'utilise plus la structure Arduino App Lab sous `D:\ArduinoApps\...`.
Le code actif est maintenant reparti dans plusieurs depots sous `C:\Users\david\OneDrive - Education Vaud\M2614 La face cachée de la lune\10_Informatique\Code\` :

- `M2614_LaFaceCacheeDeLaLune/` : firmware Arduino, sketches Uno Q et integration MCU
- `M2614_LaFaceCacheeDeLaLune-Python/` : application Python cote SBC pour RouterBridge et interfaces web
- `ball-sorter/` : firmware du sous-systeme de tri sur Seeeduino Nano
- `ball-analyzer/` : notebook et outils d'analyse pour recalibrer le capteur couleur

## Points de depart recommandes

| Besoin | Point de depart recommande | Notes |
| --- | --- | --- |
| Capture couleur Uno Q + RouterBridge | `M2614_LaFaceCacheeDeLaLune/color-data-gather/` | sketch autonome Uno Q qui lit l'AS7341 sur `Wire1` et envoie les echantillons via `Bridge.notify(...)` |
| Application SBC pour la capture | `M2614_LaFaceCacheeDeLaLune-Python/` | app FastAPI qui demarre la collecte, recoit `color_sensor.sample`, et ecrit le CSV |
| Tri des balles et mapping des 10 features | `ball-sorter/ball-sorter.ino` et `ball-sorter/config.h` | source de vérité pour l'ordre des canaux projetés, la liste des couleurs et les constantes générées |
| Notebook de recalibrage | `ball-analyzer/analysis.ipynb` | charge le CSV labellise et imprime le tableau C final |
| Telecommande RC | `Tests/RC_Reciever/RC-Controller/` | lecture des canaux, normalisation et mapping operateur |
| Pilotage mecanum | `Tests/RC_Reciever/Mecanum-Controller/` et `Tests/MechanumTest/` | logique de conduite et securites moteur |
| LiDAR | `Tests/LiDAR/` | integration du LD19 |
| SPI Uno Q / Seeeduino | `Tests/SPI/` | reference pour checksum, magic byte et reinitialisation |

## Conventions Uno Q a respecter

### Debug et logs

- ne pas utiliser `Serial.println(...)` dans le code RouterBridge
- utiliser `Monitor.println(...)`
- appeler `Monitor.begin()` sans baudrate

### Repartition MCU / SBC

- le MCU gere le temps reel et les acces materiels
- le SBC Python gere les interfaces, les fichiers CSV, les visualisations et les calculs plus lourds
- tout traitement non temps reel doit rester cote Python tant que possible

### RouterBridge

- exposer des RPC simples et explicites cote MCU
- preferer `notify` pour la telemetrie fire-and-forget a faible frequence
- documenter les payloads et l'ordre exact des champs lorsqu'un flux traverse MCU et SBC
- utiliser la [documentation officielle](https://docs.arduino.cc/tutorials/uno-q/routerbridge-multilanguage/) en cas de doute sur `Bridge.provide(...)`, `Bridge.call(...)` et `Bridge.notify(...)`

### Commande GPIO du trieur

- le bouton RC F bascule l'etat du trieur sur flanc descendant en mode manuel
- la combinaison `E + F` bascule vers le mode automatique et desactive le trieur
- l'Uno Q ecrit directement sur `ENABLE_SORTER` (GPIO)
- voir `M2614_LaFaceCacheeDeLaLune.ino` lignes ~470-490 et `PINS.h` ligne ~84

## Workflow de recalibrage du capteur couleur

Le recalibrage se fait maintenant depuis le sketch principal et l'interface web SBC, sans changer de branche ou de service pour le workflow nominal.

1. au boot du sketch principal, l'Uno Q tente d'initialiser l'AS7341 sur `Wire1`
2. si le capteur est detecte, le MCU bascule en mode calibration uniquement
3. le SBC appelle `color_sensor.capture.start` via RouterBridge
4. le MCU envoie les 10 canaux projetés avec `Bridge.notify("color_sensor.sample", ...)`
5. l'application Python ecrit les lignes dans `data/color_sensor_samples.csv` au format `color_name,channel1..channel10`
6. depuis la meme interface web, l'operateur peut reinitialiser le CSV, le telecharger, lancer l'analyse des centroïdes, et declencher `arduino-reset`
7. le service Python sauvegarde localement les graphiques, puis affiche le bloc final avec le rayon interne global et les rayons externes par classe a recopier dans `ball-sorter/config.h` (fichier dédié, pensé pour être vidé puis remplacé par copier-coller)
8. si le capteur n'est pas detecte au boot, l'Uno Q continue son comportement robot habituel et l'interface affiche explicitement que le capteur est debranche

## Outils et compilation

### MCU Uno Q

Pour le sketch principal du depot :

```powershell
arduino-cli compile -b arduino:zephyr:unoq --build-path .build
```

Pour le sketch autonome de capture couleur :

```powershell
arduino-cli compile -b arduino:zephyr:unoq --build-path .build/color-data-gather .\color-data-gather
```

Si Windows ne suffit pas pour compiler la pile Uno Q locale, verifier le meme build sous WSL/Ubuntu avant d'incriminer le code applicatif.

Le sketch principal contient maintenant aussi le point d'entree de calibration conditionnelle. Le sketch autonome `color-data-gather/` reste une source de reference utile pour comparer un portage fidele ou isoler un probleme de calibration, mais il n'est plus cense etre necessaire pour l'usage courant.

### Validation SPI Uno Q / trieur

1. compiler `M2614_LaFaceCacheeDeLaLune/`
2. compiler `ball-sorter/`
3. verifier sur banc que `F` seul active la commande moteur distante
4. verifier que `E + F` conserve le passage en mode automatique cote Uno Q
5. verifier que l'acquittement de `sequence` et l'etat `motor running` reviennent bien du Nano vers l'Uno Q
6. verifier qu'une perte de paquets ou une coupure du lien efface la commande distante cote Nano et declenche la reinitialisation defensive cote Uno Q si necessaire

### Python SBC

```powershell
uv sync
uv run main.py
```

Le service ecoute par defaut sur `0.0.0.0:8000` et lit/ecrit `data/color_sensor_samples.csv`.

L'interface expose maintenant aussi :

- `POST /api/gather/csv/reset` — reinitialise le CSV avec confirmation de l'operateur
- `GET /api/gather/csv/download` — telecharge le CSV actuel
- `POST /api/gather/analysis/run` — lance le calcul des centroïdes côté SBC et sauvegarde les résultats
- `POST /api/gather/device/reset` — execute `arduino-reset` pour reinitialiser le MCU

Les graphiques d'analyse sont sauvegardes sous `static/generated/analysis/` et servis comme fichiers statiques consultables via des liens dans l'interface.

La derniere analyse reussie est persistee dans `data/analysis/last_centroid_analysis.json` et rechargee au demarrage du service ou lors d'un rechargement de page. L'interface affiche :

- un bouton "Centroid analysis" desactive et affichant "Analyzing..." avec un spinner pendant l'execution
- le resultat C++ avec un bouton "Copy to clipboard" pour faciliter le copier-coller dans le classifieur
- les metriques d'analyse (nombre d'echantillons, score silhouette, rayon interne et rayons externes par classe)
- les graphiques generes sous forme de liens cliquables

## Regles de travail

- commencer par lire le prototype ou le depot qui possede deja le comportement cible
- ne pas recopier des sketches complets si une petite couche d'adaptation suffit
- lorsqu'un contrat de communication change, mettre a jour la documentation dans le meme patch
- considerer les notebooks comme des outils d'analyse et de generation de constantes, pas comme le lieu final d'execution sur le robot
