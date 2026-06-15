# M2614 - La face cachée de la lune

Ce dépôt contient le firmware principal du robot M2614 pour l'Arduino Uno Q. Il pilote la base mécanum, lit la télécommande RC, gère les capteurs de navigation, commande l'activation du trieur de balles et peut aussi servir de point d'entrée pour la calibration du capteur couleur AS7341.

Cette documentation s'adresse à une personne qui connaît déjà l'écosystème Arduino, mais qui n'a jamais utilisé l'Uno Q. Le point important à retenir est que l'Uno Q combine un microcontrôleur temps réel et un environnement Linux embarqué. Dans ce projet, le robot est donc réparti en trois sous-systèmes :

- l'Uno Q côté MCU pour le temps réel et le pilotage du robot
- l'Uno Q côté SBC Linux pour l'interface web, les fichiers de calibration et les calculs plus lourds
- une Seeeduino Nano dédiée au trieur de balles

## Ce que fait ce dépôt

Le firmware principal gère les fonctions suivantes :

- lecture de la télécommande RC 8 canaux
- pilotage des quatre roues mécanum
- lecture des encodeurs de vitesse
- intégration des capteurs ToF et du LiDAR
- logique de modes manuel et automatique
- commande GPIO du trieur de balles
- bascule conditionnelle en mode calibration couleur si le capteur AS7341 est détecté au démarrage

## Différence importante avec une Uno R4, une AVR ou une ESP32

L'Uno Q n'est pas seulement une carte Arduino classique :

- le sketch est compilé pour la plateforme `arduino:zephyr:unoq`
- le microcontrôleur et le Linux embarqué communiquent via RouterBridge
- les traitements temps réel doivent rester côté MCU
- les traitements lents, les interfaces web et les fichiers doivent rester côté SBC Linux

En pratique, il ne faut pas concevoir l'Uno Q comme une simple Uno R4 avec plus de puissance. La communication RouterBridge a une latence sensible et ne convient pas aux boucles de contrôle rapides. Elle est adaptée aux commandes, à la configuration et à une télémétrie peu fréquente.

## Architecture du système

### 1. Uno Q côté MCU

Le MCU exécute le sketch Arduino de ce dépôt. Il prend en charge :

- la lecture des joysticks et interrupteurs de la télécommande
- le mixage mécanum et la commande moteur
- la lecture des encodeurs et la régulation associée
- les capteurs de distance pour la navigation
- la logique de mode manuel ou automatique
- la sortie `ENABLE_SORTER` qui active ou désactive le trieur

### 2. Uno Q côté SBC Linux

Le SBC Linux héberge l'application Python du dépôt `M2614_LaFaceCacheeDeLaLune-Python`. Il fournit :

- l'interface web de capture des échantillons couleur
- le stockage CSV des mesures AS7341
- l'analyse des centroïdes de calibration
- le redémarrage logiciel du MCU

### 3. Seeeduino Nano du trieur

Le trieur est documenté dans le dépôt `ball-sorter`. Il gère :

- le moteur du mécanisme de tri
- le servo de déviation
- les capteurs ToF locaux
- le capteur couleur AS7341 pour la classification embarquée
- les NeoPixels de retour visuel

## Prérequis

Pour compiler et téléverser ce dépôt, il faut au minimum :

- `arduino-cli`
- le coeur Arduino Uno Q avec la carte `arduino:zephyr:unoq`
- un accès USB à la carte ou un accès réseau si vous utilisez le téléversement WiFi

Pour utiliser le robot complet, il faut également :

- la télécommande RC correctement appairée
- le trieur `ball-sorter` déjà chargé sur la Seeeduino Nano
- le service Python du dépôt `M2614_LaFaceCacheeDeLaLune-Python` si vous voulez recalibrer le capteur couleur

## Compilation et téléversement

### Compilation locale

Commande minimale :

```powershell
arduino-cli compile -b arduino:zephyr:unoq --build-path .build
```

Dans cet atelier, la tâche VS Code utilise aussi `arduino:zephyr:unoq` avec un dossier de build temporaire.

### Téléversement USB

Exemple de commande :

```powershell
arduino-cli upload -p COM9 -b arduino:zephyr:unoq --input-dir $env:TEMP\unoq-build\
```

`COM9` correspond à la configuration actuelle de l'atelier. Sur un autre poste, il faut vérifier le port série réel avant le téléversement.

### Téléversement WiFi

Le dépôt est aussi prévu pour un téléversement via `remoteocd` sur le Linux embarqué de l'Uno Q. Dans la configuration actuelle :

- adresse IP : `10.206.61.29`
- mot de passe : `M2614`

Cette configuration est propre au robot de l'atelier. Si l'adresse change, il faudra adapter la commande ou la tâche VS Code correspondante.

## Mise en service rapide

1. Vérifier que le sketch de ce dépôt est chargé sur l'Uno Q.
2. Vérifier que le firmware `ball-sorter` est chargé sur la Seeeduino Nano.
3. Alimenter le robot et la télécommande.
4. Confirmer que le capteur couleur AS7341 n'est branché au boot que si vous voulez démarrer en mode calibration.
5. Vérifier le comportement des canaux RC avant tout essai de déplacement.

## Modes de fonctionnement

### Mode robot normal

Si le capteur AS7341 de calibration n'est pas détecté au démarrage, le robot exécute son comportement normal :

- lecture RC
- pilotage manuel
- logique automatique
- commande du trieur via GPIO

### Mode calibration couleur

Si le capteur AS7341 est détecté au démarrage sur `Wire1`, le sketch entre en mode calibration uniquement. Dans ce mode :

- le robot ne suit pas son comportement de navigation normal
- le MCU expose des RPC RouterBridge pour la capture couleur
- les échantillons sont envoyés vers le service Python côté SBC

Cette bascule est volontaire. Elle évite d'entretenir deux sketches différents pour l'exploitation et la calibration.

## Télécommande et commandes opérateur

Le robot utilise une télécommande 8 canaux. Le mapping exact est documenté dans l'API du pilote RC, mais l'usage pratique est le suivant :

- canaux A à D : commandes principales de déplacement et rotation
- canal E : participation au changement de mode
- canal F : commande du trieur en mode manuel
- canaux G et H : trims et réglages auxiliaires

### Activation du trieur

Le trieur n'est pas piloté ici par une liaison de haut niveau. Ce dépôt commande simplement une sortie numérique `ENABLE_SORTER` :

- en mode manuel, le canal F bascule l'état du trieur sur flanc descendant
- en mode automatique, la combinaison `E + F` force le passage en mode automatique et coupe le trieur

## Recalibrage du capteur couleur

Le recalibrage complet fait intervenir trois dépôts :

1. ce dépôt capture les mesures AS7341 côté MCU
2. `M2614_LaFaceCacheeDeLaLune-Python` reçoit les échantillons, les étiquette et lance l'analyse
3. `ball-sorter` intègre ensuite les nouveaux centroïdes dans son classifieur embarqué

Le workflow nominal est le suivant :

1. brancher le capteur AS7341 de calibration
2. redémarrer l'Uno Q pour entrer en mode calibration
3. lancer le service Python sur le SBC
4. ouvrir l'interface web de capture
5. enregistrer des échantillons pour chaque couleur
6. lancer l'analyse des centroïdes depuis l'interface
7. recopier le tableau C++ généré dans le dépôt `ball-sorter`
8. recompiler et téléverser le trieur

Le notebook `ball-analyzer` reste un outil de secours et d'analyse hors ligne. Ce n'est plus le workflow principal pour l'utilisateur final.

## Limitations et points de vigilance

- RouterBridge n'est pas prévu pour des boucles rapides ou des échanges à haute fréquence.
- Le mode calibration dépend de la présence de l'AS7341 au démarrage du sketch.
- Les valeurs `COM9` et `10.206.61.29` sont celles de la configuration actuelle, pas des constantes universelles.
- Si vous modifiez les gains, les seuils ou le matériel de propulsion, il faudra revérifier le comportement dynamique du robot.

## Dépannage rapide

### Le sketch compile mal sur Uno Q

- vérifier que la bonne cible est `arduino:zephyr:unoq`
- vérifier que le coeur Uno Q est correctement installé
- si besoin, refaire le test de compilation sous WSL/Ubuntu avant de conclure à un problème applicatif

### Le robot ne démarre pas en mode normal

- vérifier si l'AS7341 de calibration est branché
- si oui, le sketch peut être entré volontairement en mode calibration uniquement

### Le trieur ne répond pas

- vérifier que la Seeeduino Nano exécute bien le firmware `ball-sorter`
- vérifier la ligne `ENABLE_SORTER`
- vérifier le comportement du canal F en mode manuel

### La calibration web ne reçoit aucun échantillon

- vérifier que le service Python est démarré sur le SBC
- vérifier que RouterBridge fonctionne côté Uno Q
- vérifier que le capteur AS7341 a bien été détecté au boot

## Documentation associée

- `docs/document-transmission.md` : document final de transmission et tutoriel d'utilisation global
- `docs/architecture.md` : architecture détaillée et contraintes système
- `docs/operation-guide.md` : utilisation courante du robot
- `docs/recalibration-guide.md` : procédure complète de recalibrage couleur
- `docs/development-workflow.md` : workflow de build, de validation et de recalibrage
- `ball-sorter/README.md` : documentation utilisateur du trieur
- `M2614_LaFaceCacheeDeLaLune-Python/README.md` : documentation utilisateur du service SBC

## Référence API

La documentation Doxygen de ce dépôt reprend à la fois les pages Markdown et les API documentées dans les headers du projet, notamment :

- le pilote RC
- le pilote mécanum
- les encodeurs
- les capteurs ToF et LiDAR
- le service de calibration couleur
