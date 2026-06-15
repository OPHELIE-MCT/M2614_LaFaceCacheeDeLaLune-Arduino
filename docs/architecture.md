# Architecture du projet

## Objectif de cette page

Cette page complète le README général avec une vue architecturelle détaillée orientée exploitation et maintenance. Elle décrit uniquement l'état actuel du système.

## Découpage du système

Le robot M2614 est organisé en trois domaines d'exécution complémentaires.

### 1. Arduino Uno Q côté MCU

Rôle : exécution temps réel et contrôle direct du robot.

Responsabilités principales :

- lecture des entrées RC et validation des signaux
- pilotage des moteurs mécanum
- acquisition capteurs proches du temps réel (encodeurs, ToF, LiDAR)
- machine d'état de conduite (manuel/automatique)
- commande GPIO du trieur via `ENABLE_SORTER`
- publication et réception des RPC de calibration couleur en mode dédié

Contraintes :

- les boucles de contrôle rapides restent exclusivement côté MCU
- le MCU ne dépend pas d'un aller-retour RouterBridge pour sa stabilité temps réel

### 2. Uno Q côté SBC Linux (service Python)

Rôle : services applicatifs, interface utilisateur et traitement non temps réel.

Responsabilités principales :

- interface web de capture de données couleur
- persistance des données de calibration en CSV et JSON
- génération des métriques et graphiques d'analyse
- orchestration du cycle de recalibrage
- commandes de service vers le MCU (démarrage/arrêt capture, reset)

Contraintes :

- la disponibilité RouterBridge conditionne les fonctions de calibration
- le service peut rester accessible sans capteur, mais ne peut pas capturer sans MCU prêt

### 3. Seeeduino Nano (trieur)

Rôle : sous-système autonome de tri des balles.

Responsabilités principales :

- détection locale de présence de balle
- lecture AS7341 et classification embarquée
- actionnement servo + moteur du mécanisme de tri
- feedback visuel local (NeoPixels)

Contraintes :

- la qualité du tri dépend des centroïdes embarqués
- les centroïdes doivent être mis à jour après recalibrage significatif

## Interfaces entre domaines

## A. Interface MCU <-> SBC (RouterBridge)

Canal : RPC RouterBridge.

Usage :

- commandes de session de capture
- interrogation de l'état capteur
- notifications d'échantillons couleur

Contrat logique :

- le MCU expose des méthodes de capture (`color_sensor.capture.*`)
- le SBC consomme les notifications `color_sensor.sample`
- les payloads sont faibles et périodiques, pas destinés au pilotage dynamique

Limite architecturelle :

- RouterBridge a une latence non négligeable
- il ne doit jamais porter une boucle de contrôle de trajectoire ou de stabilisation

## B. Interface MCU -> Trieur (commande)

Canal : GPIO `ENABLE_SORTER`.

Usage : activation/désactivation simple du trieur.

Caractéristiques :

- commande unidirectionnelle
- pas de protocole paquet, checksum ou ACK sur cette ligne
- comportement déterminé par la logique mode manuel/automatique côté MCU

## C. Interface de maintenance (SBC -> opérateur)

Canal : interface web FastAPI.

Usage :

- démarrer/arrêter les captures
- réinitialiser/télécharger le CSV
- exécuter l'analyse locale
- récupérer le code C++ de centroïdes à reporter dans le trieur

## États fonctionnels importants

### 1. Exploitation robot

Condition : capteur de calibration non actif au boot.

Effet :

- logique de navigation complète active
- télécommande et capteurs dédiés au pilotage
- commande trieur disponible selon mode

### 2. Calibration couleur

Condition : capteur AS7341 de calibration détecté au démarrage MCU.

Effet :

- activation du chemin RouterBridge de capture
- émission des échantillons vers le SBC
- traitement de calibration centré sur la chaîne de données

Conséquence opérationnelle :

- ce mode est orienté acquisition/qualité de données, pas déplacement autonome

## Données produites et cycle de vie

### 1. Données brutes et labellisées

- fichier principal : `data/color_sensor_samples.csv`
- origine : notifications MCU + étiquette opérateur côté SBC
- usage : base d'analyse pour recalcul des centroïdes

### 2. Résultats d'analyse

- persistance : `data/analysis/last_centroid_analysis.json`
- artefacts visuels : `static/generated/analysis/`
- sortie exploitable : tableau C++ et seuil inconnu pour `ball-sorter`

### 3. Données embarquées trieur

- localisation : `ball-sorter/classification.cpp`
- contenu : centroïdes et paramètres de décision
- mise à jour : manuelle après validation du recalibrage

## Invariants d'architecture

Ces règles ne doivent pas être cassées lors des évolutions :

- le contrôle moteur et la sécurité immédiate restent côté MCU
- RouterBridge reste un bus de commande et télémétrie lente
- le SBC reste responsable des fichiers et analyses
- le trieur reste autonome pour la décision locale de tri
- le recalibrage ne modifie pas la logique temps réel du pilotage robot

## Localisation des sources techniques

- firmware principal : `M2614_LaFaceCacheeDeLaLune.ino`
- brochage et liaisons : `PINS.h`
- service de calibration MCU : `calibration/ColorCalibration.h`
- pilotes MCU : `driver/`
- LiDAR : `LiDAR/`
- service SBC : `M2614_LaFaceCacheeDeLaLune-Python/app/`
- classifieur trieur : `ball-sorter/classification.h` et `ball-sorter/classification.cpp`

## Risques opérationnels à surveiller

- indisponibilité RouterBridge pendant une session de capture
- dérive de performance du trieur si calibration obsolète
- incohérence entre seuils d'analyse SBC et constantes embarquées trieur
- confusion opérateur entre mode exploitation robot et mode calibration

Ces risques doivent être couverts par la procédure d'exploitation et de recalibrage, et vérifiés après chaque mise à jour de firmware ou de constantes de classification.
