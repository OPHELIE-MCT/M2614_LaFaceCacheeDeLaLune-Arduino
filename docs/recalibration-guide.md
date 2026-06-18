# Guide de recalibrage couleur

Cette page décrit le workflow nominal pour recalibrer le tri couleur du projet M2614 sans notebook externe obligatoire.

## Objectif

Le recalibrage sert à produire de nouveaux centroïdes pour le classifieur embarqué du dépôt `ball-sorter`. Il est nécessaire quand :

- l'éclairage change fortement
- le capteur AS7341 ou l'optique ont été modifiés
- les balles réelles se comportent différemment du jeu de référence actuel
- la qualité du tri devient insuffisante

## Sous-systèmes impliqués

Le recalibrage fait intervenir trois dépôts :

1. `M2614_LaFaceCacheeDeLaLune` pour la capture côté MCU
2. `M2614_LaFaceCacheeDeLaLune-Python` pour l'interface web, le CSV et l'analyse
3. `ball-sorter` pour l'intégration finale des nouveaux centroïdes

## Préparation

Avant de commencer :

- brancher le capteur AS7341 de calibration sur l'Uno Q
- vérifier que le service Python est disponible sur le SBC Linux
- vérifier que RouterBridge fonctionne correctement
- préparer un échantillonnage propre pour chaque couleur à capturer

## Entrée en mode calibration

Le sketch principal tente d'initialiser l'AS7341 au démarrage. Si le capteur est présent :

- le MCU entre en mode calibration uniquement
- les RPC `color_sensor.capture.start` et `color_sensor.capture.stop` deviennent utilisables
- les échantillons sont émis avec `color_sensor.sample`

Si le capteur n'est pas détecté, le robot revient à son comportement normal et aucune capture ne pourra être réalisée.

## Capture des échantillons

Depuis l'interface web du service Python :

1. choisir la couleur à enregistrer
2. lancer la capture
3. présenter les balles correspondantes devant le capteur
4. attendre l'atteinte de `100` échantillons ou arrêter manuellement
5. répéter l'opération pour toutes les classes utiles

Le fichier produit est `data/color_sensor_samples.csv`.

## Analyse locale

Une fois le CSV complet, lancer l'analyse depuis l'interface. Le service Python :

- calcule les centroïdes
- calcule un rayon interne global de confiance maximale au 95e percentile
- calcule un rayon externe par classe comme la moitie de la distance au centroïde voisin le plus proche
- génère des graphiques de contrôle
- sauvegarde le résultat dans `data/analysis/last_centroid_analysis.json`
- affiche le tableau C++ final à recopier

## Réinjection dans le trieur

Le bloc C++ documenté produit par l'analyse doit être reporté dans `ball-sorter/config.h`.
Ce fichier est dédié aux constantes générées : il peut être vidé puis remplacé par un simple copier-coller du nouveau bloc. Ensuite :

1. recompiler `ball-sorter`
2. téléverser le firmware sur la Seeeduino Nano
3. vérifier le comportement réel du tri sur un jeu de test

## Vérification après recalibrage

Après mise à jour du trieur, vérifier :

- que chaque couleur connue est bien reconnue
- que le comportement sur les balles rouges reste conforme
- que le nombre de faux positifs est acceptable
- que les valeurs de confiance semblent cohérentes avec la décroissance linéaire entre rayon interne et rayon externe sur l'affichage NeoPixel

## Cas d'échec fréquents

### Le service Python ne reçoit pas d'échantillons

- vérifier que l'Uno Q est bien en mode calibration
- vérifier la connexion RouterBridge
- vérifier que le capteur AS7341 est détecté au boot

### L'analyse refuse le CSV

- vérifier le format du fichier
- vérifier que plusieurs classes ont bien été capturées
- vérifier que les captures ne sont pas mélangées ou mal étiquetées

### Le tri reste mauvais après mise à jour

- vérifier la qualité des échantillons d'origine
- vérifier les conditions d'éclairage
- vérifier que le bon bloc de constantes a été copié dans `config.h`
