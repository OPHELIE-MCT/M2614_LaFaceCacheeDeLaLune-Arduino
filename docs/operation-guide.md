# Guide d'exploitation du robot

Cette page décrit l'utilisation courante du robot M2614 une fois les firmwares et services déjà en place.

## Avant le démarrage

Vérifier les points suivants :

- le firmware principal est chargé sur l'Uno Q
- le firmware `ball-sorter` est chargé sur la Seeeduino Nano
- la télécommande RC est alimentée et appairée
- le service Python SBC est prêt si une session de calibration doit être réalisée
- le capteur AS7341 de calibration n'est branché au boot que si l'on souhaite entrer en mode calibration

## Démarrage normal

En mode normal, le robot utilise le sketch principal pour :

- lire les entrées RC
- piloter la base mécanum
- interpréter les capteurs de navigation
- activer ou désactiver le trieur via la sortie `ENABLE_SORTER`

Si le capteur couleur de calibration n'est pas détecté au démarrage, le robot reste dans ce mode de fonctionnement normal.

## Commandes opérateur

Le détail exact des canaux est documenté dans l'API de `RemoteController`, mais le comportement attendu est :

- A à D : commande de déplacement et rotation
- E : participation au changement de mode
- F : commande du trieur en mode manuel
- G et H : trims auxiliaires

## Mode manuel

Le mode manuel est utilisé pour :

- déplacer le robot avec la télécommande
- vérifier le comportement du train mécanum
- commander le trieur de façon simple

Dans ce mode, le bouton F agit comme un basculement de l'état du trieur sur flanc descendant.

## Mode automatique

Le passage en mode automatique se fait par la combinaison `E + F`. Lors de ce passage :

- l'Uno Q désactive le trieur
- la machine d'état automatique reprend la main
- les décisions de navigation exploitent les capteurs de distance et la logique temps réel du MCU

## Trieur de balles

Le firmware principal n'effectue pas lui-même la classification couleur. Il active ou coupe seulement le sous-système de tri.

Le tri proprement dit est réalisé par le dépôt `ball-sorter`, qui gère :

- l'avance locale des balles
- la lecture AS7341
- la classification embarquée
- la déviation servo selon la couleur

## Bonnes pratiques d'utilisation

- ne pas lancer une calibration couleur pendant un usage normal du robot
- ne pas supposer que RouterBridge peut remplacer une logique temps réel
- vérifier la réponse du trieur en mode manuel avant toute séquence automatique
- revalider les réglages si le matériel mécanique ou les capteurs changent

## Symptômes courants

### Le robot ne réagit pas comme attendu

- vérifier que le sketch n'est pas entré en mode calibration au démarrage
- vérifier la validité des canaux RC
- vérifier que le trieur n'est pas forcé dans un état inattendu

### Le mode automatique ne s'active pas

- vérifier l'état des canaux E et F
- vérifier la logique d'entrée RC dans les diagnostics disponibles

### Le trieur semble inactif

- vérifier l'action du canal F en mode manuel
- vérifier le firmware `ball-sorter`
- vérifier le câblage de `ENABLE_SORTER`
