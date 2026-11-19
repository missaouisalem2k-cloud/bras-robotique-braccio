# Bras robotique Braccio — simulation MATLAB et commande Arduino

Simulation de la cinématique directe d'un bras robotique à 5 articulations, et commande d'un bras réel **Arduino Braccio** sur une trajectoire de « pick-and-place » (attraper un objet et le déposer ailleurs).

## Contenu

| Dossier / fichier | Rôle |
|---|---|
| `matlab/pick_and_place.m` | Simulation 3D animée : cinématique directe par matrices homogènes, tracé de la trajectoire de l'effecteur |
| `matlab/drawArm.mlx` | Tracé 2D du bras à partir des angles des servomoteurs |
| `matlab/moveBraccio.mlx` | Affiche une position (6 angles) et dessine le bras correspondant |
| `matlab/executeTrajectory.mlx` | Enchaîne une liste de positions pour simuler une trajectoire |
| `arduino/essaier/essaier.ino` | Programme Arduino : le bras Braccio exécute en boucle une trajectoire de 6 positions |

## Méthode

1. **Modèle géométrique** : chaque articulation est décrite par une matrice de transformation homogène 4×4 ; leur produit donne la position de chaque segment et de l'outil.
2. **Trajectoire** : les angles varient progressivement entre les positions clés (départ → rotation → saisie → rotation → dépôt → retour), avec animation à chaque pas.
3. **Commande réelle** : la même séquence de positions est envoyée aux 6 servomoteurs du Braccio avec la bibliothèque `Braccio.h`.

## Lancer

- **MATLAB** : ouvrir le dossier `matlab/` et exécuter `pick_and_place`.
- **Arduino** : ouvrir `arduino/essaier/essaier.ino` dans l'IDE Arduino (bibliothèque *Braccio* installée), puis téléverser sur la carte.

## Compétences

Cinématique directe · matrices homogènes · génération de trajectoire · MATLAB · Arduino / C++ · servomoteurs

---
Projet académique — Salem Missaoui, Master ISC parcours VISTA, Université de Toulon.
