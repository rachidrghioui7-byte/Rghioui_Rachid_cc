# Système de Collision pour Essaim Autonome (UAV)

Projet Industriel - Programmation Avancée en C
École des Sciences de l'Information | Pr. Tarik HOUICHIME

---

## Description

Module de sécurité pour 10 000 micro-drones autonomes.
Détection instantanée des deux drones les plus proches.
Algorithme Diviser pour Régner - Complexité O(n log² n).

---

## Structure du dépôt

- `src/swarm_collision.c` : Code source industriel (Livrable 1)
- `docs/conception_technique.pdf` : Dossier de conception technique (Livrable 2)
- `docs/audit_asymptotique.pdf` : Audit asymptotique et preuve (Livrable 3)

---

## Compilation

gcc -std=c99 -Wall -Wextra -O2 -o swarm_collision src/swarm_collision.c -lm

## Exécution

./swarm_collision

---

## Auteur

Rghioui Rachid
Date : 01/05/2026
