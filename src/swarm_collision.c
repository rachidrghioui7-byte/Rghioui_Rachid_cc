/*
 ============================================================================
 Projet Industriel : Système de Collision pour Essaim Autonome (UAV)
 École des Sciences de l'Information – Programmation Avancée en C
 Auteur : Nom-Prenom
 Date : 2026-05-11
 Description : Détection de la paire de drones la plus proche parmi 10 000
               appareils. Navigation par arithmétique de pointeurs,
               sans crochets d'indexation.
 ============================================================================
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

#define N 10000               // Taille de l'essaim
#define MAX_STRIP_NEIGHBORS 32 // Nombre max de voisins examinés dans la bande

// Structure hétérogène d'un drone (cahier des charges)
struct Drone {
    int id;
    float x;
    float y;
    float z;
};

// Structure de résultat : deux drones et leur distance
struct Pair {
    struct Drone *a;
    struct Drone *b;
    float dist;
};

/* --------------------------------------------------------------------------
   Comparateurs pour qsort (tri selon x, puis selon y).
   Aucun crochet n'est utilisé, uniquement des pointeurs.
   -------------------------------------------------------------------------- */
int cmp_x(const void *p1, const void *p2) {
    const struct Drone *d1 = (const struct Drone *)p1;
    const struct Drone *d2 = (const struct Drone *)p2;
    return (d1->x > d2->x) - (d1->x < d2->x);
}

int cmp_y(const void *p1, const void *p2) {
    const struct Drone *d1 = (const struct Drone *)p1;
    const struct Drone *d2 = (const struct Drone *)p2;
    return (d1->y > d2->y) - (d1->y < d2->y);
}

/* --------------------------------------------------------------------------
   Distance euclidienne 3D entre deux drones pointés par a et b.
   -------------------------------------------------------------------------- */
float distance(const struct Drone *a, const struct Drone *b) {
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

/* --------------------------------------------------------------------------
   Recherche brute de la paire minimale dans un petit sous-ensemble.
   Utilisée quand n <= 3.
   -------------------------------------------------------------------------- */
struct Pair brute_force(struct Drone *points, int n) {
    struct Pair best;
    best.dist = FLT_MAX;
    best.a = best.b = NULL;

    struct Drone *pi, *pj;
    for (int i = 0; i < n; i++) {
        pi = points + i;               // pas de crochets
        for (int j = i + 1; j < n; j++) {
            pj = points + j;
            float d = distance(pi, pj);
            if (d < best.dist) {
                best.dist = d;
                best.a = pi;
                best.b = pj;
            }
        }
    }
    return best;
}

/* --------------------------------------------------------------------------
   Récursif diviser pour régner (closest pair en 3D).
   - points : tableau trié selon x (début du sous-tableau)
   - n : nombre d'éléments
   - buffer : espace de travail temporaire (taille N) pour construire la bande
   -------------------------------------------------------------------------- */
struct Pair closest_util(struct Drone *points, int n, struct Drone *buffer) {
    struct Pair best;
    best.dist = FLT_MAX;
    best.a = best.b = NULL;

    if (n <= 3) {
        return brute_force(points, n);
    }

    int mid = n / 2;
    struct Drone *mid_point = points + mid;  // point médian

    // Appels récursifs sur les deux moitiés (arithmétique de pointeurs)
    struct Pair left_best  = closest_util(points, mid, buffer);
    struct Pair right_best = closest_util(points + mid, n - mid, buffer);

    // Meilleur résultat des deux sous-problèmes
    if (left_best.dist < right_best.dist)
        best = left_best;
    else
        best = right_best;

    float delta = best.dist;

    // Construction de la bande (points dont |x - median| < delta)
    int strip_count = 0;
    for (int i = 0; i < n; i++) {
        struct Drone *p = points + i;
        if (fabsf(p->x - mid_point->x) < delta) {
            *(buffer + strip_count) = *p;  // copie via arithmétique
            strip_count++;
        }
    }

    // Tri de la bande selon y
    qsort(buffer, strip_count, sizeof(struct Drone), cmp_y);

    // Examen des paires dans la bande
    for (int i = 0; i < strip_count; i++) {
        struct Drone *p1 = buffer + i;
        for (int j = i + 1;
             j < strip_count && (buffer + j)->y - p1->y < delta;
             j++) {
            // Limitation à MAX_STRIP_NEIGHBORS pour garantir la complexité
            if (j - i > MAX_STRIP_NEIGHBORS) break;

            float d = distance(p1, buffer + j);
            if (d < best.dist) {
                best.dist = d;
                // Attention : les adresses dans buffer sont des copies,
                // on doit retrouver les originaux dans l'essaim.
                // Pour l'affichage final on gardera les id plutôt que les adresses.
                best.a = p1;
                best.b = buffer + j;
            }
        }
    }

    return best;
}

/* --------------------------------------------------------------------------
   Fonction principale : alloue l'essaim, exécute l'algorithme, affiche.
   -------------------------------------------------------------------------- */
int main() {
    // Allocation contiguë de l'essaim (10 000 drones) dans un seul bloc
    struct Drone *swarm = (struct Drone *)malloc(N * sizeof(struct Drone));
    if (swarm == NULL) {
        fprintf(stderr, "Échec allocation mémoire essaim.\n");
        return EXIT_FAILURE;
    }

    // Allocation du buffer de travail (taille N)
    struct Drone *buffer = (struct Drone *)malloc(N * sizeof(struct Drone));
    if (buffer == NULL) {
        fprintf(stderr, "Échec allocation mémoire buffer.\n");
        free(swarm);
        return EXIT_FAILURE;
    }

    // Initialisation des drones avec coordonnées aléatoires
    for (int i = 0; i < N; i++) {
        struct Drone *d = swarm + i;   // pointeur par arithmétique
        d->id = i;
        d->x = (float)(rand() % 100000) / 100.0f;
        d->y = (float)(rand() % 100000) / 100.0f;
        d->z = (float)(rand() % 100000) / 100.0f;
    }

    // Tri initial de l'essaim selon la coordonnée x
    qsort(swarm, N, sizeof(struct Drone), cmp_x);

    // Recherche de la paire la plus proche
    struct Pair result = closest_util(swarm, N, buffer);

    // Affichage du résultat
    if (result.a != NULL && result.b != NULL) {
        printf("Paire la plus proche :\n");
        printf("  Drone %d : (%.2f, %.2f, %.2f)\n",
               result.a->id, result.a->x, result.a->y, result.a->z);
        printf("  Drone %d : (%.2f, %.2f, %.2f)\n",
               result.b->id, result.b->x, result.b->y, result.b->z);
        printf("  Distance minimale = %.4f\n", result.dist);
    } else {
        printf("Aucune paire trouvée.\n");
    }

    // Libération des ressources
    free(swarm);
    free(buffer);
    return 0;
}
