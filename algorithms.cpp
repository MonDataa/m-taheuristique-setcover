#include <fstream>
#include <iostream>
#include <vector>
#include <set>
#include <algorithm>
#include <numeric>
#include <unordered_set>
#include <queue>
#include <random>
#include <limits>
#include <chrono>
#include "algorithms.h"
#include "utils.h"



using namespace std;


// Vérifie si une solution est faisable et affiche des informations détaillées
bool isFeasibleSolution(const vector<int>& solution, int m) {
    vector<bool> covered(m, false);
    int totalWeight = 0;

    for (int subset : solution) {
        totalWeight += weights[subset];
        for (int i = 0; i < m; ++i) {
            if (aij[i][subset] == 1) {
                covered[i] = true;
            }
        }
    }

    bool feasible = true;
    for (int i = 0; i < m; ++i) {
        if (!covered[i]) {
            feasible = false;
            // Suppression du cout "Non couvert: ..." si vous ne souhaitez pas afficher
        }
    }

    // Supprimer ce cout << endl; ici
    //cout << endl; // plus besoin, ça ajoutait une ligne vide

    return feasible;
}

vector<int> calculateCoverage(const vector<int>& solution, int m) {
    vector<int> coverage(m, 0);
    for (int subset : solution) {
        for (int i = 0; i < m; ++i) {
            if (aij[i][subset] == 1) {
                coverage[i]++;
            }
        }
    }
    return coverage;
}

int calculateWeight(const vector<int>& solution) {
    int totalWeight = 0;
    for (int subset : solution) {
        totalWeight += weights[subset];
    }
    return totalWeight;
}
// --------------------------------Fonction Greedy pour une solution réalisable------------------------------------
vector<int> greedySolution(int m, int n) {
    vector<int> covered(m, 0);  // Éléments couverts (0 : non couvert, 1 : couvert)
    vector<int> solution;       // Indices des sous-ensembles sélectionnés
    set<int> uncovered;         // Ensemble des éléments non encore couverts

    // Initialiser les éléments non couverts
    for (int i = 0; i < m; ++i) {
        uncovered.insert(i);
    }

    while (!uncovered.empty()) {
        int bestSubset = -1;    // Index du meilleur sous-ensemble
        int maxCovered = 0;     // Nombre maximum d'éléments couverts

        // Trouver le sous-ensemble qui couvre le maximum d'éléments non couverts
        for (int j = 0; j < n; ++j) {
            int coveredCount = 0;
            for (int i : uncovered) {
                if (aij[i][j] == 1) {
                    ++coveredCount;
                }
            }

            // Sélectionner ce sous-ensemble s'il couvre plus d'éléments
            if (coveredCount > maxCovered) {
                maxCovered = coveredCount;
                bestSubset = j;
            }
        }

        if (bestSubset == -1) {
            cerr << "Erreur : Impossible de couvrir tous les éléments." << endl;
            exit(EXIT_FAILURE);
        }

        // Ajouter le meilleur sous-ensemble à la solution
        solution.push_back(bestSubset);

        // Mettre à jour les éléments couverts
        for (int i = 0; i < m; ++i) {
            if (aij[i][bestSubset] == 1) {
                uncovered.erase(i);
            }
        }
    }

    return solution;
}


// ---------------------------------Recherche Tabou améliorée--------------------------------------------------

/*bool isFeasibleSolution(const vector<int>& solution, int m) {
    vector<bool> covered(m, false);
    for (int subset : solution) {
        for (int i = 0; i < m; ++i) {
            if (aij[i][subset] == 1) {
                covered[i] = true;
            }
        }
    }

    return all_of(covered.begin(), covered.end(), [](bool c){return c;});
}*/

vector<int> tabuSearch(int m, int n, int maxIterations, int tabuTenure, int maxNoImprovement) {
    // Générer une solution initiale faisable
    vector<int> solution;
    vector<bool> covered(m, false);
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < m; ++i) {
            if (!covered[i] && aij[i][j] == 1) {
                solution.push_back(j);
                for (int k = 0; k < m; ++k) {
                    if (aij[k][j] == 1) {
                        covered[k] = true;
                    }
                }
                break;
            }
        }
    }

    unordered_set<int> tabuList;
    int bestWeight = calculateWeight(solution);
    vector<int> bestSolution = solution;
    int iteration = 0;
    int noImprovementCount = 0;

    mt19937 rng((unsigned int)chrono::steady_clock::now().time_since_epoch().count());

    cout << "Démarrage de la recherche Tabu simple..." << endl;
    cout << "maxIterations=" << maxIterations << ", tabuTenure=" << tabuTenure
         << ", maxNoImprovement=" << maxNoImprovement << endl;

    for (iteration = 0; iteration < maxIterations; ++iteration) {
        cout << "Itération " << iteration+1 << "/" << maxIterations << endl;
        vector<int> currentSolution = solution;
        int candidateWeight = numeric_limits<int>::max();
        vector<int> bestCandidate = solution;

        // Générer des voisins par ajout
        for (int j = 0; j < n; ++j) {
            if (find(currentSolution.begin(), currentSolution.end(), j) == currentSolution.end()) {
                currentSolution.push_back(j);
                int w = calculateWeight(currentSolution);
                bool feasible = isFeasibleSolution(currentSolution, m);

                bool notTabuOrAspiration = (tabuList.find(j) == tabuList.end()) || (w < bestWeight);
                if (feasible && notTabuOrAspiration && w < candidateWeight) {
                    bestCandidate = currentSolution;
                    candidateWeight = w;
                }
                currentSolution.pop_back();
            }
        }

        // Générer des voisins par retrait
        for (int subset : solution) {
            auto it = find(currentSolution.begin(), currentSolution.end(), subset);
            if (it != currentSolution.end()) {
                currentSolution.erase(it);
                int w = calculateWeight(currentSolution);
                bool feasible = isFeasibleSolution(currentSolution, m);
                bool notTabuOrAspiration = (tabuList.find(subset) == tabuList.end()) || (w < bestWeight);

                if (feasible && notTabuOrAspiration && w < candidateWeight) {
                    bestCandidate = currentSolution;
                    candidateWeight = w;
                }
                // Annuler le retrait
                currentSolution.push_back(subset);
            }
        }

        if (candidateWeight < bestWeight) {
            solution = bestCandidate;
            bestWeight = candidateWeight;
            bestSolution = solution;
            noImprovementCount = 0;
            cout << "Amélioration trouvée à l'itération " << iteration+1 << " ! Nouveau poids : " << bestWeight << endl;

            // Mise à jour Tabu
            tabuList.clear();
            for (int subset : solution) {
                tabuList.insert(subset);
                if ((int)tabuList.size() > tabuTenure) {
                    auto first = tabuList.begin();
                    tabuList.erase(first);
                }
            }
        } else {
            noImprovementCount++;
            cout << "Pas d'amélioration à l'itération " << iteration+1
                 << " (itérations sans amélioration : " << noImprovementCount << ")" << endl;

            if (candidateWeight < numeric_limits<int>::max()) {
                solution = bestCandidate;
            }

            if (noImprovementCount >= maxNoImprovement) {
                cout << "Early stopping : aucune amélioration depuis " << maxNoImprovement << " itérations." << endl;
                break;
            }
        }

        cout << "Fin itération " << iteration+1 << " - Meilleur poids actuel : " << bestWeight << endl;
    }

    cout << "Fin de la recherche Tabu. Nombre total d'itérations : " << iteration << endl;
    return bestSolution;
}
// --------------------------------------------LNS ---------------------------------------------------------

//Exemple de fonction reconstructSolution
vector<int> reconstructSolution(const vector<int>& partialSolution, int m, int n) {
    // À partir de partialSolution, certains éléments sont non couverts.
    // On identifie ces éléments, puis on applique une stratégie gloutonne pour les recouvrir.
    // C'est similaire à constructFeasibleSolution, mais on part de partialSolution.

    vector<int> newSolution = partialSolution;
    vector<bool> covered(m, false);

    // Marquer les éléments couverts par la partialSolution
    for (int subset : partialSolution) {
        for (int i = 0; i < m; ++i) {
            if (aij[i][subset] == 1) {
                covered[i] = true;
            }
        }
    }

    // Couvrir les éléments non couverts
    set<int> uncovered;
    for (int i = 0; i < m; ++i) {
        if (!covered[i]) {
            uncovered.insert(i);
        }
    }

    while (!uncovered.empty()) {
        int bestSubset = -1;
        int maxCovered = 0;
        for (int j = 0; j < n; ++j) {
            if (find(newSolution.begin(), newSolution.end(), j) == newSolution.end()) {
                int coveredCount = 0;
                for (int i : uncovered) {
                    if (aij[i][j] == 1) {
                        ++coveredCount;
                    }
                }
                if (coveredCount > maxCovered) {
                    maxCovered = coveredCount;
                    bestSubset = j;
                }
            }
        }

        if (bestSubset == -1) {
            // Impossible de recouvrir tous les éléments
            break;
        }

        newSolution.push_back(bestSubset);
        // Marquer les nouveaux éléments couverts
        vector<int> newlyCovered;
        for (auto it = uncovered.begin(); it != uncovered.end();) {
            if (aij[*it][bestSubset] == 1) {
                it = uncovered.erase(it);
            } else {
                ++it;
            }
        }
    }

    return newSolution;
}

vector<int> largeNeighborhoodSearch(int m, int n, const vector<int>& initialSolution,
                                    int maxIterations, int destroySize) {
    vector<int> currentSolution = initialSolution;
    vector<int> bestSolution = currentSolution;
    int bestWeight = calculateWeight(bestSolution);

    mt19937 rng((unsigned int)chrono::steady_clock::now().time_since_epoch().count());

    cout << "Démarrage du LNS..." << endl;
    cout << "maxIterations=" << maxIterations << ", destroySize=" << destroySize << endl;

    int noImprovementCount = 0;
    int maxNoImprovement = 50; // Par exemple, un critère de stagnation pour LNS

    for (int iter = 0; iter < maxIterations; ++iter) {
        cout << "Itération " << iter+1 << "/" << maxIterations << endl;

        // Étape 1 : Destruction
        vector<int> destroyedSolution = currentSolution;
        set<int> removed;

        while ((int)removed.size() < destroySize && !destroyedSolution.empty()) {
            int idx = uniform_int_distribution<int>(0, (int)destroyedSolution.size()-1)(rng);
            removed.insert(destroyedSolution[idx]);
            destroyedSolution.erase(destroyedSolution.begin() + idx);
        }

        cout << "Après destruction : taille=" << destroyedSolution.size() << ", éléments retirés=" << removed.size() << endl;

        // Étape 2 : Reconstruction
        vector<int> newSolution = reconstructSolution(destroyedSolution, m, n);

        if (isFeasibleSolution(newSolution, m)) {
            int w = calculateWeight(newSolution);
            cout << "Solution faisable trouvée. Poids=" << w << " (Meilleur=" << bestWeight << ")" << endl;

            if (w < bestWeight) {
                bestWeight = w;
                bestSolution = newSolution;
                currentSolution = newSolution;
                noImprovementCount = 0;
                cout << "Amélioration ! Nouveau meilleur poids=" << bestWeight << endl;
            } else {
                // Stratégie simple : accepter si c'est légèrement plus grand ?
                if (w <= bestWeight * 1.05) {
                    currentSolution = newSolution;
                    cout << "Solution non améliorante acceptée pour diversification." << endl;
                } else {
                    cout << "Solution non améliorante rejetée." << endl;
                }
                noImprovementCount++;
            }
        } else {
            cout << "Solution non faisable, ignorée." << endl;
            noImprovementCount++;
        }

        // Critère d'arrêt si stagnation trop longue
        if (noImprovementCount >= maxNoImprovement) {
            cout << "Aucune amélioration depuis " << noImprovementCount << " itérations. Arrêt anticipé." << endl;
            break;
        }

        cout << "Fin itération " << iter+1 << " - Meilleur poids actuel : " << bestWeight << endl;
    }

    cout << "Fin du LNS. Meilleur poids obtenu : " << bestWeight << endl;
    return bestSolution;
}



// ---------------------------------Recherche Tabou Meta ++ --------------------------------------------------

// Fonction pour diversification (métapprentissage)
// Retire aléatoirement un certain nombre de sous-ensembles et reconstruit
vector<int> diversifySolution(const vector<int>& solution, int destroySize, int m, int n, mt19937& rng) {
    vector<int> newSol = solution;
    if ((int)newSol.size() > destroySize) {
        // Retirer destroySize sous-ensembles aléatoirement
        uniform_int_distribution<int> dist(0,(int)newSol.size()-1);
        for (int i = 0; i < destroySize; ++i) {
            if(newSol.empty()) break;
            int idx = dist(rng);
            newSol.erase(newSol.begin()+idx);
        }
    }

    // Reconstruire
    newSol = reconstructSolution(newSol, m, n);
    return newSol;
}

vector<int> tabuSearchMetaLearning(int m, int n, int maxIterations, int tabuTenure, int maxNoImprovement) {
    // Générer une solution initiale faisable
    vector<int> solution = greedySolution(m, n);
    int bestWeight = calculateWeight(solution);
    vector<int> bestSolution = solution;

    int iteration = 0;
    int noImprovementCount = 0;
    int initialTabuTenure = tabuTenure;

    unordered_set<int> tabuList;
    mt19937 rng((unsigned int)chrono::steady_clock::now().time_since_epoch().count());

    // Seuils pour le méta-learning
    int stagnationThreshold = maxNoImprovement / 2; // Par exemple, à mi-chemin
    int destroySizeForDiversification = 5; // Nombre de sous-ensembles à retirer lors de la diversification

    cout << "Début de la recherche Tabu avec méta-learning..." << endl;
    cout << "maxIterations=" << maxIterations << ", tabuTenure=" << tabuTenure
         << ", maxNoImprovement=" << maxNoImprovement << endl;

    // Vous pouvez aussi garder un historique des améliorations ou de l'évolution du poids
    // si vous le souhaitez, similaire au code hybridMetaLearningAndTabuv1
    // vector<pair<int,int>> historicalData; // (iteration, bestWeight)

    for (iteration = 0; iteration < maxIterations; iteration++) {
        cout << "Itération " << iteration+1 << "/" << maxIterations << endl;

        // Copie de la solution actuelle
        vector<int> currentSolution = solution;
        int candidateWeight = numeric_limits<int>::max();
        vector<int> bestCandidate = solution;

        // Génération des voisins par ajout
        for (int j = 0; j < n; ++j) {
            if (find(currentSolution.begin(), currentSolution.end(), j) == currentSolution.end()) {
                currentSolution.push_back(j);
                int w = calculateWeight(currentSolution);
                bool feasible = isFeasibleSolution(currentSolution, m);
                bool notTabuOrAspiration = (tabuList.find(j) == tabuList.end()) || (w < bestWeight);

                if (feasible && notTabuOrAspiration && w < candidateWeight) {
                    bestCandidate = currentSolution;
                    candidateWeight = w;
                }

                currentSolution.pop_back();
            }
        }

        // Génération des voisins par retrait
        for (int subset : solution) {
            auto it = find(currentSolution.begin(), currentSolution.end(), subset);
            if (it != currentSolution.end()) {
                currentSolution.erase(it);
                int w = calculateWeight(currentSolution);
                bool feasible = isFeasibleSolution(currentSolution, m);
                bool notTabuOrAspiration = (tabuList.find(subset) == tabuList.end()) || (w < bestWeight);

                if (feasible && notTabuOrAspiration && w < candidateWeight) {
                    bestCandidate = currentSolution;
                    candidateWeight = w;
                }

                // Annuler le retrait
                currentSolution.push_back(subset);
            }
        }

        // Vérification de l'amélioration
        if (candidateWeight < bestWeight) {
            solution = bestCandidate;
            bestWeight = candidateWeight;
            bestSolution = solution;
            noImprovementCount = 0;

            cout << "Amélioration trouvée à l'itération " << iteration+1 << " ! Nouveau poids : " << bestWeight << endl;

            // Mise à jour de la liste Tabou
            tabuList.clear();
            for (int subset : solution) {
                tabuList.insert(subset);
                if ((int)tabuList.size() > tabuTenure) {
                    auto first = tabuList.begin();
                    tabuList.erase(first);
                }
            }
        } else {
            // Pas d'amélioration
            noImprovementCount++;
            if (candidateWeight < numeric_limits<int>::max()) {
                solution = bestCandidate;
            }

            // Affichage de la stagnation
            cout << "Pas d'amélioration à l'itération " << iteration+1 << ". Nombre d'itérations sans amélioration : " << noImprovementCount << endl;

            // Diversification si stagnation
            if (noImprovementCount == stagnationThreshold) {
                cout << "Stagnation détectée après " << stagnationThreshold << " itérations sans amélioration. Diversification..." << endl;
                vector<int> diversifiedSolution = diversifySolution(solution, destroySizeForDiversification, m, n, rng);
                if (isFeasibleSolution(diversifiedSolution, m)) {
                    int w = calculateWeight(diversifiedSolution);
                    if (w < bestWeight) {
                        bestWeight = w;
                        bestSolution = diversifiedSolution;
                        cout << "Amélioration après diversification ! Poids : " << bestWeight << endl;
                    } else {
                        cout << "Diversification effectuée, pas d'amélioration, mais on conserve la solution diversifiée." << endl;
                    }
                    solution = diversifiedSolution;
                } else {
                    cout << "Diversification non faisable, on conserve la solution actuelle." << endl;
                }

                // On peut ajuster la durée tabou
                tabuTenure += 5;
                cout << "Tabu Tenure augmenté à " << tabuTenure << endl;
            }

            // Critère d'arrêt si aucune amélioration pendant trop longtemps
            /*if (noImprovementCount >= maxNoImprovement) {
                cout << "Early stopping : aucune amélioration depuis " << maxNoImprovement << " itérations." << endl;
                break;
            }*/
        }

        // Affichage de l'état à la fin de l'itération
        cout << "Fin de l'itération " << iteration+1 << " - Meilleur poids actuel : " << bestWeight
             << ", Itérations sans amélioration : " << noImprovementCount << endl;
    }

    cout << "Fin de la recherche Tabu avec méta-learning. Nombre total d'itérations effectuées : " << iteration << endl;

    return bestSolution;
}


// --------------------------------------------------ALNS-----------------------------------------

int selectMethod(const vector<Method>& methods, mt19937& rng) {
    double sumScores=0.0;
    for (auto &meth : methods) sumScores += meth.score;
    if(sumScores==0.0) return 0;
    uniform_real_distribution<double> dist(0.0,sumScores);
    double r=dist(rng);
    for (int i=0;i<(int)methods.size();i++){
        if(r<methods[i].score)return i;
        r-=methods[i].score;
    }
    return (int)methods.size()-1;
}

void updateScores(vector<Method>& destructionMethods, vector<Method>& reconstructionMethods,
                  int dIdx, int rIdx, double alpha, bool improved) {
    if(improved) {
        destructionMethods[dIdx].score += alpha;
        reconstructionMethods[rIdx].score += alpha;
    }
}

static vector<int> safeDestroyRandom(const vector<int>& solution, int destroySize, mt19937& rng) {
    vector<int> newSol = solution;
    if (newSol.empty()) {
        cout << "[destroyRandom] newSol est vide, rien à retirer." << endl;
        return newSol;
    }
    destroySize = min(destroySize, (int)newSol.size());
    uniform_int_distribution<int> dist(0,(int)newSol.size()-1);
    for (int i=0;i<destroySize && !newSol.empty();i++){
        int idx=dist(rng);
        if(idx<0 || idx>=(int)newSol.size()){
            cout<<"[destroyRandom] idx hors limite: idx="<<idx<<", taille="<<newSol.size()<<endl;
            continue; // éviter un erase hors limite
        }
        // Retirer l'élément à idx
        newSol.erase(newSol.begin()+idx);
    }
    return newSol;
}

static vector<int> safeDestroyHeavy(const vector<int>& solution, int destroySize, mt19937& rng) {
    vector<int> newSol;
    if (solution.empty()) {
        cout << "[destroyHeavy] solution vide, rien à retirer." << endl;
        return solution;
    }
    destroySize = min(destroySize, (int)solution.size());

    vector<pair<int,int>> subs;
    for (int s : solution) subs.push_back({weights[s], s});
    sort(subs.begin(), subs.end(), [](auto &a, auto &b){return a.first > b.first;});
    int count=0;
    for (int i=0; i<(int)subs.size(); i++){
        if(count<destroySize) count++;
        else newSol.push_back(subs[i].second);
    }
    return newSol;
}

vector<int> reconstructGreedyMethod(const vector<int>& partialSolution, int m, int n) {
    return reconstructSolution(partialSolution, m, n);
}

vector<int> adaptiveLargeNeighborhoodSearch(int m, int n, const vector<int>& initialSolution,
                                            int maxIterations, int destroySize, double alpha = 1.0, int maxNoImprovement=50) {

    vector<int> currentSolution = initialSolution;
    vector<int> bestSolution = currentSolution;
    int bestWeight = calculateWeight(bestSolution);

    cout << "ALNS démarré. maxIterations=" << maxIterations << ", destroySize=" << destroySize << ", maxNoImprovement=" << maxNoImprovement << endl;

    mt19937 rng((unsigned int)chrono::steady_clock::now().time_since_epoch().count());

    vector<Method> destructionMethods = {
        {"Random", 1.0},
        {"Heavy", 1.0}
    };

    vector<Method> reconstructionMethods = {
        {"Greedy", 1.0}
    };

    int noImprovementCount=0;
    int stagnationThreshold = maxNoImprovement / 2;
    int iteration=0;

    for (; iteration < maxIterations; ++iteration) {
        cout << "Itération " << iteration+1 << "/" << maxIterations << endl;

        // Sélection des méthodes
        int dIdx = selectMethod(destructionMethods, rng);
        int rIdx = selectMethod(reconstructionMethods, rng);

        cout << "Destruction: " << destructionMethods[dIdx].name << ", Reconstruction: " << reconstructionMethods[rIdx].name << endl;

        int currentDestroySize = destroySize;
        if(noImprovementCount>stagnationThreshold) {
            currentDestroySize = min(destroySize*(noImprovementCount/stagnationThreshold+1), (int)currentSolution.size()/2);
        }

        // Appliquer la destruction
        vector<int> destroyedSolution;
        if (destructionMethods[dIdx].name == "Random") {
            cout << "[ALNS] Avant destruction Random: taille currentSolution="<<currentSolution.size()<<", currentDestroySize="<<currentDestroySize<<endl;
            destroyedSolution = safeDestroyRandom(currentSolution, currentDestroySize, rng);
        } else if (destructionMethods[dIdx].name == "Heavy") {
            cout << "[ALNS] Avant destruction Heavy: taille currentSolution="<<currentSolution.size()<<", currentDestroySize="<<currentDestroySize<<endl;
            destroyedSolution = safeDestroyHeavy(currentSolution, currentDestroySize, rng);
        }

        cout << "[ALNS] Après destruction: taille=" << destroyedSolution.size() << endl;

        // Reconstruction
        vector<int> newSolution = reconstructGreedyMethod(destroyedSolution, m, n);
        if(isFeasibleSolution(newSolution, m)) {
            int w = calculateWeight(newSolution);
            cout << "Solution faisable: poids=" << w << ", taille=" << newSolution.size() << endl;
            bool improved=false;
            if(w<bestWeight) {
                bestWeight=w;
                bestSolution=newSolution;
                currentSolution=newSolution;
                noImprovementCount=0;
                improved=true;
                cout << "Amélioration ! Nouveau meilleur poids="<<bestWeight<<endl;
            } else {
                // Accepter solutions non améliorantes si proches
                if (w <= bestWeight*1.05) {
                    currentSolution=newSolution;
                    cout << "Solution non améliorante acceptée. Poids="<<w<<endl;
                } else {
                    // Accepter parfois même solutions moins bonnes (après stagnation)
                    if(noImprovementCount>stagnationThreshold) {
                        uniform_real_distribution<double> dist(0.0,1.0);
                        double gap = w - bestWeight;
                        double p = exp(-gap/1000.0);
                        if(dist(rng)<p) {
                            currentSolution=newSolution;
                            cout << "Solution nettement moins bonne acceptée aléatoirement (gap="<<gap<<")." << endl;
                        } else {
                            cout << "Solution non améliorante rejetée (gap="<<gap<<")." << endl;
                        }
                    } else {
                        cout << "Solution non améliorante rejetée." << endl;
                    }
                }
                if(!improved) noImprovementCount++;
            }
            updateScores(destructionMethods, reconstructionMethods, dIdx, rIdx, alpha, improved);
        } else {
            cout << "Solution non faisable, ignorée." << endl;
            noImprovementCount++;
        }

        // Diversification si stagnation
        if(noImprovementCount==stagnationThreshold) {
            cout << "Stagnation détectée. Diversification radicale." << endl;
            currentSolution=bestSolution;
            int aggressiveDestroySize = min((int)currentSolution.size()/2,(int)(destroySize*2));
            cout << "[Diversification] Avant destruction radicale: size="<<currentSolution.size()<<", aggressiveDestroySize="<<aggressiveDestroySize<<endl;
            vector<int> diversifiedSolution = safeDestroyRandom(currentSolution, aggressiveDestroySize, rng);
            diversifiedSolution = reconstructGreedyMethod(diversifiedSolution,m,n);
            if(isFeasibleSolution(diversifiedSolution,m)) {
                int w = calculateWeight(diversifiedSolution);
                if(w<bestWeight){
                    bestWeight=w;
                    bestSolution=diversifiedSolution;
                    cout << "Amélioration après diversification radicale ! Poids:"<<bestWeight<<endl;
                } else {
                    cout << "Pas d'amélioration, mais on accepte la solution diversifiée." << endl;
                }
                currentSolution=diversifiedSolution;
                noImprovementCount=0;
            } else {
                cout << "Diversification radicale non faisable, on garde la solution actuelle." << endl;
            }
        }

        // Early stopping
        if(noImprovementCount>=maxNoImprovement) {
            cout << "Early stopping après " << noImprovementCount << " itérations sans amélioration." << endl;
            break;
        }
    }

    cout << "ALNS terminé après "<<iteration<<" itérations. Meilleur poids="<<bestWeight<<"."<<endl;
    return bestSolution;
}


//------------------------------------------VNS------------------------------------------------------

vector<int> localSearchVNS(const vector<int>& solution, int m, int n, int localSearchAttempts) {
    vector<int> best = solution;
    int bestWeight = calculateWeight(best);
    mt19937 rng((unsigned int)chrono::steady_clock::now().time_since_epoch().count());

    for (int attempt = 0; attempt < localSearchAttempts; attempt++) {
        if (best.empty()) break;
        vector<int> candidate = best;

        // Retirer un sous-ensemble aléatoirement
        uniform_int_distribution<int> dist(0, (int)candidate.size()-1);
        int idx = dist(rng);
        candidate.erase(candidate.begin() + idx);

        // Tenter d'ajouter un sous-ensemble différent
        uniform_int_distribution<int> dist2(0, n-1);
        int added = dist2(rng);
        if (find(candidate.begin(), candidate.end(), added) == candidate.end()) {
            candidate.push_back(added);

            candidate = reconstructSolution(candidate, m, n);

            if (isFeasibleSolution(candidate, m)) {
                int w = calculateWeight(candidate);
                if (w < bestWeight) {
                    best = candidate;
                    bestWeight = w;
                }
            }
        }
    }

    return best;
}


vector<int> shakingVNS(const vector<int>& solution, int k, int m, int n) {
    // Exemple de "shaking" :
    // On modifie la solution en retirant et ajoutant k sous-ensembles aléatoirement.
    vector<int> shaken = solution;
    mt19937 rng((unsigned int)chrono::steady_clock::now().time_since_epoch().count());

    // Retirer k sous-ensembles aléatoirement (si possible)
    for (int i = 0; i < k && !shaken.empty(); i++) {
        uniform_int_distribution<int> dist(0, (int)shaken.size()-1);
        int idx = dist(rng);
        shaken.erase(shaken.begin()+idx);
    }

    // Ajouter k sous-ensembles aléatoires
    for (int i = 0; i < k; i++) {
        uniform_int_distribution<int> dist(0, n-1);
        int added = dist(rng);
        if (find(shaken.begin(), shaken.end(), added) == shaken.end()) {
            shaken.push_back(added);
        }
    }

    // Reconstruction (si nécessaire) pour s'assurer que la solution est faisable
    shaken = reconstructSolution(shaken, m, n);

    return shaken;
}

vector<int> vnsSearchCustom(int m, int n, const vector<int>& initialSolution, int maxIterations, int kMax, int localSearchAttempts) {
    vector<int> bestSolution = initialSolution;
    int bestWeight = calculateWeight(bestSolution);

    int iteration = 0;
    cout << "Démarrage du VNS..." << endl;
    cout << "maxIterations=" << maxIterations << ", kMax=" << kMax << ", localSearchAttempts=" << localSearchAttempts << endl;

    while (iteration < maxIterations) {
        int k = 1;
        bool improvementFound = false;

        // On part de la meilleure solution courante
        vector<int> currentSolution = bestSolution;

        while (k <= kMax) {
            cout << "Itération " << iteration+1 << "/" << maxIterations << ", voisinage k=" << k << endl;

            // Shaking
            vector<int> shakenSolution = shakingVNS(currentSolution, k, m, n);

            // Local Search avec nombre d'essais spécifié
            vector<int> improvedSolution = localSearchVNS(shakenSolution, m, n, localSearchAttempts);

            if (isFeasibleSolution(improvedSolution, m)) {
                int w = calculateWeight(improvedSolution);
                if (w < bestWeight) {
                    bestSolution = improvedSolution;
                    bestWeight = w;
                    cout << "Amélioration trouvée ! Nouveau poids : " << bestWeight << endl;
                    improvementFound = true;
                    // On revient au premier voisinage
                    k = 1;
                    // On met à jour la solution courante
                    currentSolution = bestSolution;
                    continue; // Recommence avec k=1
                }
            }

            // Si pas d'amélioration, on passe au voisinage suivant
            k++;
        }

        iteration++;
    }

    cout << "Fin du VNS après " << iteration << " itérations. Meilleur poids=" << bestWeight << "." << endl;
    return bestSolution;
}
