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


// V�rifie si une solution est faisable et affiche des informations d�taill�es
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
    //cout << endl; // plus besoin, �a ajoutait une ligne vide

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
// --------------------------------Fonction Greedy pour une solution r�alisable------------------------------------
vector<int> greedySolution(int m, int n) {
    vector<int> covered(m, 0);  // �l�ments couverts (0 : non couvert, 1 : couvert)
    vector<int> solution;       // Indices des sous-ensembles s�lectionn�s
    set<int> uncovered;         // Ensemble des �l�ments non encore couverts

    // Initialiser les �l�ments non couverts
    for (int i = 0; i < m; ++i) {
        uncovered.insert(i);
    }

    while (!uncovered.empty()) {
        int bestSubset = -1;    // Index du meilleur sous-ensemble
        int maxCovered = 0;     // Nombre maximum d'�l�ments couverts

        // Trouver le sous-ensemble qui couvre le maximum d'�l�ments non couverts
        for (int j = 0; j < n; ++j) {
            int coveredCount = 0;
            for (int i : uncovered) {
                if (aij[i][j] == 1) {
                    ++coveredCount;
                }
            }

            // S�lectionner ce sous-ensemble s'il couvre plus d'�l�ments
            if (coveredCount > maxCovered) {
                maxCovered = coveredCount;
                bestSubset = j;
            }
        }

        if (bestSubset == -1) {
            cerr << "Erreur : Impossible de couvrir tous les �l�ments." << endl;
            exit(EXIT_FAILURE);
        }

        // Ajouter le meilleur sous-ensemble � la solution
        solution.push_back(bestSubset);

        // Mettre � jour les �l�ments couverts
        for (int i = 0; i < m; ++i) {
            if (aij[i][bestSubset] == 1) {
                uncovered.erase(i);
            }
        }
    }

    return solution;
}


// ---------------------------------Recherche Tabou am�lior�e--------------------------------------------------

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
    // G�n�rer une solution initiale faisable
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

    cout << "D�marrage de la recherche Tabu simple..." << endl;
    cout << "maxIterations=" << maxIterations << ", tabuTenure=" << tabuTenure
         << ", maxNoImprovement=" << maxNoImprovement << endl;

    for (iteration = 0; iteration < maxIterations; ++iteration) {
        cout << "It�ration " << iteration+1 << "/" << maxIterations << endl;
        vector<int> currentSolution = solution;
        int candidateWeight = numeric_limits<int>::max();
        vector<int> bestCandidate = solution;

        // G�n�rer des voisins par ajout
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

        // G�n�rer des voisins par retrait
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
            cout << "Am�lioration trouv�e � l'it�ration " << iteration+1 << " ! Nouveau poids : " << bestWeight << endl;

            // Mise � jour Tabu
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
            cout << "Pas d'am�lioration � l'it�ration " << iteration+1
                 << " (it�rations sans am�lioration : " << noImprovementCount << ")" << endl;

            if (candidateWeight < numeric_limits<int>::max()) {
                solution = bestCandidate;
            }

            if (noImprovementCount >= maxNoImprovement) {
                cout << "Early stopping : aucune am�lioration depuis " << maxNoImprovement << " it�rations." << endl;
                break;
            }
        }

        cout << "Fin it�ration " << iteration+1 << " - Meilleur poids actuel : " << bestWeight << endl;
    }

    cout << "Fin de la recherche Tabu. Nombre total d'it�rations : " << iteration << endl;
    return bestSolution;
}
// --------------------------------------------LNS ---------------------------------------------------------

//Exemple de fonction reconstructSolution
vector<int> reconstructSolution(const vector<int>& partialSolution, int m, int n) {
    // � partir de partialSolution, certains �l�ments sont non couverts.
    // On identifie ces �l�ments, puis on applique une strat�gie gloutonne pour les recouvrir.
    // C'est similaire � constructFeasibleSolution, mais on part de partialSolution.

    vector<int> newSolution = partialSolution;
    vector<bool> covered(m, false);

    // Marquer les �l�ments couverts par la partialSolution
    for (int subset : partialSolution) {
        for (int i = 0; i < m; ++i) {
            if (aij[i][subset] == 1) {
                covered[i] = true;
            }
        }
    }

    // Couvrir les �l�ments non couverts
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
            // Impossible de recouvrir tous les �l�ments
            break;
        }

        newSolution.push_back(bestSubset);
        // Marquer les nouveaux �l�ments couverts
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

    cout << "D�marrage du LNS..." << endl;
    cout << "maxIterations=" << maxIterations << ", destroySize=" << destroySize << endl;

    int noImprovementCount = 0;
    int maxNoImprovement = 50; // Par exemple, un crit�re de stagnation pour LNS

    for (int iter = 0; iter < maxIterations; ++iter) {
        cout << "It�ration " << iter+1 << "/" << maxIterations << endl;

        // �tape 1 : Destruction
        vector<int> destroyedSolution = currentSolution;
        set<int> removed;

        while ((int)removed.size() < destroySize && !destroyedSolution.empty()) {
            int idx = uniform_int_distribution<int>(0, (int)destroyedSolution.size()-1)(rng);
            removed.insert(destroyedSolution[idx]);
            destroyedSolution.erase(destroyedSolution.begin() + idx);
        }

        cout << "Apr�s destruction : taille=" << destroyedSolution.size() << ", �l�ments retir�s=" << removed.size() << endl;

        // �tape 2 : Reconstruction
        vector<int> newSolution = reconstructSolution(destroyedSolution, m, n);

        if (isFeasibleSolution(newSolution, m)) {
            int w = calculateWeight(newSolution);
            cout << "Solution faisable trouv�e. Poids=" << w << " (Meilleur=" << bestWeight << ")" << endl;

            if (w < bestWeight) {
                bestWeight = w;
                bestSolution = newSolution;
                currentSolution = newSolution;
                noImprovementCount = 0;
                cout << "Am�lioration ! Nouveau meilleur poids=" << bestWeight << endl;
            } else {
                // Strat�gie simple : accepter si c'est l�g�rement plus grand ?
                if (w <= bestWeight * 1.05) {
                    currentSolution = newSolution;
                    cout << "Solution non am�liorante accept�e pour diversification." << endl;
                } else {
                    cout << "Solution non am�liorante rejet�e." << endl;
                }
                noImprovementCount++;
            }
        } else {
            cout << "Solution non faisable, ignor�e." << endl;
            noImprovementCount++;
        }

        // Crit�re d'arr�t si stagnation trop longue
        if (noImprovementCount >= maxNoImprovement) {
            cout << "Aucune am�lioration depuis " << noImprovementCount << " it�rations. Arr�t anticip�." << endl;
            break;
        }

        cout << "Fin it�ration " << iter+1 << " - Meilleur poids actuel : " << bestWeight << endl;
    }

    cout << "Fin du LNS. Meilleur poids obtenu : " << bestWeight << endl;
    return bestSolution;
}



// ---------------------------------Recherche Tabou Meta ++ --------------------------------------------------

vector<int> diversifySolution(const vector<int>& solution, int destroySize, int m, int n, mt19937& rng) {
    vector<int> newSol = solution;
    set<int> removed;

    // Retirer al�atoirement destroySize sous-ensembles
    while ((int)removed.size() < destroySize && !newSol.empty()) {
        uniform_int_distribution<int> dist(0, (int)newSol.size() - 1);
        int idx = dist(rng);
        removed.insert(newSol[idx]);
        newSol.erase(newSol.begin() + idx);
    }

    // Reconstruire la solution
    newSol = reconstructSolution(newSol, m, n);
    return newSol;
}

vector<int> tabuSearchMetaLearning(int m, int n, int maxIterations, int initialTabuTenure, int maxNoImprovement) {
    // Solution initiale avec l'algorithme glouton
    vector<int> solution = greedySolution(m, n);
    int bestWeight = calculateWeight(solution);
    vector<int> bestSolution = solution;

    unordered_set<int> tabuList;
    int tabuTenure = initialTabuTenure;

    int iteration = 0, noImprovementCount = 0;
    int stagnationThreshold = maxNoImprovement / 2;
    int destroySize = 5; // Taille pour la diversification

    mt19937 rng((unsigned int)chrono::steady_clock::now().time_since_epoch().count());
    auto startTime = chrono::high_resolution_clock::now();

    cout << "D�marrage de Tabu Search M�ta-Learning am�lior�e..." << endl;

    for (; iteration < maxIterations; ++iteration) {
        cout << "\nIt�ration " << iteration + 1 << "/" << maxIterations << endl;

        vector<int> bestCandidate = solution;
        int candidateWeight = numeric_limits<int>::max();

        // Exploration am�lior�e des voisins
        for (int j = 0; j < n; ++j) {
            if (find(solution.begin(), solution.end(), j) == solution.end()) {
                vector<int> tempSolution = solution;
                tempSolution.push_back(j);

                if (isFeasibleSolution(tempSolution, m)) {
                    int weight = calculateWeight(tempSolution);
                    if ((tabuList.find(j) == tabuList.end() || weight < bestWeight) && weight < candidateWeight) {
                        bestCandidate = tempSolution;
                        candidateWeight = weight;
                    }
                }
            }
        }

        for (int subset : solution) {
            vector<int> tempSolution = solution;
            tempSolution.erase(remove(tempSolution.begin(), tempSolution.end(), subset), tempSolution.end());

            if (isFeasibleSolution(tempSolution, m)) {
                int weight = calculateWeight(tempSolution);
                if ((tabuList.find(subset) == tabuList.end() || weight < bestWeight) && weight < candidateWeight) {
                    bestCandidate = tempSolution;
                    candidateWeight = weight;
                }
            }
        }

        // Mise � jour des solutions et Tabu Tenure
        if (candidateWeight < bestWeight) {
            solution = bestCandidate;
            bestWeight = candidateWeight;
            bestSolution = solution;
            noImprovementCount = 0;
            tabuTenure = max(5, tabuTenure - 1); // R�duire l�g�rement la dur�e tabou
            cout << "Am�lioration trouv�e ! Nouveau poids : " << bestWeight << endl;
        } else {
            noImprovementCount++;
            tabuTenure = min(20, tabuTenure + 1); // Augmenter l�g�rement en cas de stagnation
            cout << "Aucune am�lioration. Tabu Tenure ajust�e � : " << tabuTenure << endl;
        }

        // Mettre � jour la liste Tabou
        tabuList.insert(candidateWeight);
        if (tabuList.size() > tabuTenure) tabuList.erase(tabuList.begin());

        // Diversification en cas de stagnation
        if (noImprovementCount == stagnationThreshold) {
            cout << "Stagnation d�tect�e. Diversification..." << endl;
            vector<int> diversifiedSolution = diversifySolution(solution, destroySize, m, n, rng);

            if (isFeasibleSolution(diversifiedSolution, m)) {
                solution = diversifiedSolution;
                noImprovementCount = 0;
                cout << "Diversification r�ussie !" << endl;
            } else {
                cout << "Diversification �chou�e. Solution inchang�e." << endl;
            }
        }

        // Arr�t anticip� si aucune am�lioration sur une longue p�riode
        if (noImprovementCount >= maxNoImprovement) {
            cout << "Arr�t pr�coce : aucune am�lioration depuis " << maxNoImprovement << " it�rations." << endl;
            break;
        }

        cout << "Fin de l'it�ration " << iteration + 1 << ", Meilleur poids actuel : " << bestWeight << endl;
    }

    auto endTime = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed = endTime - startTime;
    cout << "Fin de la recherche Tabu M�ta-Learning am�lior�e en " << elapsed.count() << " secondes." << endl;
    cout << "Meilleur poids obtenu : " << bestWeight << endl;

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
        cout << "[destroyRandom] newSol est vide, rien � retirer." << endl;
        return newSol;
    }
    destroySize = min(destroySize, (int)newSol.size());
    uniform_int_distribution<int> dist(0,(int)newSol.size()-1);
    for (int i=0;i<destroySize && !newSol.empty();i++){
        int idx=dist(rng);
        if(idx<0 || idx>=(int)newSol.size()){
            cout<<"[destroyRandom] idx hors limite: idx="<<idx<<", taille="<<newSol.size()<<endl;
            continue; // �viter un erase hors limite
        }
        // Retirer l'�l�ment � idx
        newSol.erase(newSol.begin()+idx);
    }
    return newSol;
}

static vector<int> safeDestroyHeavy(const vector<int>& solution, int destroySize, mt19937& rng) {
    vector<int> newSol;
    if (solution.empty()) {
        cout << "[destroyHeavy] solution vide, rien � retirer." << endl;
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

    cout << "ALNS d�marr�. maxIterations=" << maxIterations << ", destroySize=" << destroySize << ", maxNoImprovement=" << maxNoImprovement << endl;

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
        cout << "It�ration " << iteration+1 << "/" << maxIterations << endl;

        // S�lection des m�thodes
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

        cout << "[ALNS] Apr�s destruction: taille=" << destroyedSolution.size() << endl;

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
                cout << "Am�lioration ! Nouveau meilleur poids="<<bestWeight<<endl;
            } else {
                // Accepter solutions non am�liorantes si proches
                if (w <= bestWeight*1.05) {
                    currentSolution=newSolution;
                    cout << "Solution non am�liorante accept�e. Poids="<<w<<endl;
                } else {
                    // Accepter parfois m�me solutions moins bonnes (apr�s stagnation)
                    if(noImprovementCount>stagnationThreshold) {
                        uniform_real_distribution<double> dist(0.0,1.0);
                        double gap = w - bestWeight;
                        double p = exp(-gap/1000.0);
                        if(dist(rng)<p) {
                            currentSolution=newSolution;
                            cout << "Solution nettement moins bonne accept�e al�atoirement (gap="<<gap<<")." << endl;
                        } else {
                            cout << "Solution non am�liorante rejet�e (gap="<<gap<<")." << endl;
                        }
                    } else {
                        cout << "Solution non am�liorante rejet�e." << endl;
                    }
                }
                if(!improved) noImprovementCount++;
            }
            updateScores(destructionMethods, reconstructionMethods, dIdx, rIdx, alpha, improved);
        } else {
            cout << "Solution non faisable, ignor�e." << endl;
            noImprovementCount++;
        }

        // Diversification si stagnation
        if(noImprovementCount==stagnationThreshold) {
            cout << "Stagnation d�tect�e. Diversification radicale." << endl;
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
                    cout << "Am�lioration apr�s diversification radicale ! Poids:"<<bestWeight<<endl;
                } else {
                    cout << "Pas d'am�lioration, mais on accepte la solution diversifi�e." << endl;
                }
                currentSolution=diversifiedSolution;
                noImprovementCount=0;
            } else {
                cout << "Diversification radicale non faisable, on garde la solution actuelle." << endl;
            }
        }

        // Early stopping
        if(noImprovementCount>=maxNoImprovement) {
            cout << "Early stopping apr�s " << noImprovementCount << " it�rations sans am�lioration." << endl;
            break;
        }
    }

    cout << "ALNS termin� apr�s "<<iteration<<" it�rations. Meilleur poids="<<bestWeight<<"."<<endl;
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

        // Retirer un sous-ensemble al�atoirement
        uniform_int_distribution<int> dist(0, (int)candidate.size()-1);
        int idx = dist(rng);
        candidate.erase(candidate.begin() + idx);

        // Tenter d'ajouter un sous-ensemble diff�rent
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
    // On modifie la solution en retirant et ajoutant k sous-ensembles al�atoirement.
    vector<int> shaken = solution;
    mt19937 rng((unsigned int)chrono::steady_clock::now().time_since_epoch().count());

    // Retirer k sous-ensembles al�atoirement (si possible)
    for (int i = 0; i < k && !shaken.empty(); i++) {
        uniform_int_distribution<int> dist(0, (int)shaken.size()-1);
        int idx = dist(rng);
        shaken.erase(shaken.begin()+idx);
    }

    // Ajouter k sous-ensembles al�atoires
    for (int i = 0; i < k; i++) {
        uniform_int_distribution<int> dist(0, n-1);
        int added = dist(rng);
        if (find(shaken.begin(), shaken.end(), added) == shaken.end()) {
            shaken.push_back(added);
        }
    }

    // Reconstruction (si n�cessaire) pour s'assurer que la solution est faisable
    shaken = reconstructSolution(shaken, m, n);

    return shaken;
}

vector<int> vnsSearchCustom(int m, int n, const vector<int>& initialSolution, int maxIterations, int kMax, int localSearchAttempts) {
    vector<int> bestSolution = initialSolution;
    int bestWeight = calculateWeight(bestSolution);

    int iteration = 0;
    cout << "D�marrage du VNS..." << endl;
    cout << "maxIterations=" << maxIterations << ", kMax=" << kMax << ", localSearchAttempts=" << localSearchAttempts << endl;

    while (iteration < maxIterations) {
        int k = 1;
        bool improvementFound = false;

        // On part de la meilleure solution courante
        vector<int> currentSolution = bestSolution;

        while (k <= kMax) {
            cout << "It�ration " << iteration+1 << "/" << maxIterations << ", voisinage k=" << k << endl;

            // Shaking
            vector<int> shakenSolution = shakingVNS(currentSolution, k, m, n);

            // Local Search avec nombre d'essais sp�cifi�
            vector<int> improvedSolution = localSearchVNS(shakenSolution, m, n, localSearchAttempts);

            if (isFeasibleSolution(improvedSolution, m)) {
                int w = calculateWeight(improvedSolution);
                if (w < bestWeight) {
                    bestSolution = improvedSolution;
                    bestWeight = w;
                    cout << "Am�lioration trouv�e ! Nouveau poids : " << bestWeight << endl;
                    improvementFound = true;
                    // On revient au premier voisinage
                    k = 1;
                    // On met � jour la solution courante
                    currentSolution = bestSolution;
                    continue; // Recommence avec k=1
                }
            }

            // Si pas d'am�lioration, on passe au voisinage suivant
            k++;
        }

        iteration++;
    }

    cout << "Fin du VNS apr�s " << iteration << " it�rations. Meilleur poids=" << bestWeight << "." << endl;
    return bestSolution;
}

//--------------------------------------------Grasp-------------------------------------------
// **Phase de construction de GRASP** : construction gloutonne avec randomisation
vector<int> graspConstruct(int m, int n, double alpha) {
    vector<int> solution;
    set<int> uncovered; // Ensemble des �l�ments non encore couverts
    for (int i = 0; i < m; ++i) uncovered.insert(i);

    while (!uncovered.empty()) {
        vector<pair<int, double>> candidateList;

        for (int j = 0; j < n; ++j) {
            int coverCount = 0;
            for (int i : uncovered) {
                if (aij[i][j] == 1) coverCount++;
            }
            if (coverCount > 0) {
                double cost = static_cast<double>(weights[j]) / coverCount; // Score glouton
                candidateList.push_back({j, cost});
            }
        }

        // Trier par co�t croissant
        sort(candidateList.begin(), candidateList.end(), [](auto& a, auto& b) {
            return a.second < b.second;
        });

        if (candidateList.empty()) {
            cerr << "Erreur : Impossible de couvrir tous les �l�ments." << endl;
            exit(EXIT_FAILURE);
        }

        // S�lection al�atoire parmi les meilleurs candidats (alpha d�finit la randomisation)
        int limit = max(1, static_cast<int>(alpha * candidateList.size()));
        random_device rd;
        mt19937 rng(rd());
        uniform_int_distribution<int> dist(0, limit - 1);

        int selectedSubset = candidateList[dist(rng)].first;
        solution.push_back(selectedSubset);

        // Mettre � jour les �l�ments couverts
        for (auto it = uncovered.begin(); it != uncovered.end();) {
            if (aij[*it][selectedSubset] == 1) it = uncovered.erase(it);
            else ++it;
        }
    }

    return solution;
}

// **Recherche locale** : am�liore une solution en essayant des ajouts/retraits de sous-ensembles
vector<int> localSearch(const vector<int>& initialSolution, int m, int n) {
    vector<int> solution = initialSolution;
    bool improved = true;

    while (improved) {
        improved = false;

        for (int j = 0; j < n; ++j) {
            if (find(solution.begin(), solution.end(), j) == solution.end()) {
                vector<int> newSolution = solution;
                newSolution.push_back(j);

                if (isFeasibleSolution(newSolution, m) && calculateWeight(newSolution) < calculateWeight(solution)) {
                    solution = newSolution;
                    improved = true;
                    break;
                }
            }
        }

        for (auto it = solution.begin(); it != solution.end();) {
            int removedSubset = *it;
            vector<int> newSolution = solution;
            newSolution.erase(remove(newSolution.begin(), newSolution.end(), removedSubset), newSolution.end());

            if (isFeasibleSolution(newSolution, m) && calculateWeight(newSolution) < calculateWeight(solution)) {
                solution = newSolution;
                improved = true;
                break;
            } else {
                ++it;
            }
        }
    }

    return solution;
}

// **GRASP principal** : combinaison de la construction et de la recherche locale
vector<int> GRASP(int m, int n, int maxIterations, double alpha) {
    vector<int> bestSolution;
    int bestWeight = numeric_limits<int>::max();

    for (int iter = 0; iter < maxIterations; ++iter) {
        cout << "It�ration " << iter + 1 << "/" << maxIterations << endl;

        // Phase de construction
        vector<int> constructedSolution = graspConstruct(m, n, alpha);

        // Phase de recherche locale
        vector<int> improvedSolution = localSearch(constructedSolution, m, n);

        int currentWeight = calculateWeight(improvedSolution);
        if (currentWeight < bestWeight) {
            bestWeight = currentWeight;
            bestSolution = improvedSolution;
            cout << "Nouvelle solution trouv�e avec poids = " << bestWeight << endl;
        }
    }

    return bestSolution;
}

// -----------------------------VNS + Grasp--------------------------------------

vector<int> hybridVNSWithGRASP(int m, int n, int maxIterations, int kMax, int localSearchAttempts, int graspIterations, double alpha) {
    cout << "D�marrage du VNS Hybride avec GRASP..." << endl;

    // �tape 1 : Utiliser GRASP pour g�n�rer une solution initiale
    vector<int> bestSolution = GRASP(m, n, graspIterations, alpha);
    int bestWeight = calculateWeight(bestSolution);

    cout << "Solution initiale trouv�e par GRASP, Poids : " << bestWeight << endl;

    int iteration = 0;
    mt19937 rng((unsigned int)chrono::steady_clock::now().time_since_epoch().count());

    while (iteration < maxIterations) {
        int k = 1;

        // Solution courante pour l'it�ration en cours
        vector<int> currentSolution = bestSolution;
        bool improvementFound = false;

        while (k <= kMax) {
            cout << "It�ration " << iteration + 1 << "/" << maxIterations << ", Voisinage k=" << k << endl;

            // �tape 2 : "Shaking" pour explorer un voisinage
            vector<int> shakenSolution = shakingVNS(currentSolution, k, m, n);

            // �tape 3 : Recherche locale sur la solution secou�e
            vector<int> improvedSolution = localSearchVNS(shakenSolution, m, n, localSearchAttempts);

            // �valuation de la solution am�lior�e
            int currentWeight = calculateWeight(improvedSolution);
            if (isFeasibleSolution(improvedSolution, m) && currentWeight < bestWeight) {
                bestSolution = improvedSolution;
                bestWeight = currentWeight;
                cout << "Am�lioration trouv�e ! Nouveau poids : " << bestWeight << endl;

                // Revenir au premier voisinage apr�s am�lioration
                k = 1;
                improvementFound = true;
                currentSolution = bestSolution;
                continue; // Recommencer avec k=1
            }

            k++; // Passer au prochain voisinage si pas d'am�lioration
        }

        if (!improvementFound) {
            cout << "Pas d'am�lioration � l'it�ration " << iteration + 1 << "." << endl;
        }

        iteration++;
    }

    cout << "Fin du VNS Hybride avec GRASP apr�s " << iteration << " it�rations." << endl;
    cout << "Meilleur poids trouv� : " << bestWeight << endl;
    return bestSolution;
}

//---------------------------------------VNS & Meta learning----------------------------


vector<int> vnsSearchMetaLearning(int m, int n, const vector<int>& initialSolution, int maxIterations, int kMax, int localSearchAttempts) {
    vector<int> bestSolution = initialSolution;
    int bestWeight = calculateWeight(bestSolution);

    mt19937 rng((unsigned int)chrono::steady_clock::now().time_since_epoch().count());
    vector<double> neighborhoodScores(kMax + 1, 1.0); // Initialisation des scores pour chaque voisinage
    double alpha = 0.1; // Taux d'apprentissage pour la mise � jour des scores

    int iteration = 0;

    cout << "===== D�MARRAGE DU VNS AVEC M�TA-LEARNING =====" << endl;
    cout << "Param�tres : maxIterations=" << maxIterations
         << ", kMax=" << kMax
         << ", localSearchAttempts=" << localSearchAttempts << endl;

    while (iteration < maxIterations) {
        cout << "\n--- It�ration " << iteration + 1 << "/" << maxIterations << " ---" << endl;

        int k = 1;
        bool improvementFound = false;
        vector<int> currentSolution = bestSolution;

        while (k <= kMax) {
            cout << "  [Voisinage k=" << k << "] S�lection des scores..." << endl;

            // S�lection du voisinage bas� sur les scores (roulette wheel selection)
            vector<double> cumulativeScores(kMax + 1, 0.0);
            for (int i = 1; i <= kMax; i++) {
                cumulativeScores[i] = cumulativeScores[i - 1] + neighborhoodScores[i];
            }
            double r = uniform_real_distribution<double>(0.0, cumulativeScores[kMax])(rng);
            for (int i = 1; i <= kMax; i++) {
                if (r <= cumulativeScores[i]) {
                    k = i;
                    break;
                }
            }
            cout << "    Voisinage s�lectionn� : k = " << k << " (score actuel = " << neighborhoodScores[k] << ")" << endl;

            // Appliquer le shaking
            cout << "    Application du shaking..." << endl;
            vector<int> shakenSolution = shakingVNS(currentSolution, k, m, n);
            cout << "    Taille de la solution secou�e : " << shakenSolution.size() << endl;

            // Appliquer la recherche locale
            cout << "    Recherche locale en cours..." << endl;
            vector<int> improvedSolution = localSearchVNS(shakenSolution, m, n, localSearchAttempts);
            int w = calculateWeight(improvedSolution);

            // V�rification de la faisabilit� et am�lioration
            if (isFeasibleSolution(improvedSolution, m)) {
                cout << "    Solution faisable obtenue avec poids = " << w << endl;
                if (w < bestWeight) {
                    cout << "    ** Am�lioration trouv�e ! ** Nouveau meilleur poids = " << w << endl;
                    bestSolution = improvedSolution;
                    bestWeight = w;
                    neighborhoodScores[k] += alpha; // R�compenser le voisinage k
                    improvementFound = true;
                    currentSolution = bestSolution;
                    cout << "    Mise � jour du score pour k=" << k << " (nouveau score = " << neighborhoodScores[k] << ")" << endl;
                    break; // Revenir au premier voisinage
                } else {
                    neighborhoodScores[k] *= (1 - alpha); // P�naliser l�g�rement si pas d'am�lioration
                    cout << "    Aucune am�lioration. Score du voisinage k=" << k << " r�duit � " << neighborhoodScores[k] << endl;
                }
            } else {
                cout << "    Solution non faisable. Passage au prochain voisinage." << endl;
            }

            k++;
        }

        if (!improvementFound) {
            cout << "  -> Aucune am�lioration trouv�e � cette it�ration." << endl;
        } else {
            cout << "  -> Solution am�lior�e, poids actuel = " << bestWeight << endl;
        }

        iteration++;
    }

    cout << "\n===== FIN DU VNS AVEC M�TA-LEARNING =====" << endl;
    cout << "Meilleur poids trouv� = " << bestWeight << endl;
    cout << "Taille de la meilleure solution : " << bestSolution.size() << " sous-ensembles" << endl;

    return bestSolution;
}
