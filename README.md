# Évaluation expérimentale — Fusion multi-capteurs (ROS2 Jazzy / CARLA / robot_localization)

## Cadre commun (tous les runs)

- **Référence (Ground Truth / GT)** : `/carla/hero/odometry`
- **Estimation** : `/odometry/filtered` (sortie de `robot_localization`)
- **Erreurs calculées** :
  - `ex = x_est − x_gt` (m)
  - `ey = y_est − y_gt` (m)
  - `e2d = sqrt(ex² + ey²)` (m)
  - `eyaw = yaw_est − yaw_gt` (rad)
- **Indicateurs globaux** : `RMSE_x`, `RMSE_y`, `RMSE_2D`, `RMSE_yaw`

> ⚠️ **Limite méthodologique importante**  
> Les paires GT/EST sont constituées via un appariement **par temps de réception (arrival time)** et non par `header.stamp`.  
> Cela introduit potentiellement une composante d’erreur liée au **désalignement temporel** (latence/jitter) entre flux, visible sous forme de **pics** sur `e2d`, même si la trajectoire est correctement suivie.

---

## RUN1 — EKF + Odom (Baseline)

### 1) Contexte et objectif
Ce run sert de **référence** : le filtre EKF utilise uniquement l’odom (`/carla/hero/odometry`).  
Objectif : établir un niveau de performance “baseline” et valider la chaîne de comparaison.

### 2) Analyse des figures

#### 2.1 Trajectoire 2D (GT vs EST)
<img width="1024" height="768" alt="RUN1_EKF_odom_traj2d" src="https://github.com/user-attachments/assets/e381ea1c-7359-49e0-b982-6c42f6333bc4" />

- **Figure** : `RUN1_EKF_odom_traj2d.png`
- 
**Interprétation** : trajectoires **quasi superposées** sur la séquence. Suivi global excellent, pas de divergence visible.

#### 2.2 Erreurs position (ex, ey, e2d)

<img width="1024" height="768" alt="RUN1_EKF_odom_errors_pos" src="https://github.com/user-attachments/assets/003c6641-6fbd-4c61-ae32-0699c64406cd" />

- **Figure** : `RUN1_EKF_odom_errors_pos.png`  
**Interprétation** : erreurs globalement faibles, mais présence de **pics** sur `e2d` (jusqu’à ~0.4 m).  
Compte tenu du bon suivi 2D, ces pics sont compatibles avec un **désalignement temporel** dû à l’appariement arrival-time.

#### 2.3 Erreur yaw

<img width="1024" height="768" alt="RUN1_EKF_odom_error_yaw" src="https://github.com/user-attachments/assets/88a8b67b-fd4e-43e7-88ea-cc67829e0e89" />

- **Figure** : `RUN1_EKF_odom_error_yaw.png`  
**Interprétation** : erreur yaw faible et stable, cohérente avec l’odom.

#### 2.4 Histogramme e2d (dispersion)

<img width="1024" height="768" alt="RUN1_EKF_odom_hist_e2d" src="https://github.com/user-attachments/assets/a1356f10-7caf-40b7-95d2-29c015f3920d" />

- **Figure** : `RUN1_EKF_odom_hist_e2d.png`  
**Interprétation** : masse près de 0, avec une **queue** jusqu’à ~0.4–0.45 m, correspondant aux pics observés.

### 3) Métriques (RUN1)
- **Fichier metrics** : `RUN1_EKF_odom_metrics.txt`

| Samples | Durée (s) | RMSE_x (m) | RMSE_y (m) | RMSE_2D (m) | RMSE_yaw (rad) | RMSE_yaw (°) |
|---:|---:|---:|---:|---:|---:|---:|
| 240 | 33.101 | 0.1103 | 0.1075 | 0.1540 | 0.0360 | 2.06 |

### 4) Conclusion RUN1
RUN1 valide la baseline :
- suivi position excellent,
- meilleur yaw parmi les runs,
- erreurs `e2d` dominées par quelques pics (souvent liés à la synchro arrival-time).

---

## RUN2 — EKF + Odom + IMU

### 1) Contexte et objectif
Ajout de l’IMU (`/carla/hero/imu`) en plus de l’odom.  
Objectif : mesurer l’impact de l’IMU sur la précision position et l’orientation.

### 2) Analyse des figures

#### 2.1 Trajectoire 2D (GT vs EST)

<img width="1024" height="768" alt="RUN2_EKF_odom_imu_traj2d" src="https://github.com/user-attachments/assets/c0d9a939-857c-43b6-b1d4-e199a6dafd57" />

- **Figure** : `RUN2_EKF_odom_imu_traj2d.png`  
**Interprétation** : superposition GT/EST toujours très forte. Pas d’amélioration visible par rapport à RUN1.

#### 2.2 Erreurs position (ex, ey, e2d)

<img width="1024" height="768" alt="RUN2_EKF_odom_imu_errors_pos" src="https://github.com/user-attachments/assets/df73c765-55b9-4d99-9293-91f615705444" />

- **Figure** : `RUN2_EKF_odom_imu_errors_pos.png`  
**Interprétation** : profil proche de RUN1. Pas de réduction nette de `e2d` sur cette séquence.

#### 2.3 Erreur yaw

<img width="1024" height="768" alt="RUN2_EKF_odom_imu_error_yaw" src="https://github.com/user-attachments/assets/b48ab1ff-7eb5-4737-a322-3fef611bbf80" />

- **Figure** : `RUN2_EKF_odom_imu_error_yaw.png`  
**Interprétation** : forte augmentation de l’erreur yaw. Cela suggère une incohérence **IMU ↔ repère véhicule** (offset/TF) ou une fusion d’orientation trop contraignante.

#### 2.4 Histogramme e2d

<img width="1024" height="768" alt="RUN2_EKF_odom_imu_hist_e2d" src="https://github.com/user-attachments/assets/13e61689-99ec-4d31-8183-81577291e975" />

- **Figure** : `RUN2_EKF_odom_imu_hist_e2d.png`  
**Interprétation** : dispersion similaire à RUN1.

### 3) Métriques (RUN2)
- **Fichier metrics** : `RUN2_EKF_odom_imu_metrics.txt`

| Samples | Durée (s) | RMSE_x (m) | RMSE_y (m) | RMSE_2D (m) | RMSE_yaw (rad) | RMSE_yaw (°) |
|---:|---:|---:|---:|---:|---:|---:|
| 236 | 32.485 | 0.1117 | 0.1071 | 0.1548 | 0.1536 | 8.80 |

### 4) Conclusion RUN2
- **Position** : pas de gain (RMSE_2D ≈ RUN1)  
- **Yaw** : dégradation majeure (≈ ×4)  
→ l’IMU nécessite une **reconfiguration** (repères/offset ou fusion gyro-only).

---

## RUN3 — EKF + Odom + IMU + GNSS (navsat)

### 1) Contexte et objectif
Ajout GNSS via chaîne classique :
- GNSS + IMU (+ odom local) → `navsat_transform_node` → `/odometry/gps`
- EKF fuse odom + IMU + GPS → `/odometry/filtered`

Objectif : vérifier si le GNSS apporte une stabilité globale et améliore la position.

### 2) Analyse des figures

#### 2.1 Trajectoire 2D (GT vs EST)

<img width="1024" height="768" alt="RUN3_EKF_odom_imu_gnss_traj2d" src="https://github.com/user-attachments/assets/db0f97f2-13f2-4cfe-a03a-da37d681aa3b" />


- **Figure** : `RUN3_EKF_odom_imu_gnss_traj2d.png`  
**Interprétation** : trajectoire superposée, similaire aux runs précédents. Pas de gain visible sur cette séquence.

#### 2.2 Erreurs position (ex, ey, e2d)

<img width="1024" height="768" alt="RUN3_EKF_odom_imu_gnss_errors_pos" src="https://github.com/user-attachments/assets/7dbb322a-d012-46ab-8786-b20fa4b7ae80" />

- **Figure** : `RUN3_EKF_odom_imu_gnss_errors_pos.png`  
**Interprétation** : profil proche de RUN1/RUN2. GNSS ne réduit pas `e2d` ici.

#### 2.3 Erreur yaw

<img width="1024" height="768" alt="RUN3_EKF_odom_imu_gnss_error_yaw" src="https://github.com/user-attachments/assets/a102eab1-e60e-4a29-85b6-231a3fdd6942" />

- **Figure** : `RUN3_EKF_odom_imu_gnss_error_yaw.png`  
**Interprétation** : erreur yaw similaire à RUN2. GNSS ne corrige pas yaw (attendu).

#### 2.4 Histogramme e2d

<img width="1024" height="768" alt="RUN3_EKF_odom_imu_gnss_hist_e2d" src="https://github.com/user-attachments/assets/2361f2c6-7db8-4acd-9cc8-d57f73b7030e" />

- **Figure** : `RUN3_EKF_odom_imu_gnss_hist_e2d.png`  
**Interprétation** : dispersion comparable aux runs précédents.

### 3) Métriques (RUN3)
- **Fichier metrics** : `RUN3_EKF_odom_imu_gnss_metrics.txt`

| Samples | Durée (s) | RMSE_x (m) | RMSE_y (m) | RMSE_2D (m) | RMSE_yaw (rad) | RMSE_yaw (°) |
|---:|---:|---:|---:|---:|---:|---:|
| 228 | 31.359 | 0.1119 | 0.1151 | 0.1605 | 0.1533 | 8.78 |

### 4) Conclusion RUN3
- GNSS ne montre pas de gain visible en position sur cette séquence (odom très précise + métrique arrival-time).
- yaw reste dégradé car l’origine du problème est liée à l’IMU.
- RUN3 valide néanmoins la faisabilité de la chaîne GNSS (topics + pipeline + logs).

---

## Synthèse globale (comparaison)

### Tableau comparatif (principaux indicateurs)

| Run | Fusion | Samples | Durée (s) | RMSE_2D (m) | RMSE_yaw (rad) | Conclusion principale |
|---|---|---:|---:|---:|---:|---|
| RUN1 | Odom | 240 | 33.101 | 0.1540 | 0.0360 | Meilleur yaw, baseline |
| RUN2 | Odom+IMU | 236 | 32.485 | 0.1548 | 0.1536 | Yaw dégradé, pas de gain position |
| RUN3 | Odom+IMU+GNSS | 228 | 31.359 | 0.1605 | 0.1533 | GNSS sans gain ici, yaw toujours dégradé |

---

## Discussion générale (points à défendre)
1) **Position quasi identique sur les 3 runs** : odom CARLA domine la fusion, et les pics `e2d` sont amplifiés par l’appariement arrival-time.
2) **Yaw fortement dégradé dès l’ajout IMU** : probable incohérence repère/offset IMU ou fusion orientation IMU inadaptée.
3) **GNSS n’améliore pas sur cette séquence** : effet masqué par la qualité de l’odom ; l’intérêt apparaîtra davantage avec odom bruitée et covariances réalistes.

---

## Figures
- `RUN1_EKF_odom_traj2d.png`
- `RUN1_EKF_odom_errors_pos.png`
- `RUN1_EKF_odom_error_yaw.png`
- `RUN1_EKF_odom_hist_e2d.png`

- `RUN2_EKF_odom_imu_traj2d.png`
- `RUN2_EKF_odom_imu_errors_pos.png`
- `RUN2_EKF_odom_imu_error_yaw.png`
- `RUN2_EKF_odom_imu_hist_e2d.png`

- `RUN3_EKF_odom_imu_gnss_traj2d.png`
- `RUN3_EKF_odom_imu_gnss_errors_pos.png`
- `RUN3_EKF_odom_imu_gnss_error_yaw.png`
- `RUN3_EKF_odom_imu_gnss_hist_e2d.png`

## Chaîne temps réel (ROS2) — Démo live

### Objectif
Passer progressivement de l’analyse hors-ligne (CSV) à une chaîne temps réel basée sur les topics ROS2 :  
`/carla/hero/odometry` (GT) + `/odometry/filtered` (EKF) → calcul des erreurs en ligne → publication de métriques → affichage live.

### Pipeline
1. **Lecture rosbag** avec temps simulé (`/clock`)
2. **EKF (robot_localization)** : estimation publiée sur `/odometry/filtered`
3. **Comparateur online** : souscription GT + estimation, calcul des erreurs instantanées et RMSE glissante, publication sur `/metrics/*`
4. **Visualisation live** : plotting en temps réel via matplotlib



# EKF (Extended Kalman Filter) — Prédiction / Correction (concept clé)

Cette section résume le fonctionnement d’un **EKF** tel qu’utilisé dans `robot_localization`.  
L’objectif est de comprendre **ce que fait le filtre** (au-delà des courbes RMSE), afin de relier correctement la théorie (**Q / R / P0**) aux résultats expérimentaux (erreurs, stabilité, latence).

---

## 1) Étape de prédiction (*Prediction*)

L’EKF “fait avancer” son estimation **dans le temps**, même lorsqu’aucune nouvelle mesure n’arrive.

### 1.1 État estimé
Le filtre maintient un **vecteur d’état** \(x\) (ce que le filtre estime).  
Dans `robot_localization`, l’état peut contenir (selon la config 2D/3D) :
- position \((x, y, z)\)
- vitesses \((v_x, v_y, v_z)\)
- orientation \((roll, pitch, yaw)\)
- vitesses angulaires \((\omega_x, \omega_y, \omega_z)\)
- (et éventuellement d’autres composantes selon les paramètres)

### 1.2 Modèle de mouvement
Entre deux instants \(t_k\) et \(t_{k+1}\), l’EKF suppose un **modèle de mouvement** :  
> *“si je me déplace à telle vitesse et je tourne à tel taux, ma position et mon orientation évoluent ainsi.”*

### 1.3 Équations (version simplifiée)
**État prédit :**
\[
x_{k|k-1} = f(x_{k-1|k-1}, u_k)
\]

**Covariance prédite :**
\[
P_{k|k-1} = F_k\,P_{k-1|k-1}\,F_k^T + Q
\]

Où :
- \(P\) : covariance de l’état (niveau d’incertitude sur l’estimation)
- \(Q\) : covariance de processus (*process noise*) — incertitude du modèle (dynamique non modélisée, glissement, approximations)
- \(F_k\) : Jacobien du modèle \(f\) (linéarisation, car l’EKF traite des systèmes non-linéaires)

**Interprétation intuitive :**
> “Je prédis où je devrais être maintenant, et j’augmente mon incertitude selon \(Q\).”

---

## 2) Étape de correction (*Update / Correction*)

Lorsque qu’une mesure capteur arrive (ex. odom, IMU, GNSS), l’EKF corrige la prédiction.

### 2.1 Mesure et modèle de mesure
Le filtre compare :
- la mesure réelle du capteur \(z_k\)
- la mesure attendue selon l’état prédit \(h(x_{k|k-1})\)

### 2.2 Équations (version simplifiée)
**Innovation (résidu) :**
\[
y_k = z_k - h(x_{k|k-1})
\]

**Gain de Kalman :**
\[
K_k = P_{k|k-1}\,H_k^T\,(H_k\,P_{k|k-1}\,H_k^T + R)^{-1}
\]

**Mise à jour de l’état :**
\[
x_{k|k} = x_{k|k-1} + K_k\,y_k
\]

**Mise à jour de la covariance :**
\[
P_{k|k} = (I - K_k\,H_k)\,P_{k|k-1}
\]

Où :
- \(R\) : covariance de mesure (*measurement noise*) — confiance dans le capteur
- \(H_k\) : Jacobien du modèle de mesure \(h(.)\)

**Interprétation intuitive :**
> “Je corrige ma prédiction avec la mesure, et je pondère la correction selon la confiance capteur \(R\) et ma propre incertitude \(P\) (influencée par \(Q\)).”

---

## 3) Pourquoi c’est important pour nos résultats (lien direct avec RMSE / courbes)

Les réglages \(Q\), \(R\) et la covariance initiale \(P_0\) influencent directement :
- l’amplitude des corrections
- la stabilité / réactivité
- le niveau de bruit sur l’estimation
- la vitesse de convergence au démarrage

### 3.1 Effet de \(Q\) (process noise)
- Si \(Q\) **augmente** :
  - le filtre considère que son modèle est moins fiable
  - il devient plus “réactif” aux mesures (corrige davantage)
  - risque : estimation plus “bruitée”

### 3.2 Effet de \(R\) (measurement noise)
- Si \(R\) **augmente** pour un capteur :
  - le filtre fait moins confiance à ce capteur
  - ce capteur influence moins l’estimation finale

### 3.3 Effet de \(P_0\) (incertitude initiale)
- Si \(P_0\) est **grand** :
  - convergence plus rapide au début (le filtre accepte de grosses corrections)
  - risque : transitoire plus instable/oscillant

---

## 4) Formulation “simple” attendue dans un rapport

> “L’EKF alterne une étape de prédiction basée sur un modèle de mouvement (propagation de l’état et de la covariance via \(Q\)), puis une étape de correction lors de l’arrivée des mesures capteurs, où le gain de Kalman pondère la correction selon la covariance de mesure \(R\) et l’incertitude courante \(P\).”

*(Dans RUN1, la correction est déclenchée uniquement par l’odom ; dans RUN2/RUN3, d’autres mesures peuvent intervenir selon la configuration.)*
