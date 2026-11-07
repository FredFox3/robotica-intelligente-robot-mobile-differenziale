# TRACCIA UFFICIALE — PIANIFICAZIONE E CONTROLLO DI UN ROBOT MOBILE DIFFERENZIALE

**Origine:** `ELABORATO_DEL_2025_12_17_16_10_33.pdf` (4 pagine).

> Trascrizione strutturata del documento fornito dal docente. Le formule sono riscritte in notazione leggibile. Le note fra parentesi quadre sono chiarimenti editoriali, non requisiti aggiuntivi.

## 1. Descrizione del progetto

Si consideri un robot mobile differenziale che si muove nel piano bidimensionale XY, caratterizzato da due ruote motrici indipendenti.

Lo scopo del progetto è progettare un sistema completo di:
1. modellazione cinematica del robot;
2. pianificazione del moto in un ambiente con ostacoli;
3. controllo del moto lungo il percorso pianificato;
4. simulazione e visualizzazione in MATLAB.

**Vincolo esplicito:** il robot deve muoversi utilizzando esclusivamente il modello differenziale, sia in fase di pianificazione che di controllo.

## 2. Modello del robot

### 2.1 Modello cinematico differenziale

Stato del robot: `q = [x, y, theta]^T`.

Ingressi:
- velocità angolare ruota destra: `omega_R`;
- velocità angolare ruota sinistra: `omega_L`.

Relazioni cinematiche:
- `v = (r / 2) * (omega_R + omega_L)`
- `omega = (r / d) * (omega_R - omega_L)`

Equazioni di moto:
- `dx/dt = v * cos(theta)`
- `dy/dt = v * sin(theta)`
- `dtheta/dt = omega`

[La traccia non fornisce valori numerici per r o d: sceglierli esplicitamente e documentarli.]

## 3. Ambiente e rappresentazione degli ostacoli

L'ambiente di lavoro è un piano rettangolare che contiene:
- ostacoli circolari o poligonali;
- configurazione iniziale `q_0`;
- configurazione finale `q_f`.

Gli ostacoli devono essere modellati geometricamente e visualizzati.

## 4. Pianificazione del moto (Path Planning)

**Implementare almeno UNO dei seguenti metodi.** Non è necessario implementarli tutti.

### 4.1 Cell Decomposition con Retraction

Obiettivo: pianificare un percorso collision-free nello spazio di lavoro.

Richieste:
1. Suddividere l'ambiente in celle libere e occupate.
2. Costruire il grafo di adiacenza delle celle libere.
3. Applicare una mappa di retraction che riduca ogni cella a un nodo o segmento centrale.
4. Trovare un percorso tra inizio e fine usando un algoritmo di ricerca (es. Dijkstra).

Output: sequenza di waypoint `(x_i, y_i)`.

### 4.2 Metodi probabilistici (PRM o RRT)

Obiettivo: generare un grafo di configurazioni valide in modo stocastico.

Richieste:
1. Generare configurazioni casuali collision-free.
2. Collegare i nodi con archi validi.
3. Cercare un percorso da `q_0` a `q_f`.
4. Estrarre una traiettoria planare continua.

Output: percorso come sequenza di configurazioni o punti nel piano.

### 4.3 Artificial Potential Fields

Obiettivo: guidare il robot tramite campi di attrazione e repulsione.

Richieste:
1. Definire potenziale attrattivo verso il goal e potenziali repulsivi dagli ostacoli.
2. Calcolare il gradiente del potenziale totale.
3. Utilizzare il gradiente per generare una direzione di moto.
4. Discutere problemi di minimi locali.

Output: traiettoria continua nel piano.

## 5. Controllo del robot differenziale

Indipendentemente dal metodo di pianificazione scelto, lo studente deve:
1. Convertire il percorso pianificato in waypoint oppure riferimento continuo.
2. Progettare un controllore cinematico del tipo:
   - `v = k_rho * rho`
   - `omega = k_alpha * alpha`
3. Convertire `v, omega` nelle velocità delle ruote `omega_R, omega_L`.
4. Simulare il moto con integrazione numerica.

## 6. Simulazione e risultati

Lo studente deve produrre:
- traiettoria pianificata;
- traiettoria eseguita;
- errore di tracking;
- andamento delle velocità delle ruote;
- animazione del moto del robot.

## 7. Valutazione

| Criterio | Peso |
|---|---:|
| Modello differenziale corretto | 20% |
| Pianificazione del moto | 20% |
| Controllo del robot | 20% |
| Analisi dei risultati | 20% |
| Qualità codice e animazione | 20% |

## Note sulla trascrizione

- I pedici del PDF sono stati normalizzati in notazione testuale (`R`, `L`, `0`, `f`, `rho`, `alpha`), preservando il significato delle formule.
- La traccia non specifica dimensioni dell'ambiente, parametri del robot, ostacoli numerici, guadagni del controllore o condizioni iniziali numeriche.
- Non assumere che un percorso geometrico collision-free sia automaticamente compatibile con la cinematica differenziale: verificare tale vincolo nell'implementazione.
