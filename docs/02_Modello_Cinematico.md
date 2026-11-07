# FASE 3 — Modello cinematico differenziale

**Progetto:** PIANIFICAZIONE E CONTROLLO DI UN ROBOT MOBILE DIFFERENZIALE  
**Fonte:** [traccia ufficiale trascritta, §2 e §5](../TRACCIA_Robot_Mobile_Differenziale.md)  
**Stato:** implementazione e revisione statica completate per il perimetro cinematico; test predisposti ma **NON ESEGUITI**. Nessuna simulazione avviata.

Questo documento descrive i contratti effettivamente implementati. Il [documento di progettazione complessiva](01_Analisi_e_Progettazione.md) rimane il riferimento per i moduli futuri; la [checklist](Checklist_Requisiti.md) distingue implementazione, revisione statica e verifica esecutiva. I nomi e le firme effettive qui descritti prevalgono sui contratti concettuali precedenti.

## 1. Perimetro, ipotesi e convenzioni

Sono realizzati: configurazione della geometria, conversioni cinematiche, derivata dello stato, passo numerico RK4, propagazione analitica a comandi costanti, normalizzazione degli angoli e validazione. Non sono realizzati ambiente con ostacoli, collision checking, RRT, controllore completo, simulazione end-to-end, grafici o animazione.

Si assume:

- moto su piano XY fisso e orizzontale;
- due ruote motrici indipendenti, dello stesso raggio `r`, con rotolamento senza slittamento laterale o longitudinale;
- geometria rigida e distanza completa fra le ruote `d`;
- origine del robot nel punto medio dell'asse delle ruote;
- `theta` misurato dall'asse X in senso antiorario;
- velocità positiva di ciascuna ruota nel verso che fa avanzare quella ruota;
- velocità ruota ammesse anche negative; le funzioni non introducono limitazioni di marcia;
- nessun modello di accelerazioni, coppie, attrito, sensori, stima dello stato o rumore.

I limiti di velocità e l'ingombro del robot saranno applicati dai futuri moduli che generano e verificano i comandi. Il modello cinematico non modifica gli ingressi con saturazioni implicite. Il futuro pianificatore potrà limitare `v>=0`, ma questa è una restrizione del pianificatore, non della cinematica implementata.

## 2. Stato, ingressi, parametri e unità

\[
q=\begin{bmatrix}x\\y\\\theta\end{bmatrix}\in\mathbb{R}^2\times S^1,
\qquad
u=\begin{bmatrix}\omega_R\\\omega_L\end{bmatrix},
\qquad
\xi=\begin{bmatrix}v\\\omega\end{bmatrix}.
\]

| Grandezza | Significato | Forma nel codice | Unità |
| --- | --- | --- | --- |
| `q` | Posa del centro dell'asse ruote | `3×1` | m, m, rad |
| `u` | Velocità angolari destra, sinistra | `2×1` | rad/s, rad/s |
| `twist` | Velocità lineare e angolare del corpo | `2×1` | m/s, rad/s |
| `params.r` | Raggio comune ruote, strettamente positivo | Scalare | m |
| `params.d` | Distanza completa tra ruote, strettamente positiva | Scalare | m |
| `dt`, `duration` | Intervallo con comandi ruota costanti | Scalare non negativo | s |
| `dq` | Derivata temporale della posa | `3×1` | m/s, m/s, rad/s |

I parametri iniziali sono restituiti da `robot.config.defaultConfig()`:

| Campo | Valore | Origine |
| --- | --- | --- |
| `cfg.robot.r` | 0,05 m | Scelta progettuale, non assegnata dal docente |
| `cfg.robot.d` | 0,30 m | Scelta progettuale, non assegnata dal docente |
| `cfg.integration.dt` | 0,02 s | Passo suggerito da verificare, non valore già validato |

Sono modificabili dal chiamante senza stato globale. Le funzioni ricevono esplicitamente `params = cfg.robot`; `cfg` intero non è un sostituto di `params`. Campi aggiuntivi della struttura geometrica sono conservati dal validatore e ignorati dalle formule della fase corrente.

## 3. Cinematica diretta e inversa

### 3.1 Velocità delle ruote verso velocità del corpo

La traccia assegna

\[
v=\frac r2(\omega_R+\omega_L),\qquad
\omega=\frac rd(\omega_R-\omega_L).
\]

In forma matriciale:

\[
\xi=Bu,\qquad B=\begin{bmatrix}r/2&r/2\\r/d&-r/d\end{bmatrix}.
\]

La prima riga ha unità di lunghezza e produce m/s; la seconda contiene un rapporto di lunghezze e produce rad/s. Con ruote uguali si ha traslazione rettilinea; con ruote opposte si ha rotazione sul posto. Una ruota destra più veloce della sinistra genera `omega>0`, cioè una rotazione antioraria.

### 3.2 Inversione

Poiché `det(B) = -r^2/d`, per `r,d>0` la trasformazione è invertibile:

\[
\omega_R=\frac{v+(d/2)\omega}{r},\qquad
\omega_L=\frac{v-(d/2)\omega}{r}.
\]

Il fattore `d/2` compare soltanto nella conversione inversa; nella formula diretta della velocità angolare il denominatore è `d`, non `2*d`. L'ordine destra/sinistra è lo stesso in ogni funzione e test.

Per i parametri iniziali, i seguenti valori sono **oracoli calcolati algebricamente**, non risultati di un'esecuzione:

| `u` in rad/s | `v` in m/s | `omega` in rad/s | Interpretazione |
| --- | --- | --- | --- |
| `[4;4]` | 0,2 | 0 | Rettilineo |
| `[3;-3]` | 0 | 1 | Rotazione antioraria sul posto |
| `[6;2]` | 0,2 | 2/3 | Arco antiorario di raggio 0,3 m |
| `[0;6]` | 0,15 | -1 | Rotazione oraria con avanzamento |

## 4. Equazioni di moto e vincolo non olonomo

La funzione `robot.model.derivative` implementa

\[
\dot q=f(q,u)=\begin{bmatrix}v\cos\theta\\v\sin\theta\\\omega\end{bmatrix}.
\]

Il vincolo di assenza di velocità laterale è

\[
A(q)\dot q=0,\qquad A(q)=\begin{bmatrix}-\sin\theta&\cos\theta&0\end{bmatrix}.
\]

Infatti

\[
-\sin\theta\,v\cos\theta+\cos\theta\,v\sin\theta=0.
\]

Il sistema ha tre coordinate di posa e due ingressi; non ammette una velocità traslazionale arbitraria nel piano a orientamento fissato. A `theta=0`, una velocità `[0;1;0]` viola il vincolo. Spostamenti laterali complessivi richiedono una sequenza di rotazioni e traslazioni, non una velocità laterale istantanea.

La derivata è valutata sullo stato fornito; `theta` finito non deve essere già normalizzato. La componente `dtheta/dt` è una velocità e non viene sottoposta a wrapping. Il vincolo è soddisfatto analiticamente dal campo; il residuo calcolato dalle medesime formule non è, da solo, una verifica numerica indipendente. Per questo i test includono anche casi con endpoint e segni noti.

## 5. Integrazione numerica

### 5.1 Passo RK4 implementato

`robot.model.integrateRK4(q,u,dt,params)` effettua **un solo passo**, con `u` mantenuto costante. Non contiene un controllore, un ciclo di simulazione generale o una pianificazione.

Con `h=dt`:

\[
k_1=f(q,u),\qquad k_2=f(q+(h/2)k_1,u),
\]
\[
k_3=f(q+(h/2)k_2,u),\qquad k_4=f(q+hk_3,u),
\]
\[
q_{next}=q+\frac h6(k_1+2k_2+2k_3+k_4).
\]

Ogni `k_i` è `3×1` e ha le unità di una derivata; `h*k_i` è un incremento di stato. L'angolo iniziale viene normalizzato, gli angoli degli stadi intermedi rimangono continui e solo l'angolo dello stato finale viene nuovamente normalizzato. Questo evita di inserire un salto artificiale dentro la combinazione RK4. La posizione non viene normalizzata.

La durata zero è ammessa: dopo la validazione completa degli input si restituiscono posizione invariata e angolo normalizzato. Durate negative non sono ammesse: un moto all'indietro si esprime con comandi ruota negativi e durata positiva. Non si adatta automaticamente il passo.

### 5.2 Accuratezza e limiti

Per soluzioni regolari e sufficientemente risolte, RK4 ha errore locale di ordine quinto e globale di ordine quarto. Il codice non dichiara una stima certificata dell'errore né un passo sicuro per ogni comando. Un intervallo molto lungo con una grande rotazione può essere sottorisolto anche se tutti i valori rimangono finiti.

Il chiamante dovrà suddividere gli intervalli e verificare la sensibilità al passo. Se in futuro i comandi cambiano a tempi assegnati, gli step devono rispettare tali discontinuità: non mantenere un comando oltre il suo intervallo dichiarato. Con ingressi diversi a ogni passo la medesima API rimane utilizzabile.

I test di convergenza predisposti confrontano quattro e otto passi sulla stessa curva con lo stesso comando e lo stesso tempo finale. La riduzione attesa è circa un fattore 16 nel regime asintotico; il test richiede prudentemente un miglioramento superiore a 10, evitando una uguaglianza esatta dipendente da arrotondamenti. Questa aspettativa teorica non è un esito osservato.

## 6. Propagazione analitica a comandi costanti

`robot.model.propagateConstant` restituisce soltanto l'endpoint del moto per `u` costante. È realizzato ora perché appartiene al modello cinematico, fornisce un confronto per l'integratore e sarà riutilizzabile per gli archi del futuro RRT.

Per `omega != 0`, la soluzione chiusa è

\[
x_1=x_0+\frac v\omega[\sin(\theta_0+\omega\tau)-\sin\theta_0],
\]
\[
y_1=y_0-\frac v\omega[\cos(\theta_0+\omega\tau)-\cos\theta_0],\qquad
\theta_1=\theta_0+\omega\tau.
\]

Per `omega=0` si ha la traslazione `v*tau*[cos(theta_0);sin(theta_0)]`. Per evitare la divisione per una velocità angolare piccola, l'implementazione utilizza

\[
z=\omega\tau/2,\qquad
\Delta p=v\tau\,\operatorname{sinc}_0(z)
\begin{bmatrix}\cos(\theta_0+z)\\\sin(\theta_0+z)\end{bmatrix},
\]

dove `sinc_0(z)=sin(z)/z`, con limite 1 in zero. Per `abs(z)<1e-4` si usa `1-z^2/6+z^4/120`; il termine successivo ha modulo dell'ordine di `abs(z)^6/5040`. La soglia riguarda un angolo adimensionale, non una velocità con unità di misura. Non si richiama una funzione MATLAB `sinc`, evitando dipendenze e ambiguità di normalizzazione.

La propagazione gestisce rettilineo, curve, rotazione pura e comandi nulli con una sola espressione. Normalizza l'angolo iniziale e finale; durata zero ha la stessa semantica di RK4. I campioni intermedi potranno essere ottenuti chiamando la funzione dalla medesima posa iniziale per durate intermedie: questo documento non implementa un estrattore di traiettorie né un controllo delle collisioni.

**Un endpoint valido non dimostra che il moto intermedio sia libero da collisioni.** La verifica dell'intero arco, dell'ingombro e delle giunzioni sarà responsabilità dei moduli di geometria e pianificazione.

## 7. Normalizzazione angolare

`robot.model.normalizeAngle` calcola `atan2(sin(angle),cos(angle))` e rappresenta l'estremo `+pi` con `-pi`. L'intervallo di uscita è `[-pi,pi)`; sono accettati scalari e array non vuoti, preservandone le dimensioni. Non si richiede `wrapToPi`.

La normalizzazione rappresenta una classe di equivalenza modulo un giro. In prossimità di un taglio angolare, gli arrotondamenti possono produrre uno dei due lati numericamente vicini: i confronti fisici devono utilizzare una differenza angolare normalizzata. Non usare una differenza lineare fra `pi-epsilon` e `-pi+epsilon` come errore di orientamento.

Gli integratori restituiscono angoli normalizzati e non conservano il numero di giri. Una futura analisi delle rotazioni multiple dovrà registrare anche l'integrale di `omega` o un angolo non avvolto separato. Valori angolari enormi sono finiti ma possono aver già perso informazione nella rappresentazione floating point; il wrapping non ricostruisce l'informazione perduta.

## 8. Contratti, validazione ed errori

### 8.1 API pubbliche

| Funzione | Firma effettiva | Risultato |
| --- | --- | --- |
| Configurazione | `robot.config.defaultConfig()` | Struttura con `robot` e `integration` |
| Validazione geometria | `robot.model.validateParameters(params)` | Stessa struttura con `r,d` convertiti in double |
| Cinematica diretta | `robot.model.wheelsToBody(u,params)` | `twist`, colonna `2×1` |
| Cinematica inversa | `robot.model.bodyToWheels(twist,params)` | `u`, colonna `2×1` |
| Campo cinematico | `robot.model.derivative(q,u,params)` | `dq`, colonna `3×1` |
| Passo numerico | `robot.model.integrateRK4(q,u,dt,params)` | Posa finale, colonna `3×1` |
| Propagazione analitica | `robot.model.propagateConstant(q,u,duration,params)` | Posa finale, colonna `3×1` |
| Normalizzazione | `robot.model.normalizeAngle(angle)` | Angolo/array con dimensioni conservate |

Le funzioni accettano `double` e `single`, convertono in double e richiedono dati reali, finiti e non sparse. Si rifiutano vettori riga al posto di colonne, lunghezze errate, interi, logici, stringhe, input vuoti, numeri complessi, `NaN` e `Inf`. L'angolo accetta array di forma arbitraria purché non vuoti. I parametri devono essere una struttura scalare con `r,d` scalari strettamente positivi; la durata è uno scalare non negativo.

Per `single` il cast a double non recupera cifre già perse: le tolleranze dei test non presuppongono che `single(0.05)` sia identico a `double(0.05)`. Il contratto non copre array distribuiti o GPU, né è necessario usarli.

### 8.2 Errori identificabili

| Identificatore | Condizione |
| --- | --- |
| `RobotProject:InvalidParameters` | Struttura geometrica errata o `r,d` non validi |
| `RobotProject:InvalidVector` | Stato, comando o twist con tipo/dimensione/valori non validi |
| `RobotProject:InvalidDuration` | Durata non scalare, negativa, non finita o di tipo non supportato |
| `RobotProject:InvalidAngle` | Angolo/array non valido |
| `RobotProject:NumericalFailure` | Risultato o stadio intermedio non finito |
| `RobotProject:InvalidTestSelection` | Selezione della suite diversa da `all`, `unit`, `smoke` |
| `RobotProject:NoTests` | Suite selezionata senza test scoperti |
| `RobotProject:TestsNotPassed` | Test falliti o incompleti dopo l'esecuzione reale |

Gli helper sotto `robot.internal` centralizzano validazione delle colonne, durate e finitezza dei risultati. Non sono punti di ingresso applicativi. Le equazioni non vengono duplicate nei test come unico oracolo: sono usati anche endpoint con significato geometrico noto.

Valori positivi estremamente piccoli di `r,d` e ingressi molto grandi possono essere formalmente validi e produrre overflow: i risultati non finiti sono rifiutati. Non si dichiara protezione completa da underflow o perdita di precisione per scale arbitrarie. Gli usi previsti hanno geometria metrica e velocità della scala indicata nei documenti. I parametri fisicamente plausibili devono comunque essere scelti dal progettista.

## 9. Test predisposti e criteri di verifica

`tests/unit/test_kinematics.m` contiene i seguenti test basati su funzioni. `tests/smoke/test_project_setup.m` copre struttura e ingresso; il vecchio test iniziale è stato trasferito in questa cartella, senza conservarne una copia.

| Gruppo di test | Casi e criterio | Riferimento requisiti |
| --- | --- | --- |
| Conversioni note e round-trip | `[4;4]`, `[3;-3]`, `[6;2]`, comandi nulli e negativi; valori noti e recupero degli ingressi | T08–T10, T19; UT-M01, UT-M04 |
| Geometria configurabile | `r=0.1`, `d=0.5`, ingresso `[5;1]`: `v=0.3`, `omega=0.8` | UT-M04; K01 |
| Derivata e ordine ruote | Direzione a `pi/2`, una sola ruota attiva, segno orario/antiorario e uscita `3×1` | T07, T11; UT-M01 |
| Vincolo non olonomo | Diverse direzioni e controesempio di velocità laterale | T06; UT-M05 |
| Rettilineo | `q0=[1;2;pi/6]`, `[4;4]`, 2 s: spostamento di 0,4 m nella direzione iniziale | UT-M02; K05 |
| Rotazione pura | `[3;-3]`, durata `pi/2`: centro invariato e angolo `pi/2` | UT-M02; K05 |
| Curva | `[6;2]`, `3*pi/4` s: quarto di cerchio di raggio 0,3, endpoint `[0.3;0.3;pi/2]` | UT-M03; K05 |
| Curva oraria, retromarcia e posa generale | Endpoint noti con segni scambiati o entrambi negativi; quarto di cerchio ruotato e traslato | UT-M03 |
| Convergenza RK4 | Stesso arco con quattro e otto passi; errore XY fine inferiore a un decimo del grossolano | UT-S01–S02 |
| Comandi e durata nulli | Stato fermo e wrapping coerente anche con durata zero | UT-M03, UT-S03 |
| Angoli | Intervallo, dimensioni array, equivalenza trigonometrica, angoli piccoli e attraversamento del taglio | UT-C04 limitatamente al wrapping; K06 |
| Limite quasi rettilineo | Ruote quasi uguali e continuità della propagazione analitica | UT-M03 |
| Tipi e dimensioni | `single` accettati con uscita double, righe/lunghezze errate e valori non finiti rifiutati | UT-D01; K07 |
| Parametri e durate non validi | Zero/negativi, strutture incomplete, array, complessi, sparse e tipi errati; durata zero non salta la validazione | UT-M04, UT-D01; K07 |
| Overflow | Risultati non finiti in conversione o propagazione generano errore esplicito | K07 |
| Struttura e portabilità | `main` da altra cartella, file necessari, selezione suite non valida | UT-D02 limitatamente allo scope corrente |

Tolleranze predisposte:

- Identità e valori noti: generalmente `AbsTol=1e-12` nella scala dei casi di test.
- Rettilineo e rotazione con passi ripetuti: `AbsTol=1e-11` per accumulo floating point.
- Curve numeriche: `AbsTol=1e-9` con 120 passi sul quarto di cerchio.
- Limite quasi rettilineo: `AbsTol=1e-9` rispetto al limite rettilineo, includendo il piccolo comando differenziale deliberato.
- Angolo `1e-20`: tolleranza `1e-30` per verificare che la normalizzazione non lo annulli con una somma a `pi`.

Queste soglie sono criteri progettati per casi nominali specifici, non tolleranze certificate da un'esecuzione. Il test di round-trip non sostituisce i casi noti, perché due conversioni errate ma coerenti potrebbero annullarsi. Il test di convergenza usa un endpoint geometrico noto, senza prendere il propagatore implementato come unica verità numerica.

Le brevi integrazioni nei test sono verifiche del modello a comandi prescritti, non il simulatore completo. Nessun test è stato avviato, nessuna soglia è stata confermata empiricamente.

## 10. Revisione statica effettuata e verifiche residue

| Area | Controllo completato mediante lettura | Verifica esecutiva ancora richiesta |
| --- | --- | --- |
| Matematica | Derivazione dell'inversa, determinante, segni e limite rettilineo | Test su valori noti e round-trip |
| Dimensioni e unità | `2×1 -> 2×1`, `3×1 -> 3×1`, scalari e unità SI | Test di dimensioni e input errati |
| Vincolo non olonomo | Sostituzione simbolica e controesempio laterale | Residui e traiettorie numeriche |
| Sintassi e chiamate | Corrispondenza nomi/firme, operatori, blocchi, dipendenze di package e uscite in ogni ramo | Parser e Code Analyzer MATLAB Online |
| RK4 | Quattro stadi, pesi 1-2-2-1, comando costante e wrapping solo ai confini | Convergenza e accuratezza |
| Propagatore | Formula midpoint/sinc, sviluppo vicino a zero, durata nulla | Arco, rotazione, limite e confronto con RK4 |
| Input e overflow | Guardie con short-circuit prima degli accessi; validazione anche con durata zero | Test negativi e identificatori effettivi |
| Portabilità | `fullfile`, sorgenti separati, path relativo ai file, ripristino nei test | Caricamento e suite in MATLAB Online |
| Consegna | Regole PDF/repository aggiornate e assenza di pubblicazione | Link/versione reali dopo la pubblicazione separata |

Non è stato utilizzato MATLAB locale, Octave o un altro interprete per eseguire o tradurre il modello. Non è stato installato alcun software. La revisione statica manuale riduce gli errori ma non dimostra che il parser o i test di MATLAB accetteranno tutti i file senza correzioni.

Per la verifica futura in MATLAB Online: caricare la cartella, chiamare `main`, usare il Code Analyzer sui sorgenti e avviare `run_tests('smoke')` e `run_tests('unit')`. Conservare release, data, versione dei sorgenti e output effettivo della suite; aggiornare soltanto allora la colonna di verifica esecutiva nella checklist. L'assegnazione dei risultati del lanciatore si interrompe in caso di errore dopo che l'esito è stato mostrato.

## 11. Preparazione della successiva pianificazione

1. Consolidare la cinematica mediante la verifica reale in MATLAB Online e correggere eventuali problemi prima di usarla come base del pianificatore.
2. Definire ambiente e ingombro: rettangolo, ostacoli circolari/poligonali, disco conservativo, margine e pose iniziale/finale valide.
3. Implementare distanze e controllo continuo/conservativo delle collisioni lungo le primitive, con test geometrici indipendenti. Non usare il solo endpoint restituito da `propagateConstant` come prova di sicurezza.
4. Introdurre il RRT progettato: ogni arco deve conservare posa iniziale, comando ruote, durata e posa finale coerente. Riutilizzare `propagateConstant`, senza sostituire il moto con interpolazioni arbitrarie in XY.
5. Applicare esplicitamente limiti ruota e vincolo accoppiato su `v,omega`; registrare i comandi effettivi. I moduli cinematici rimangono privi di clipping implicito.
6. Gestire orientamento finale, connessione rotazione-traslazione-rotazione, seed e budget. Distinguere fallimento della ricerca da dimostrazione di impossibilità.

Il controllore completo e il simulatore end-to-end saranno introdotti nelle rispettive fasi. La relazione finale conserverà una spiegazione autonoma di questo modello e dei risultati reali, con link al repository; non richiederà l'inclusione integrale dei sorgenti `.m` salvo nuova indicazione del docente.

## 12. Riferimenti MATLAB per la verifica successiva

La struttura dei test segue il framework standard documentato da MathWorks: [`functiontests`](https://www.mathworks.com/help/matlab/ref/functiontests.html) raccoglie le funzioni locali di test e le relative fixture. La verifica di finitezza usa [`isfinite` per dati numerici MATLAB](https://www.mathworks.com/help/matlab/ref/double.isfinite.html), senza dati simbolici o toolbox.

Il Code Analyzer potrà essere consultato nell'Editor di MATLAB Online o mediante [`checkcode`](https://www.mathworks.com/help/matlab/ref/checkcode.html), che segnala problemi potenziali e messaggi di parsing. La consultazione della documentazione ufficiale non equivale all'esecuzione di tali controlli sui sorgenti del progetto; questa resta da effettuare.
