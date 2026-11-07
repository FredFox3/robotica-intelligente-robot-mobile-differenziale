# PIANIFICAZIONE E CONTROLLO DI UN ROBOT MOBILE DIFFERENZIALE

Elaborato universitario basato sulla [traccia ufficiale trascritta](TRACCIA_Robot_Mobile_Differenziale.md). **FASE 7 — Audit statico finale:** revisione dei 36 file MATLAB completata e correzioni mirate applicate. Programma e test **non ancora eseguiti**. Resoconto nella [checklist, §8](docs/Checklist_Requisiti.md#8-audit-statico-finale-fase-7).

Sono disponibili cinematica, RRT differenziale, controllo a waypoint, simulazione RK4, riepilogo numerico, confronto delle traiettorie, grafici di tracking e velocità ruota, animazione e salvataggio PNG. Non sono disponibili risultati effettivi finché non sarà eseguita la prova in MATLAB Online.

## Ambiente

- Linguaggio target: MATLAB standard, compatibile con una normale installazione desktop e con MATLAB Online. La prova reale prevista resta su **MATLAB Online**.
- Nessuna esecuzione locale durante lo sviluppo, nessun uso di GNU Octave o installazione di software/ambienti.
- Il codice utilizza MATLAB standard, compatibile con MATLAB Online Basic, senza Robotics System Toolbox o altri toolbox aggiuntivi.
- Percorsi costruiti con `fullfile` e posizione dei sorgenti, codice indipendente dal sistema operativo.
- I test usano il framework standard `matlab.unittest`; nessuna dipendenza da MATLAB Test o esecuzione parallela.

Le istruzioni per chi modifica il progetto sono in [AGENTS.md](AGENTS.md).

## Organizzazione e stato

```text
main.m                             setup senza esecuzione; 'run' per la pipeline completa
run_tests.m                        lanciatore suite all/unit/smoke
src/+robot/
    +config/                       parametri, scenario e validazione degli input
    +model/                        conversioni, derivata, RK4, angoli,
                                   propagazione analitica, validazione geometria
    +internal/                     controlli condivisi sugli input e sui risultati
    +geometry/                     clearance del disco e verifica degli archi
    +planning/                     RRT, connessione al goal e ricostruzione
    +reference/                    riferimento verificato e distanza dal tratto attivo
    +control/                      legge cinematica, modalità e validazione parametri
    +simulation/simulate.m         integrazione RK4, sicurezza e log essenziali
    +analysis/                     validazione log e riepilogo numerico
    +visualization/                ambiente, grafici comparativi, PNG e animazione
tests/
    smoke/test_project_setup.m     struttura, ingresso e percorsi
    unit/test_kinematics.m         test cinematici con oracoli analitici
    unit/test_planning.m           test numerici di geometria e pianificazione
    unit/test_control_simulation.m test numerici del controllo e del moto eseguito
    unit/test_analysis.m           verifiche numeriche su log effettivi e riepiloghi
docs/
    01_Analisi_e_Progettazione.md   progetto complessivo e tracciabilità
    02_Modello_Cinematico.md       modello e API effettivamente implementati
    Checklist_Requisiti.md         implementazione/revisione/esecuzione separate
.gitignore                         file temporanei e impostazioni locali
```

`results/` ospiterà i risultati reali e `report/` il materiale della relazione quando prodotti. Le cartelle non vengono create vuote. Non ignorare indiscriminatamente tutti i `.mat`, grafici o PDF: i risultati selezionati e la relazione devono poter essere versionati.

| Ambito | Implementazione | Revisione statica | Verifica esecutiva |
| --- | --- | --- | --- |
| Parametri, conversioni, derivata, angoli | Disponibile | Completata a lettura | Da effettuare in MATLAB Online |
| Passo numerico RK4 e propagatore costante | Disponibile | Formule e flusso controllati | Da effettuare in MATLAB Online |
| Test cinematici e di struttura | Predisposti | Revisionati | NON ESEGUITI |
| Ambiente, collisioni, RRT e ricostruzione | Disponibili | Formule, contratti e terminazioni controllati | Da effettuare in MATLAB Online |
| Grafico del piano | Disponibile su chiamata esplicita | Chiamate e dati controllati a lettura | Non generato/verificato |
| Riferimento, controllore e simulatore | Disponibili | Formule, guardie, log e terminazioni revisionati | Test NON ESEGUITI |
| Grafici, animazione, PNG e riepilogo | Disponibili su esecuzione esplicita | Contratti, array e calcoli revisionati | Nessun dato o grafico prodotto |
| PDF finale | Struttura definita | Regole aggiornate | Da completare dopo le esecuzioni |

La revisione statica non equivale al controllo del parser/Code Analyzer MATLAB né al superamento dei test. Non sono state eseguite simulazioni o test durante lo sviluppo di questa fase.

## Utilizzo successivo in MATLAB Online

Caricare la cartella completa in MATLAB Drive e aprirla in MATLAB Online. Dalla cartella radice:

```matlab
project = main();
params = project.config.robot;
```

`main()` e `main('setup')` aggiungono al path la sola cartella `src/` e restituiscono configurazione e stato, senza test, simulazioni o file di output. I valori iniziali, scelti per il progetto e non assegnati dalla traccia, sono `r = 0.05 m`, `d = 0.30 m`, passo suggerito `dt = 0.02 s`. Si possono modificare i campi della configurazione e usare le singole API; non esistono variabili globali.

Esempio di chiamate per la futura verifica in MATLAB Online (non eseguito in questa fase):

```matlab
u = [4; 4];                       % [omega_R; omega_L], rad/s
q = [0; 0; 0];                    % [x; y; theta], m, m, rad
twist = robot.model.wheelsToBody(u, params);
uRecovered = robot.model.bodyToWheels(twist, params);
dq = robot.model.derivative(q, u, params);
qNext = robot.model.integrateRK4(q, u, project.config.integration.dt, params);
qExact = robot.model.propagateConstant(q, u, project.config.integration.dt, params);
```

Stato e comandi sono vettori colonna di dimensione `3×1` e `2×1`. Le funzioni cinematiche accettano `double` e `single` reali finiti, non sparse, e restituiscono `double`. Il passo RK4 mantiene costanti le velocità delle ruote durante l'intervallo. Gli integratori restituiscono l'angolo in `[-pi,pi)`; la derivata angolare rimane una velocità e non viene normalizzata. Il modello non applica saturazioni implicite o collision checking; tali verifiche sono esplicite nel pianificatore.

Per avviare la pipeline completa **durante la futura prova in MATLAB Online**:

```matlab
project = main('run');
```

Questa è una procedura predisposta, non già eseguita. L'azione esplicita `run` richiama scenario S2, RRT, simulatore, riepilogo, grafici ed esportazione PNG, poi l'animazione. Nessun test è avviato automaticamente. Se la pianificazione fallisce, restituisce il piano con il suo stato e un avviso, senza simulare o produrre grafici vuoti. Lo scenario S2 contiene il rettangolo `10×8 m`, due cerchi, un poligono rettangolare e le pose assegnate da `makeScenario`.

Su una simulazione completata o interrotta, `project.log` conserva i dati, `project.summary` il riepilogo, `project.figures` le tre figure e `project.animationAxes` l'animazione. Anche un prefisso interrotto viene analizzato, mantenendo visibile il motivo d'arresto e senza attribuirgli un arrivo riuscito.

Durante `run`, dopo simulazione e riepilogo, `results/risultati.mat` salva la struttura `project` con configurazione, scena, piano, log e metriche **prima del rendering**, senza handle grafici. Questo conserva i dati numerici anche se la successiva esportazione o animazione genera un errore. Seguono i PNG `results/traiettorie.png`, `results/errore_tracking.png` e `results/velocita_ruote.png`, a 200 dpi. La cartella è derivata dalla posizione di `main.m`; file omonimi vengono sostituiti da un'esecuzione successiva. Il MAT non viene creato se la pianificazione fallisce prima della simulazione. Nessun risultato o cartella è stato generato durante questo audit.

Le API riutilizzabili sono `robot.analysis.computeMetrics(log)`, `robot.visualization.makePlots(log,outputFolder)` e `robot.visualization.animate(log,playbackSpeed)`. Omettendo `outputFolder`, `makePlots` non salva file. La velocità nominale di riproduzione è 10 volte il tempo simulato, con circa 20 aggiornamenti al secondo; il rendering può rallentarla. Chiudere la figura interrompe l'animazione, senza modificare i dati o il riepilogo già prodotti. Non è introdotto un esportatore video/GIF.

La vista XY sovrappone il piano RRT e i campioni RK4 effettivamente eseguiti. L'animazione ricostruisce le pose intermedie tramite il passo RK4 a durata parziale e aggiorna corpo, asse, ruota destra/sinistra e direzione. Non interpola linearmente gli angoli e non riesegue pianificazione o controllo. Le linee fra campioni nei grafici sono collegamenti visivi, non nuovi archi certificati.

Il tracking usa gli errori registrati rispetto alla primitiva attiva e mostra anche il limite superiore che include l'approssimazione geometrica. Le velocità ruota sono rappresentate a gradino su intervalli costanti: N istanti corrispondono a N-1 comandi. Per un log con una sola posa sono mostrati punti di tracking e un messaggio di assenza di comandi, senza inventare una curva delle velocità.

Il riepilogo comprende errori finali, tracking medio/RMS pesato nel tempo e massimo, durata, esito e motivo d'arresto. Distingue collisioni fisiche rilevate sulle pose registrate, violazioni del margine e certificati degli intervalli presenti nel log del simulatore. Un comando respinto non è una collisione avvenuta. La certificazione riportata non è un nuovo audit continuo indipendente né una garanzia fisica.

Il RRT cresce in SE(2) con comandi ruota costanti, ricavati e propagati mediante il modello cinematico. I limiti iniziali sono 10 rad/s per ruota, 0,30 m/s e 1,50 rad/s per il corpo, applicati tramite riduzione comune dei comandi. Il corpo è contenuto per ipotesi in un disco di raggio 0,22 m, con margine 0,03 m applicato una sola volta. La collisione viene verificata lungo l'intero arco, non solo ai nodi; intervalli non certificabili sono rifiutati.

`plan.status` distingue `success`, `invalid_input` e `budget_exhausted`. Il budget esaurito non dimostra l'assenza di un percorso. Un successo richiede una connessione differenziale al goal sia in posizione sia in orientamento, entro 1e-8 m e 1e-8 rad. Errori numerici inattesi sollevano un'eccezione, senza restituire un falso successo.

Su successo, `plan.path.states` contiene le pose di giunzione (`M+1` righe), `controls` i comandi destra/sinistra (`M×2`), `durations` i tempi (`M×1`) e `clearances` i limiti inferiori certificati. `samples`, `time` e `segment` rappresentano gli archi campionati per il grafico, comprese le rotazioni sul posto. Il tempo è nominale di pianificazione, non il tempo di un moto controllato. `plan.tree` conserva nodi, genitori, comandi, durate, clearance e costo temporale cumulativo. Il seed iniziale è 17, il budget 10000 tentativi; lo stato RNG del chiamante viene ripristinato.

Il simulatore preserva e verifica i campioni del piano, segue i waypoint in ordine e conserva le rotazioni sul posto. In avanzamento usa `v=kRho*rho`, `omega=kAlpha*alpha`, con distanza dal waypoint e angolo di direzione normalizzato. Un errore angolare grande provoca allineamento senza traslazione; rotazioni del riferimento e orientamento finale hanno una modalità dedicata. La conversione inversa e i limiti accoppiati riutilizzano le funzioni esistenti.

I parametri iniziali sono `kRho=0.8`, `kAlpha=2`, `kTheta=2`, tolleranza waypoint `0.01 m` e rotazione `0.01 rad`, tolleranze finali `0.03 m` e `0.035 rad`, passo `0.02 s`, massimo `900 s` simulati e `45000` passi. Il budget considera il rallentamento proporzionale a ogni waypoint fitto; non è il tempo reale richiesto al computer né una garanzia di completamento. Sono scelte da validare, non una taratura già verificata. I campi sono in `cfg.control` e `cfg.simulation`.

`log.status` distingue `goal_reached`, `tracking_blocked`, `timeout`, `step_limit` e `numerical_failure`. Ogni passo viene verificato prima di accettarlo, includendo un margine per l'approssimazione RK4; la deviazione dal tratto attivo non deve superare `0.05 m`. Un ostacolo o un intervallo incerto produce arresto, senza replanning automatico. Un piano fattibile non garantisce che questo controllore riesca a completarlo.

`log.reference` contiene il piano verificato; `log.q` e `log.time` la traiettoria numerica eseguita. `rho`, `alpha`, `orientationError`, `trackingError`, `trackingApproximationBound`, `referenceIndex` e `mode` hanno un valore per stato. `u`, `twist`, `clearanceBound` e `integrationErrorBound` hanno un valore per intervallo, quindi una riga in meno. Le velocità delle ruote sono quelle effettive dopo i limiti. `finalState`, `finalPositionError`, `finalAngleError` e `goalReached` descrivono l'esito finale. Un log interrotto rappresenta soltanto il prefisso accettato, non un arrivo riuscito.

`log.configuration` e `log.scene` conservano parametri e ambiente usati. I dettagli matematici di log e ricostruzione sono nei §4.8–4.9 della documentazione progettuale. I moduli di simulazione e analisi non scrivono file; solo la richiesta esplicita di esportazione PNG produce i tre grafici indicati.

Per avviare la suite **nella fase di verifica esecutiva**:

```matlab
results = run_tests();             % tutte le suite disponibili
results = run_tests('unit');       % include controllo e simulazioni numeriche di verifica
results = run_tests('smoke');      % struttura
```

`run_tests` risolve le cartelle dalla propria posizione e ripristina il path alla fine, anche in caso di errore. Mostra i risultati effettivi; se la suite è vuota, fallisce o contiene test incompleti, solleva un errore. In caso di errore l'assegnazione a `results` non viene completata: consultare l'output della suite. Non è richiesto alcun cambio di cartella interno permanente. Dopo aver reso il punto di ingresso raggiungibile nel path, si può invocarlo anche da un'altra cartella.

La suite unit include anche quattro verifiche numeriche dell'analisi che usano output del pianificatore e del simulatore, senza dati sostitutivi: caso fermo, tracking rettilineo, comando bloccato e rifiuto di log incoerenti. Il validatore controlla anche il moto tra pose consecutive rispetto a comandi, tempi e bound RK4 registrati. L'audit aggiunge una regressione della connessione al goal con tolleranza di posizione stretta e rafforza i controlli sugli array vuoti. Sono presenti **52 funzioni di test** contate staticamente in cinque file, non un esito della discovery/esecuzione MATLAB. Nessun test è stato avviato; l'aspetto grafico richiederà verifica visiva.

## Procedura prevista per la sessione MATLAB Online

Aprire in MATLAB Online la cartella completa in una posizione scrivibile di MATLAB Drive e impostarla come Current Folder. Usare un'unica sessione per Code Analyzer, test e prova completa, senza eseguire questi comandi durante la preparazione statica:

```matlab
if ~isfolder('results')
    mkdir('results');
end
diary(fullfile('results', 'verifica_matlab.txt'));
disp(version);
files = [dir('*.m'); dir(fullfile('src', '**', '*.m')); ...
    dir(fullfile('tests', '**', '*.m'))];
for index = 1:numel(files)
    checkcode(fullfile(files(index).folder, files(index).name), '-id');
end
results = run_tests();
save(fullfile('results', 'test_results.mat'), 'results');
project = main('run');
diary off
```

Esaminare i messaggi del Code Analyzer: la revisione manuale dell'audit non equivale al parser MATLAB. Un test fallito o incompleto interrompe il lanciatore; in tal caso non avviare la prova completa, conservare il diario e chiuderlo con `diary off`. Le segnalazioni del Code Analyzer vanno valutate distinguendo errori di sintassi da suggerimenti e soppressioni intenzionali, come quelle dei guardiani `onCleanup`.

Nella prova completa verificare `project.plan.status`, poi `project.log.status`, `goalReached`, errori finali, limiti ruota e informazioni sulle collisioni. Un piano riuscito non equivale all'arrivo controllato. Controllare visivamente le tre figure, i PNG e l'animazione, compresi orientamento e ruote. Conservare versione dei sorgenti, diario, risultati dei test, MAT numerico e PNG realmente prodotti. Se si verifica un errore grafico, il MAT numerico già salvato consente di recuperare i dati senza ripetere la simulazione. Non considerare test eseguiti una seconda volta solo perché si rigenerano i grafici.

**Esito statico:** il progetto è pronto per essere provato in MATLAB Online; rimangono da verificare parser, risultati numerici, completamento dello scenario S2, tempi di calcolo e comportamento grafico. Non è garantito il funzionamento alla prima esecuzione e non è dichiarato superato alcun test.

## Documentazione e prossima fase

- [Analisi e progettazione](docs/01_Analisi_e_Progettazione.md): requisiti, pianificazione (§4.7), controllo/log (§4.8), analisi e visualizzazione (§4.9).
- [Modello cinematico](docs/02_Modello_Cinematico.md): formule, ipotesi, contratti, casi limite e test.
- [Checklist corrente](docs/Checklist_Requisiti.md): evidenze statiche e verifiche ancora aperte.

La prova e la validazione dell'intera pipeline saranno effettuate successivamente in MATLAB Online. Non si avvia autonomamente la fase successiva e non viene preparato ora il PDF definitivo.

## Consegna universitaria e repository

**La consegna consiste in un unico PDF autonomamente comprensibile**, con frontespizio, collegamento al repository GitHub, istruzioni di esecuzione MATLAB e documentazione tecnica completa: modelli matematici, algoritmi, implementazione, simulazioni, grafici, risultati reali, analisi critica, validazione e conclusioni. Il PDF definitivo sarà preparato soltanto dopo esecuzione e validazione.

Tutto il codice MATLAB e tutti i test saranno nel repository. Non è più necessario riportare integralmente i sorgenti `.m` nel PDF, salvo successive indicazioni del docente. Il testo spiegherà il sistema senza richiedere la lettura dei sorgenti; potrà includere pseudocodice e frammenti significativi. L'animazione sarà nel repository e illustrata nel PDF mediante fotogrammi e commento.

**Repository GitHub: non ancora pubblicato; URL non disponibile.** La pubblicazione sarà gestita separatamente. Questa fase non crea repository remoti e non effettua push. Alla pubblicazione si dovrà indicare nel PDF la revisione/tag che corrisponde a risultati e sorgenti.

## Limiti attuali

Il modello è cinematico ideale: ruote uguali, rotolamento senza slittamento, piano orizzontale e stato noto. Non modella accelerazioni, coppie, attrito o sensori. Le condizioni geometriche e dinamiche di un robot reale richiederebbero estensioni.

RK4 non ha un limite di errore automaticamente garantito per qualsiasi passo. Overflow non finiti sono segnalati; underflow e perdita di precisione su scale estreme non sono eliminati dalla validazione. Le verifiche in MATLAB Online, incluse sintassi effettiva, tolleranze e convergenza numerica, sono ancora da effettuare.

Il RRT non garantisce ottimalità o successo entro il budget. La certificazione delle collisioni è conservativa nel modello ideale del disco e può rifiutare passaggi stretti o archi insufficientemente risolti; non costituisce una prova formale in aritmetica esatta né una garanzia per slittamento o localizzazione incerta. Tempi, qualità dei percorsi e rappresentazione grafica non sono ancora verificati mediante esecuzione.

Il controllore non ha una garanzia globale di completamento per ogni piano RRT: tolleranze, cambi waypoint e guardie possono causare arresti o timeout. Il bound locale RK4 a comandi costanti è analitico; il margine aggiuntivo di roundoff è progettuale, non certificato con aritmetica a intervalli. La collision safety riguarda il moto numerico accettato nel modello ideale, non una frenata o un'esecuzione fisica reale. Il controllore e tutti i nuovi test restano da eseguire in MATLAB Online.
