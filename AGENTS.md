# Istruzioni operative del progetto

Progetto: **PIANIFICAZIONE E CONTROLLO DI UN ROBOT MOBILE DIFFERENZIALE**.
La fonte didattica è `TRACCIA_Robot_Mobile_Differenziale.md`. Le decisioni dell'utente più recenti prevalgono sulle precedenti versioni della documentazione.

## Principio guida per tutte le prossime fasi

Privilegiare **semplicità, correttezza, completezza rispetto alla traccia ufficiale e riproducibilità**. Mantenere il repository essenziale e ordinato.

- Creare soltanto file e cartelle realmente necessari all'implementazione, all'esecuzione, alla verifica e alla documentazione del progetto.
- Evitare framework, infrastrutture, configurazioni, dipendenze e astrazioni non richieste o prive di una giustificazione concreta nel progetto.
- Non produrre file di esempio, script alternativi o risultati simulati non necessari. Non presentare risultati sintetici come esiti reali.
- Non cancellare automaticamente file esistenti. Prima della pubblicazione GitHub effettuare con l'utente una revisione degli elementi effettivamente superflui. Le nuove regole non autorizzano una pulizia retroattiva.
- Questo aggiornamento riguarda le istruzioni per il lavoro futuro: non modificare ora l'implementazione già completata.

## Organizzazione e consegna definitive

- Il codice completo sarà ospitato in un repository GitHub dedicato. **Non creare o pubblicare il repository e non effettuare push in questa fase**: la pubblicazione sarà gestita separatamente.
- La consegna universitaria è **un unico PDF separato e autonomamente comprensibile**, con documentazione tecnica esaustiva, formulazione matematica, metodologie, descrizione dell'implementazione, istruzioni di esecuzione, grafici e risultati reali, analisi critica, validazione, conclusioni e link al repository GitHub.
- Non è più richiesto includere integralmente i sorgenti `.m` nel PDF, salvo successive indicazioni del docente. Il repository conterrà tutti i sorgenti e test. Nel PDF inserire algoritmi e frammenti selezionati quando utili alla comprensione.
- Non inventare l'URL del repository; indicarlo come non ancora disponibile fino alla pubblicazione. Quando pubblicato, documentare anche la revisione del codice associata ai risultati e al PDF.
- L'animazione richiesta dalla traccia sarà disponibile nel repository; il PDF includerà fotogrammi, spiegazione e riferimento al file senza richiedere la sua apertura per capire i risultati.
- Mantenere codice modulare e commentato in `src/`, test in `tests/`, documentazione in `docs/`, risultati in `results/` e materiale della relazione in `report/` quando presente. Non creare moduli o cartelle vuote senza necessità.
- Mantenere `README.md` come riferimento principale per descrizione, requisiti e istruzioni di esecuzione; conservare un `.gitignore` appropriato.
- Riservare `AGENTS.md` alle istruzioni operative di Codex, senza duplicarle negli altri documenti.
- Consolidare la documentazione tecnica esistente e mantenerla senza duplicazioni; non creare un documento Markdown separato per ogni piccola funzionalità. Aggiornare i requisiti nei riferimenti esistenti senza introdurre ulteriori checklist o infrastrutture documentali non necessarie.

## Ambiente di sviluppo e verifica

- Linguaggio target MATLAB; **MATLAB Online è l'unico ambiente di verifica esecutiva**. Non usare GNU Octave né MATLAB locale.
- Non installare software, pacchetti o ambienti di esecuzione.
- Usare MATLAB standard; evitare Robotics System Toolbox e altri toolbox non indispensabili. Il modello cinematico non richiede alcun toolbox aggiuntivo. Non aggiungere dipendenze senza necessità dimostrata e documentata.
- Usare `fullfile` e percorsi relativi o derivati dalla posizione dei sorgenti. Nessun percorso personale, comando di shell o dipendenza dal sistema operativo nel codice MATLAB.
- Durante lo sviluppo effettuare revisioni statiche, matematiche e logiche approfondite. Non eseguire ora test o simulazioni; predisporli per la verifica successiva in MATLAB Online.
- Non dichiarare superati test non eseguiti, né prestazioni, stabilità numerica o assenza di collisioni non osservate. Distinguere sempre **implementato**, **revisionato staticamente** e **verificato mediante esecuzione**.

## Fase corrente: FASE 4 — Pianificazione del moto

Riutilizzare la cinematica della Fase 3 senza una revisione generale. Implementare soltanto ambiente rettangolare, ostacoli, ingombro, collisioni, RRT differenziale già selezionato, connessione al goal, ricostruzione e grafico del piano. Predisporre solo test numerici pertinenti; non eseguire codice MATLAB, test o simulazioni ora. L'unica verifica esecutiva sarà quella finale in MATLAB Online.

- Convenzioni: `q=[x;y;theta]`, `u=[omega_R;omega_L]`, `r` raggio comune delle ruote, `d` distanza completa tra le ruote. Unità SI, angoli in radianti, rotazione antioraria positiva.
- Usare il modello differenziale della traccia, anche per i futuri archi di pianificazione e per il controllo. Un percorso geometrico non ne dimostra da solo la compatibilità cinematica.
- Predisporre test MATLAB per rettilineo, rotazione sul posto, curve, conversioni, parametri non validi e condizioni limite.
- Consolidare algoritmo e contratti della pianificazione nel documento esistente `docs/01_Analisi_e_Progettazione.md`; aggiornare README e checklist soltanto per il perimetro corrente. Non creare ulteriori Markdown per questa fase.
- **Non implementare il controllore completo e non procedere autonomamente alla Fase 5.** Nessuna esecuzione, pubblicazione GitHub o pulizia generale del progetto.

## Revisione statica obbligatoria

Per ogni modifica esaminare:

1. sintassi, struttura delle funzioni e corrispondenza nomi file/funzioni;
2. firme, ordine argomenti, valori restituiti e chiamate;
3. dimensioni e orientamento di matrici/vettori, indicizzazione e operatori;
4. formule, segni, unità di misura, limiti matematici e integrazione;
5. compatibilità MATLAB e dipendenze;
6. input non finiti, parametri degeneri, divisioni per zero, overflow e altre condizioni limite;
7. coerenza tra moduli, test, documentazione e stato dei requisiti.

Il parser e il Code Analyzer di MATLAB restano controlli esecutivi da effettuare in MATLAB Online; una lettura statica manuale non equivale a tali controlli.

## Punti di ingresso e test

- Conservare `main.m` e `run_tests.m` alla radice. `main` deve dichiarare chiaramente quali funzionalità sono disponibili e non avviare implicitamente una simulazione.
- Usare le funzionalità di test standard MATLAB senza infrastrutture aggiuntive non necessarie. Per il lavoro futuro mantenere esclusivamente test numerici utili alla correttezza del progetto, con oracoli e criteri significativi; non aggiungere test di mera struttura o verifiche che duplicano l'implementazione senza una validazione utile. I test già esistenti restano invariati fino alla revisione prima della pubblicazione.
- Documentare oracoli, tolleranze e risultati realmente osservati. Test falliti o incompleti non sono successi.
- Preparare il progetto per il caricamento in MATLAB Online senza installazioni. Non pubblicarlo su GitHub durante questo lavoro.
