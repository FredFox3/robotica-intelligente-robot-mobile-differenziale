# Pianificazione e controllo di un robot mobile differenziale

Progetto MATLAB per pianificare e seguire un percorso in un ambiente
bidimensionale rettangolare con ostacoli circolari e poligonali.
Il pianificatore RRT genera primitive compatibili con la cinematica
differenziale e verifica le collisioni considerando l'ingombro del robot.
Un controllore cinematico proporzionale segue il percorso; il moto viene
integrato con RK4 e rappresentato mediante grafici e animazione.

## Requisiti

- MATLAB desktop oppure MATLAB Online; verifica effettuata su MATLAB Online.
- Solo funzionalità MATLAB standard, senza toolbox aggiuntivi.
- I test utilizzano il framework integrato `matlab.unittest`.
- Nessuna installazione o configurazione di dipendenze richiesta.

## Esecuzione

Aprire la cartella principale del repository come Current Folder in MATLAB.
Per eseguire tutti i test:

```matlab
results = run_tests();
```

Per avviare pianificazione, simulazione, riepilogo, grafici e animazione:

```matlab
project = main('run');
```

I test sono avviati separatamente. `main()` prepara soltanto la configurazione.
I percorsi sono risolti dalla posizione dei sorgenti.
Una nuova simulazione sostituisce i file omonimi presenti in `results/`.

## Struttura

- `main.m`: ingresso per configurazione e simulazione completa.
- `run_tests.m`: esecuzione della suite di test.
- `src/+robot/`: cinematica, ambiente, RRT, controllo, simulazione e grafica.
- `tests/`: test unitari numerici e verifiche dell'organizzazione del progetto.
- `results/`: dati e grafici reali della simulazione verificata.

## Risultati della verifica su MATLAB Online

- **52 test superati su 52**, senza fallimenti.
- **Goal raggiunto** dalla simulazione `main('run')`.
- **Nessuna collisione rilevata** lungo la traiettoria eseguita.

I risultati salvati sono `risultati.mat` (configurazione, piano, log e metriche),
`traiettorie.png`, `errore_tracking.png` e `velocita_ruote.png` in `results/`.
