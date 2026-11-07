# Checklist dei requisiti — stato FASE 6

**Ambito:** analisi, visualizzazione, animazione e PNG della Fase 6. Questa checklist integra la [matrice completa e i contratti realizzati (§4.7–4.9)](01_Analisi_e_Progettazione.md). Fonte: [traccia ufficiale](../TRACCIA_Robot_Mobile_Differenziale.md). Modello, RRT, controllore e simulatore restano invariati; nessun audit generale, esecuzione o risultato generato.

Le colonne sono indipendenti: **implementato** significa sorgenti predisposti, **statico completato** significa controllo manuale di formule, flusso e contratti, **esecutivo da effettuare** significa nessuna prova ancora svolta in MATLAB Online. Nessuna riga di implementazione o revisione è una dichiarazione di test superato.

## 1. Funzionalità cinematiche della Fase 3

| ID fase | Collegamento matrice | Funzionalità | Implementazione | Revisione statica | Verifica esecutiva |
| --- | --- | --- | --- | --- | --- |
| K01 | T01–T02, D01 | Geometria configurabile `r,d` | `defaultConfig`, `validateParameters` | Completata: positività, unità, distanza completa | Da effettuare |
| K02 | T08–T10 | Conversione ruote -> corpo | `wheelsToBody` | Completata: segni, ordine ruote, dimensione `2×1` | Da effettuare |
| K03 | T19 | Conversione corpo -> ruote | `bodyToWheels` | Completata: inversione e fattore `d/2` | Da effettuare |
| K04 | T07, T11 | Campo cinematico e vincolo laterale | `derivative` | Completata: stato `3×1`, formule e identità non olonoma | Da effettuare |
| K05 | T20 | Integrazione numerica | RK4 e propagatore analitico riusati nel simulatore della Fase 5 | Completata per la nuova integrazione dei moduli | Da effettuare |
| K06 | D03, UT-C04 parziale | Normalizzazione angolare | `normalizeAngle`; angoli finali integratori | Completata: intervallo, continuità degli stadi e wrapping | Da effettuare |
| K07 | O04, UT-D01 | Validazione e casi limite | Geometria, vettori, durata, angoli, finitezza risultati | Completata: input non validi, short-circuit, overflow | Da effettuare |
| K08 | O05 | Test MATLAB richiesti | 23 funzioni di test unitari e 3 di struttura predisposte | Completata: oracoli, dimensioni, tolleranze | **NON ESEGUITI** |
| K09 | O06–O07 | Ingressi e organizzazione | `main`, `run_tests`, `src/`, suite unit/smoke | Completata: risoluzione e ripristino path | Da effettuare in MATLAB Online |

## 2. Copertura della traccia complessiva

| Requisiti | Stato implementazione corrente | Stato statico | Verifica esecutiva / lavoro restante |
| --- | --- | --- | --- |
| T01–T02, T07–T11 | Nucleo del modello disponibile | Completata nel perimetro cinematico | Tutti i test da eseguire |
| T03, T16, A-R01–A-R05 | RRT differenziale, connettore e ricostruzione disponibili | Completata a lettura per la Fase 4 | Test numerici predisposti; NON ESEGUITI |
| T04, T17–T18 | Riferimento ordinato e controllore cinematico disponibili | Legge, isteresi, rotazioni e goal completo revisionati | Test NON ESEGUITI |
| T05 | Pipeline completa con grafici e animazione su `main('run')` | Chiamate, dati e diramazione di fallimento revisionati | Programma e grafici NON ESEGUITI |
| T06 | Modello differenziale riutilizzato anche nel controllo | Conversioni, campo, RK4 e confronto analitico revisionati | Coerenza numerica da verificare in MATLAB Online |
| T12–T15 | Ambiente, ostacoli, disco e pose disponibili | Formule e validazione revisionate | Test geometrici e di input non validi NON ESEGUITI |
| T19 | Inversa e limiti comuni riutilizzati dal controllore | Comandi nominali/effettivi e ordine ruote revisionati | Test NON ESEGUITI |
| T20 | Ciclo del simulatore RK4 disponibile | Passo finale, budget e log revisionati | Convergenza e pipeline da verificare |
| T21 | Dati e grafico della traiettoria pianificata disponibili su chiamata | Campionamento degli archi revisionato | Nessun percorso o figura realmente generato |
| T22–T24 | Grafico comparativo, tracking e velocità R/L con esportazione PNG disponibili | Dimensioni, unità e allineamento N/N-1 revisionati | Nessun dato, grafico o PNG generato |
| T25 | Animazione implementata con corpo, asse, ruote, orientamento e traccia progressiva | Geometria e ricostruzione temporale RK4 revisionate | Nessuna riproduzione o verifica visiva effettuata |
| A-CD01–A-CD05, A-F01–A-F05 | Alternative non selezionate | Confronto teorico disponibile | Non richieste nell'implementazione principale |

## 3. Organizzazione e consegna definitive

- [x] README aggiornato per l'uso corrente; istruzioni operative esistenti subordinate alle richieste più recenti dell'utente.
- [x] Codice sorgente, test e documentazione separati; nessuna copia del vecchio test di struttura.
- [x] `.gitignore` predisposto senza escludere indiscriminatamente risultati finali o PDF.
- [x] Modello privo di dipendenze da toolbox aggiuntivi, shell o percorsi personali.
- [x] Documentazione del modello predisposta con ipotesi, formule, unità e contratti.
- [x] Regola precedente dei sorgenti `.m` integrali nel PDF sostituita nei documenti attivi.
- [x] Analisi e visualizzazione integrate senza modificare modello, RRT, controllore o simulatore.
- [x] Nessuna installazione, esecuzione di test/simulazioni o pubblicazione GitHub effettuata durante questa fase.
- [ ] Parser e Code Analyzer in MATLAB Online sui nuovi sorgenti.
- [ ] Test smoke e unit eseguiti in MATLAB Online, con output e release registrati.
- [ ] Conferma sperimentale delle tolleranze e della convergenza numerica.
- [ ] Moduli successivi implementati e verificati.
- [ ] Risultati reali, grafici e animazione prodotti e associati alla versione dei sorgenti.
- [ ] Repository GitHub dedicato pubblicato separatamente; URL e revisione ancora non disponibili.
- [ ] Unico PDF autonomo completato con matematica, metodologie, implementazione, risultati, analisi critica, validazione, conclusioni e link reale al repository.

I sorgenti integrali saranno disponibili nel repository; la loro riproduzione nel PDF non è richiesta, salvo successive indicazioni del docente. I risultati non ancora ottenuti non devono essere sostituiti con esempi presentati come esiti reali.

## 4. Evidenze statiche e questioni aperte

**Controlli completati a lettura:** corrispondenza nomi file/funzioni, firme e chiamate; dimensioni e unità SI; inversa delle conversioni; segni di rotazione; identità del vincolo laterale; stadi RK4; limite della funzione sinc non normalizzata; gestione di zero, angoli al taglio e valori non finiti; coerenza fra codice, test e documentazione; percorsi e organizzazione. I controlli sui testi non costituiscono parsing MATLAB o esecuzione dei test.

Gli inventari e i controlli delle fasi precedenti restano storici. La Fase 6 aggiunge quattro funzioni di analisi/visualizzazione e un file con quattro test numerici; riusa `plotPlan` e aggiorna l'ingresso principale. Nessun nuovo Markdown o file di risultati. I controlli testuali sui soli moduli coinvolti non sostituiscono parser, esecuzione dei test o verifica visiva.

**Criticità residue:** parser, rendering, gestione finestre, salvataggio PNG e prestazioni dell'animazione da verificare in MATLAB Online. Le proprietà numeriche delle fasi precedenti rimangono non verificate mediante esecuzione. Il riepilogo distingue collisioni ai campioni dai certificati registrati; non prova autonomamente la sicurezza fisica o continua. Nessun risultato osservato prima della prova reale.

## 5. Funzionalità di pianificazione della Fase 4

| ID | Funzionalità | Implementazione | Revisione statica | Verifica esecutiva |
| --- | --- | --- | --- | --- |
| P01 | Ambiente e configurazioni | `makeScenario`, `validatePlanningInputs` | Completata: dimensioni, unità e geometrie | NON ESEGUITA |
| P02 | Collisioni del disco lungo il moto | `clearance`, `checkPrimitive` | Completata: distanze firmate, bound 1-Lipschitz e rifiuto degli intervalli incerti | NON ESEGUITA |
| P03 | RRT selezionato | `planRRT`, `boundedCommand` | Completata: candidati, limiti, duplicati, seed e budget | NON ESEGUITA |
| P04 | Goal completo e ricostruzione | `connectGoal`, `reconstructPath` | Completata: primitive, genitori, continuità e rotazioni | NON ESEGUITA |
| P05 | Grafico del piano | `plotPlan` | Chiamate revisionate, nessuna figura prodotta | Da verificare in MATLAB Online |
| P06 | Test numerici essenziali | `tests/unit/test_planning.m` | Nove funzioni con oracoli geometrici e replay | **NON ESEGUITI** |

## 6. Controllo cinematico e simulazione della Fase 5

| ID | Funzionalità | Implementazione | Revisione statica | Verifica esecutiva |
| --- | --- | --- | --- | --- |
| C01 | Riferimento coerente col RRT | `buildReference` e replay delle primitive/campioni | Completata: ordine, durate, pose e rotazioni | NON ESEGUITA |
| C02 | Legge cinematica | `control.step`, errori rho/alpha e modalità | Completata: formule, singolarità, isteresi e orientamento finale | NON ESEGUITA |
| C03 | Comandi ruota e limiti | `boundedCommand` riusato senza duplicazione | Completata: ordine R/L, conversioni e riduzione comune | NON ESEGUITA |
| C04 | Integrazione controllata | `simulation.simulate` usa RK4 esistente | Completata: comando costante, tempi, passi e arresti | NON ESEGUITA |
| C05 | Sicurezza del moto eseguito | Verifica di ogni arco con margine RK4 e controllo del tratto attivo | Completata: bound locale, endpoint e rifiuto prima dell'applicazione | NON ESEGUITA |
| C06 | Dati per analisi e animazione | Log con N pose e N-1 intervalli | Completata: unità, dimensioni, stato finale e caso senza moto | Nessun dato prodotto |
| C07 | Verifiche numeriche essenziali | `test_control_simulation.m`, dodici funzioni | Oracoli e condizioni di accettazione predisposti | **NON ESEGUITI** |

## 7. Visualizzazione e analisi della Fase 6

| ID | Funzionalità | Implementazione | Revisione statica | Verifica esecutiva |
| --- | --- | --- | --- | --- |
| V01 | Validazione dei dati | `analysis.validateLog` | Formati, tempi, riferimento, twist e metadati revisionati | NON ESEGUITA |
| V02 | Traiettorie e ambiente | `makePlots` riusa `plotPlan` | Assi, ostacoli, start/goal e posa finale eseguita revisionati | Nessuna figura generata |
| V03 | Tracking e ruote | Grafico temporale e comandi a gradino R/L | Allineamento N/N-1 e caso fermo revisionati | NON ESEGUITA |
| V04 | Animazione | `visualization.animate` | Fotogrammi RK4, geometria ruote e stato finale revisionati | NON ESEGUITA |
| V05 | PNG e ingresso | `main('run')`, `print` e cartella relativa al progetto | Flusso esplicito, nomi e assenza di scritture in setup revisionati | Nessun PNG o cartella risultati creato |
| V06 | Riepilogo numerico | `computeMetrics` | Errori, pesatura temporale, durata, goal e collisioni/certificati distinti | Test NON ESEGUITI |

Quattro test numerici predisposti in `test_analysis.m` usano output del pianificatore e del simulatore, non dati di risultati inventati. Il caso a durata nulla ha un solo stato, zero intervalli e statistiche definite. Nessuna esecuzione anticipata, nessun PDF definitivo e nessun avvio autonomo della fase successiva.
