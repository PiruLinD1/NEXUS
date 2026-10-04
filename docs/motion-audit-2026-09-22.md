# Revisione odometria e movimenti autonomi — 22 settembre 2026

La revisione comprende lettura dei sensori PROS, conversioni delle unità, fusione, odometria, recupero della posizione, pianificazione, NMPC, gestione dei comandi e calibrazione. Le correzioni mantengono l'interfaccia autonomous in mm/gradi/ms e il nucleo in metri/radianti/secondi.

## Esito delle verifiche

- Build host MSVC x64 Release completata: **12 suite su 12 superate**, comprese le nuove regressioni. Lo stimatore esegue 18.656 verifiche; la preparazione dei log comprende 11 test Python/CLI. [Registro completo](motion-audit-tests-2026-09-22.txt).
- Firmware ARM PROS ricompilato integralmente con `make -B quick -j4`: tutti i sorgenti compilati e pacchetti hot/cold collegati senza avvisi del compilatore.
- Archivio `bin/NEXUS.a` rigenerato con `make library IS_LIBRARY=1`. [Registro build ARM](motion-audit-arm-2026-09-22.txt).

I percorsi software verificati superano i test. L'idoneità sul robot richiede ancora la misura dei parametri fisici e le prove sul Brain descritte sotto.

## Problemi individuati e corretti

| Area | Problema | Correzione e verifica |
| --- | --- | --- |
| Comandi e modalità gara | `Sequence` acquisiva la modalità al primo passo: un oggetto creato nella modalità precedente poteva iniziare un'azione nella nuova. Una sequenza sospesa poteva inoltre sovrascrivere o fermare un movimento più recente. | Modalità, token e generazione vengono acquisiti alla costruzione. La sequenza rivendica soltanto la propria generazione con un confronto atomico e conserva il contesto originale nel payload; il worker controlla il token prima di modificare posa o comandi. Regressioni con task concorrenti, cambio modalità e sostituzione del comando nella stessa modalità. |
| Reset dell'odometria | Un `setPose` accodato prima del cambio modalità poteva essere applicato dopo la transizione. | Il worker verifica la modalità anche per il reset della posa. Test con coda sospesa e cambio modalità prima dell'esecuzione. |
| Riconoscimento del robot fermo | Due pod invariati non bastano a distinguere l'immobilità da una rotazione attorno all'intersezione dei loro assi. In assenza degli encoder drive, una rotazione lenta poteva essere congelata e appresa come bias gyro. | Il riconoscimento della quiete richiede anche osservabilità indipendente della rotazione dalle ruote disponibili. Test con rotazione lenta, pod invariati e drivetrain assente. |
| Recupero da distanze | Dopo un incremento di recupero limitato, un frame senza distanze nuove poteva ripristinare la salute prima della convergenza. | Lo stato di recupero incompleto persiste tra i frame, mantenendo `lost` fino alla convergenza o a un reset esplicito della posa. |
| Temporizzazione delle curve | I passaggi sulla curvatura non garantivano il limite di accelerazione del riferimento delle singole ruote. Un caso laterale produceva circa 6,7 m/s² con limite 3,5 m/s². | Verifica delle accelerazioni dei segmenti interpolati e dilatazione temporale conservativa quando necessaria. Test di traiettoria e simulazioni del controller. |
| Correzioni terminali piccole | Una tangente minima fissa introduceva anse sui percorsi millimetrici; un limite minimo artificiale di velocità nella formula del tempo rendeva incoerenti heading e velocità angolare. | Tangenti proporzionali al percorso e protezioni soltanto numeriche nel calcolo del tempo. Test di integrazione della velocità angolare e di arrivo con tolleranza stretta. |
| Dati e reset del controller | Opzioni non finite potevano trasformarsi in limiti di velocità validi; alcuni parametri infiniti non venivano normalizzati e il reset conservava la durata precedente. | Rifiuto di opzioni NaN/∞, valori di configurazione/budget finiti e azzeramento della traiettoria al reset. Regressioni dedicate. |
| Fit della calibrazione | I dati scartati come outlier potevano essere l'unica evidenza di un verso o dell'accelerazione necessaria per identificare il modello. | L'eccitazione minima viene ricontrollata sui soli campioni accettati, sia per i fit scalari sia per la dinamica delle ruote. |
| Dipendenze del profilo | Cambiare la scala gyro poteva lasciare marcati come calibrati offset e carreggiata ottenuti rispetto alla scala precedente. | `fit-rotation` invalida i parametri geometrici precedenti non rimisurati e accettati nella stessa prova. Regressione sul formato binario e sulla maschera del profilo. |
| Preparazione dei log | Timestamp stimatore non finiti, stati di salute ignoti e campioni non sincronizzati ai bordi della finestra potevano entrare nella derivata della velocità. | Validazione di timestamp, salute e slip su tutta la finestra della regressione, con test Python e del programma di calibrazione. |

I test esistenti inizialmente superavano tutte le dodici suite: le nuove regressioni coprono condizioni aggiuntive che la suite precedente non verificava.

## Struttura verificata

- **Sensori:** letture finite e codici di errore PROS, ripristino di verso/unità alla riconnessione, aggregazione degli encoder per lato, IMU continua e sensori opzionali. L'adattatore acquisisce a frequenza nominale 100 Hz e interroga le distanze a 20 Hz.
- **Unità e segni:** conversioni mm/SI, diametri in pollici, rapporto ruota/motore, heading orario da +Y e offset dei pod coerenti con l'integrazione SE(2). Le letture dei motori con porta negativa sono già invertite dal [driver PROS 4.2.1](https://github.com/purduesigbots/pros/blob/4.2.1/src/devices/vdml_motors.c); il verso dei pod è configurato mediante il [driver Rotation](https://github.com/purduesigbots/pros/blob/4.2.1/src/devices/vdml_rotation.c).
- **Stimatore:** fallback da pod/IMU agli encoder disponibili, baseline dopo dropout, bias, propagazione della covarianza, gating delle distanze, conferma temporale, storico con replay e recupero con geometria corroborata.
- **Controller:** modello dinamico distinto per lato, traiettorie avanti/indietro e rotazioni, limiti di tensione/batteria, budget cooperativo, arresto, arrivo e ripianificazione. Le simulazioni usano plant indipendenti dal rollout interno.
- **Integrazione:** task persistenti, richieste copiate, generazioni che invalidano comandi vecchi, watchdog da 100 ms, cambio modalità, annullamento, lease della calibrazione e sequenze che saltano i passi successivi dopo un fallimento.
- **Calibrazione:** conversioni delle misure esterne, identificabilità, fit robusti, applicazione parziale, CRC/fingerprint/backup e procedura guidata con consenso mantenuto.

## Cosa richiede ancora il robot

La verifica software non misura l'errore fisico né il tempo di calcolo sul Cortex-A9 del Brain. Non è stato caricato firmware o azionato il robot durante questa revisione.

Prima dell'uso competitivo rimangono necessarie le prove descritte in [validation.md](validation.md): segni e distanze misurati esternamente, rotazioni, ripetibilità di rette/curve/reverse, variazione di batteria e carico, tempi effettivi del solver, disconnessioni e cambi modalità reali. I coefficienti del template restano valori iniziali finché non sono sostituiti da un profilo misurato.

Il caso di recupero da spinta del solo controller impiega 5,90 s; quello accoppiato stimatore/controller con disturbi impiega 6,56 s. Questi test dei core consentono tempi superiori ai 5 s del timeout pubblico predefinito, applicato dal Chassis. Sul robot un movimento con quel timeout verrebbe arrestato allo scadere: scegliere un timeout coerente con il percorso e i disturbi accettabili, senza presumere che tutti i recuperi simulati avvengano entro il valore predefinito.

La configurazione corrente usa cartucce blu da 600 RPM e un rapporto che produce 360 RPM alla ruota. Se si cambia la cartuccia, va aggiornata anche la selezione del gearset in `robot.cpp`: modificare soltanto `motorRPM` cambia il rapporto numerico ma non la cartuccia dichiarata a PROS.

Le distanze sono disabilitate nella configurazione corrente (`port = 0`): senza riferimenti assoluti non è garantita la correzione di deriva o spinte non osservate. I limiti di accelerazione della traiettoria descrivono il riferimento; i vincoli di velocità e trazione del controller restano penalità, non garanzie fisiche. Non è presente una pianificazione degli ostacoli.
