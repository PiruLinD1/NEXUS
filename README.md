# NEXUS: localizzazione e movimento per VEX V5

NEXUS è la libreria di localizzazione, movimento e calibrazione del progetto, basata su PROS. L'API pubblica è [nexus/api.hpp](include/nexus/api.hpp), il namespace C++ è `nexus`, gli header sono in `include/nexus/` e i sorgenti in `src/nexus/`.

Per usare il template bastano **due file**:

| File | Cosa contiene |
| --- | --- |
| [include/robot-config.hpp](include/robot-config.hpp) | Porte, misure del robot, limiti e parametri fisici |
| [src/main.cpp](src/main.cpp) | Curve dei joystick, movimenti in `autonomous()` e tasti in `opcontrol()` |

NEXUS gestisce sensori, localizzazione, controllo e arresti. Semplificare queste dichiarazioni non cambia gli algoritmi, le frequenze di controllo o la calibrazione. Le lunghezze sono in **mm**, i diametri delle ruote in **pollici** e gli angoli autonomous in **gradi**.

## Assegnare i tasti

Nel ciclo di `opcontrol()` in `main.cpp` trovi queste righe. Cambia direttamente i nomi dei pulsanti:

```cpp
updateController();
const bool macroOwnsMechanisms = updateMacros(); // Gestisce anche X e la ripartenza dell'intake.
arcade(LEFT_Y, RIGHT_X, joystickCurves);
if (!macroOwnsMechanisms && (held(RIGHT) || held(Y))) manualLift(buttons(RIGHT, Y));
```

Nel programma B sale di livello e freccia giù scende di livello; Destra e Y, tenuti premuti, muovono il lift manualmente. L1 attiva/disattiva la pinza anche durante le macro senza fermare i movimenti. Un clic breve su X accende/spegne l'intake al rilascio, fuori dalle macro; tenendolo premuto per almeno **0,4 secondi**, l'intake gira al contrario finché X resta premuto e riparte subito in avanti al rilascio. L'intake riparte da solo quando la pinza raggiunge la posizione bassa completa, con braccio a zero e lift al fondo. R1 avvia la presa e R2 il rilascio con le posizioni prefissate del braccio. `updateController()` aggiorna i pulsanti una volta per ciclo; lascia questa riga prima dei comandi. X è gestito in `updateMacros()`.

Lascia anche il blocco `if (calibrationActive()) ... continue;` già presente dopo `updateController()`: permette al menu di calibrazione di usare gli stessi tasti senza azionare i meccanismi.

R1 avvia la presa e R2 il rilascio. La combinazione L1+R1 apre il menu di calibrazione nelle prove senza field controller.

Il livello extra del lift è **0,68 rotazioni sotto il fondo normale**. R2 mantiene il rilascio e il ritorno automatico, compresa la discesa del lift.

## Regolare le curve dei joystick

In cima a `main.cpp`, `joystickCurves` regola la potenza in funzione della posizione dei due stick:

```cpp
constexpr nexus::ArcadeCurves joystickCurves{
    .forward = 0.55, // Avanti/indietro.
    .turn = 0.55     // Sterzo.
};
```

Ogni valore va da **0.0** (risposta lineare) a **1.0** (curva cubica, più dolce vicino al centro); **0.55** mantiene la risposta precedente. I due valori sono indipendenti: aumentando `forward` rendi più graduale l'avanzamento ai piccoli spostamenti del joystick; aumentando `turn` rendi più graduale lo sterzo. Restano la zona morta sotto 4/127 e il limite dello sterzo al 75%. La curva segue immediatamente la posizione dello stick.

## Scrivere l'autonomous

In `autonomous()` in `main.cpp`, dopo `Auton auton;`, scrivi una riga per azione:

```cpp
using namespace robot;
Auton auton;

auton.setPose(0, 0, 0);
auton.intake(true);
auton.moveToPoint(0, 600);
auton.moveToPose(600, 900, 90, {.maxSpeed = 1000});
auton.moveThrough({{400, 1100}, {0, 1200}, {-300, 1200}}, -90, {.maxSpeed = 1200});
auton.pinza(true);
auton.wait(200);
auton.lift(127);
auton.braccio(-127);
auton.wait(300);
auton.lift(0);
auton.braccio(0);
auton.moveToPoint(300, 900, {.reverse = true, .maxSpeed = 750});
auton.turnToHeading(0);
auton.intake(false);
```

`Auton auton;` e `auton.setPose(0, 0, 0);` sono già presenti nel file: non duplicarli quando aggiungi i comandi. I movimenti di esempio nel progetto sono **commentati**, quindi all'inizio l'autonomous non muove il robot. Per la prima prova decommenta soltanto un movimento breve, adattato allo spazio disponibile.

Le coordinate sono posizioni assolute nella mappa; `setPose` dichiara la posa iniziale del robot. X positivo a destra, Y positivo in avanti, heading 0 lungo +Y e positivo in senso orario. `moveToPoint` lascia libero l'heading finale; `moveToPose` impone anche l'orientamento. `maxSpeed` è in **mm/s**, i timeout e `wait` in **ms**. Lift e braccio accettano potenze da `-127` a `127`; il comando resta attivo fino al successivo comando o all'arresto. `pinza(true)` chiude e `pinza(false)` apre; `intake(true)` avvia e `intake(false)` ferma.

I movimenti attendono l'arrivo oppure il proprio timeout prima della riga successiva. **Il timeout ferma soltanto quella mossa: le istruzioni successive dell'oggetto `auton` continuano**, comprese attese e meccanismi. `auton.result()` resta `timedOut` dopo la scadenza, fino all'esito della mossa successiva: non viene presentata come un arrivo riuscito. Perdita della localizzazione, guasto del solver, richiesta non valida o annullamento bloccano invece le istruzioni successive. Al termine della funzione l'oggetto arresta automaticamente tutti i motori e mostra `Auton: ...` sul display. Il cambio modalità gara annulla il movimento. Non sono pianificati ostacoli: scegli punti che lascino spazio al robot e alle curve.

`moveThrough({{x1, y1}, {x2, y2}, ...}, heading, opzioni)` percorre da 2 a 8 punti in una sola traiettoria: i primi guidano il passaggio senza fermate intermedie, mentre l'ultimo è la destinazione precisa con l'heading indicato. I punti guida non impongono un orientamento; la curva ricava una tangente continua dal percorso. Il timeout vale per l'intero percorso e `reverse` ne seleziona il verso. Usa `moveToPose` per una presa o un rilascio che richiede l'arresto, e raggruppa in `moveThrough` i punti che servono solo a indicare dove passare.

Il planner confronta diverse forme della curva usando tempo previsto, sterzo e lunghezza, entro i limiti di velocità e accelerazione configurati. Se il robot raggiunge la posa prima del riferimento e può frenare dentro la tolleranza, anticipa la frenata. Una volta fermo con margine dentro la tolleranza, mantiene quella posa durante l'attesa di assestamento; l'arrivo richiede comunque posizione, heading e velocità misurati entro i limiti.

`auton.liftToBottom()` attende la quota minima consentita dal prematch: fondo normale, oppure altre 0,68 rotazioni sotto il fondo con braccio allo stop 2. Per muovere il lift insieme al telaio, aggiungi `LiftMove::upStep` oppure `LiftMove::bottom` come quinto argomento di `auton.moveToPose(...)` o quarto argomento di `auton.moveThrough(...)`. La salita è di uno step configurato (1,38 rotazioni) dalla quota corrente. Il controllo del lift prosegue durante la guida e la riga successiva attende entrambi i movimenti. Un guasto del lift arresta anche la mossa del telaio e blocca i comandi successivi.

## Preparare un nuovo robot

1. In `robot-config.hpp`, verifica porte e segni dei motori, diametri delle ruote, `wheelRPM`, `trackWidth` e offset dei pod. I diametri sono in pollici, gli altri valori di lunghezza in mm.
2. All'avvio lascia il robot fermo durante la calibrazione nativa dell'IMU. Da ODOM 7 non viene aggiunta una seconda misura del bias; il rilevamento della quiete non modifica l'heading IMU.
3. Controlla i versi dei motori e dei sensori. Esegui la calibrazione guidata sotto per offset dei pod e carreggiata; verifica comunque diametri, rapporto di trasmissione e scala delle distanze con un riferimento esterno.
4. In `main.cpp`, imposta tasti e un primo movimento breve. Parti per esempio da `auton.moveToPoint(0, 300, {.maxSpeed = 250, .timeout = 5000, .positionTolerance = 20});` dopo `auton.setPose(0, 0, 0);`. Misura il risultato sul pavimento, ripeti e segui la [guida alla calibrazione](docs/calibration.md).

Per una nuova stagione duplica il progetto, aggiorna la configurazione e incrementa `robotRevision`. Porte e geometria contribuiscono all'identificatore hardware: un profilo incompatibile viene rifiutato. I valori iniziali non sono misure del tuo robot.

La mappa iniziale ha l'origine al centro del campo, con pareti a ±1828.8 mm. Se usi i sensori di distanza, `setPose` e i limiti del campo devono riferirsi alla stessa mappa. I sensori di distanza sono disattivati per default con porta `0`.

## Calibrazione guidata sul robot

Non serve modificare l'autonomous per calibrare:

1. Metti il robot sul pavimento con spazio libero per girare. Usa DRIVER abilitato, con field controller e competition switch scollegati.
2. Tieni **L1+R1 per un secondo**, poi rilasciali: si apre il menu e i normali comandi di guida e meccanismi vengono sospesi.
3. **Y** prepara la nuova prova degli offset con soli pod e IMU, utilizzabile anche con tutti gli encoder motore esclusi. Prima di partire, segna il centro geometrico della trazione e una croce sul pavimento sotto quel punto.
4. Tieni **R1** fino alla sosta a circa **+90°**, quindi rilascialo quando compare `FERMO`. Ricentra a mano lo stesso punto del telaio sulla croce mantenendo l'orientamento della sosta, senza sollevare il robot; premi **A**. Tieni R1 per la seconda sosta a circa **-90° rispetto alla partenza** e ripeti ricentraggio e A. Tieni R1 un'ultima volta per il ritorno vicino all'heading iniziale. [Guida passo passo e motivo delle due soste](docs/calibration.md#prova-offset-con-due-riferimenti-tasto-y).
5. Il Brain mostra **Avanti X**, **Laterale Y** e la differenza fra le due stime, in mm. **X** applica solo in RAM se le stime concordano; **B** esce e **A** torna al menu. Non serve una microSD. Rilasciare R1 durante una rotazione o premere B annulla la prova; non vengono usati i vecchi blocchi per velocita', quiete o encoder motore.

Nel menu iniziale **A** informa sulla calibrazione IMU nativa e **X** apre la precedente prova di offset e carreggiata, che richiede gli encoder motore. La nuova prova **Y** non misura la carreggiata e conserva gli altri parametri.

Dalla schermata del risultato, **B** torna alla guida dopo il rilascio dei tasti, così il pulsante usato per uscire non aziona subito il lift.

Questo robot usa `nativeImuHeading=true`: calibrazione nativa all'avvio e incrementi dell'angolo IMU senza bias locale né sostituzione con lo yaw delle ruote in quiete. La geometria usa le scale già configurate: non può scoprire da sola il vero diametro delle ruote, il rapporto di trasmissione o la scala assoluta del gyro. Non calibra automaticamente i coefficienti dinamici del controllo. Il fit confronta i due versi e rifiuta prove insufficienti o incoerenti; dopo averlo applicato, verifica una rotazione e una distanza misurate esternamente. [Procedura e limiti](docs/calibration.md).

## Hardware attuale

| Dispositivo | Porte | Cartucce / note |
| --- | --- | --- |
| Drivetrain sinistro | 5, -4, 6 | Blu; centrale invertito rispetto agli altri |
| Drivetrain destro | -7, 9, -10 | Blu; centrale invertito rispetto agli altri |
| Lift | 13, -14 | Verdi, controrotanti |
| Braccio | 20 | Verde |
| Intake | -15 | Un motore blu, invertito; comando massimo 12000 mV |
| Pinza | ADI D | Una elettrovalvola condivisa dai pistoni |
| IMU | 1 | Calibrazione all'avvio, robot fermo |
| Pod avanti / laterale | 2 / -3 | Diametro configurato 2.0 pollici |

I numeri negativi invertono il verso. Le ruote drive sono da 3.25 pollici; cartucce blu a 600 RPM e rapporto esterno nominale 360/600 producono 360 RPM alle ruote. La carreggiata configurata è 287 mm. Gli offset configurati sono: pod avanti a X=+20 mm, laterale a Y=−70 mm. Un profilo valido può sostituire questi parametri con quelli calibrati; [robot-config.hpp](include/robot-config.hpp) resta la fonte per porte e valori iniziali.

| Comando | Azione |
| --- | --- |
| Stick sinistro Y / destro X | Avanzamento / rotazione |
| R1 | Presa e alternanza del braccio; conserva le quote del lift superiori alla base |
| R2 | Rilascio e ritorno automatico |
| B, una pressione | Sale di un livello del lift |
| Freccia giù, una pressione | Scende di un livello del lift |
| Freccia destra / Y tenuti premuti | Muovono il lift nei due versi; rilascio ferma il comando manuale |
| L1 | Toggle pinza, anche durante i movimenti delle macro senza interromperli |
| X | Clic breve: toggle intake; pressione di almeno 0,4 s: inversione finché premuto, poi avanti |

R1 ha priorità su R2 se premuti insieme. Nei callback di arresto tutti i motori vengono fermati; la pinza conserva lo stato.

Il drivetrain usa `coast`: al rilascio non applica la frenatura elettrica. L'intake riparte automaticamente all'arrivo della pinza in basso; X permette il toggle con un clic breve e l'inversione con una pressione di almeno 0,4 s. Riceve il comando massimo nei due versi; il segno negativo della porta ne inverte il verso fisico.

Sul Brain, `DRIVER ENABLED | Pad:OK` indica che il field controller abilita la guida manuale e il telecomando risulta collegato. `DISABLED` indica che il campo mantiene i motori disabilitati; `Pad:OFF` o `Pad:ERROR` segnalano un problema di connessione o lettura del telecomando. `Auton profile: defaults (not tuned)` indica soltanto che manca un profilo di calibrazione valido: non blocca la guida manuale.

`Auton profile: partial` indica che sono state nexus_calibrate soltanto alcune categorie, come la geometria del menu; `fitted` indica che tutti i campi del profilo sono marcati come calibrati. Nessuna delle due diciture sostituisce una verifica fisica della precisione.

La riga `Heading ... [IMU]` conferma che l'ultimo incremento dell'angolo viene dal giroscopio. `[ENC]` indica il ripiego sugli encoder del drivetrain; `[NONE]` indica che manca una misura utilizzabile. La riga sotto mostra l'angolo IMU normalizzato a ±180°, oppure `IMU unavailable` / `IMU CAL FAILED`. L'IMU mantiene il conteggio continuo dei giri nei calcoli e nel CSV: -720° dopo due giri equivale a 0° sul display. Le origini di Heading e IMU possono differire dopo `setPose`. Per verificare, avvia il robot fermo, attendi la fine della calibrazione e ruotalo di 90° su un riferimento: entrambi gli angoli devono cambiare di circa 90°, tenendo conto del passaggio fra +180° e -180°. Non modificare PID o coefficienti del modello per correggere una lettura del sensore.

`Conf` riguarda tutta la posa, quindi X/Y e angolo, e non dipende dal numero assoluto di giri dell'IMU. La riga `Incertezza ...mm ...deg` mostra rispettivamente l'incertezza di posizione e di angolo del modello, non l'errore fisico misurato; `Slip` mostra il disaccordo fra sensori. Tornare allo stesso orientamento non dimostra che la posizione sia rimasta esatta: non si azzera la covarianza a ogni giro. È stata corretta la crescita artificiale dell'incertezza mentre i sensori confermano che il robot è fermo; i piccoli movimenti reali continuano a essere integrati. Con confidence bassa in movimento, annota le due incertezze, `[IMU]`/`[ENC]`, Slip, X/Y e il tempo dall'avvio prima di modificare parametri. Un test host verifica che cambiare il contatore di 720° lasci identiche stima e confidence.

Il driver interpreta lo stato IMU come maschera di bit: il bit di calibrazione è distinto dai bit dell'orientamento di montaggio. Confrontare lo stato intero con zero scarterebbe sensori validi in altri montaggi. [Implementazione PROS 4.2.1](https://github.com/purduesigbots/pros/blob/4.2.1/src/devices/vdml_imu.c).

I tasti sinistro/destro sullo schermo LCD del Brain scorrono sei pagine: posizione, IMU, sensori, contributi alla posizione (`ODOM 7`), quiete e tempi. La pagina IMU mostra accelerazioni, velocità angolari native, picchi e conteggi vicino ai limiti; non integra le accelerazioni in posizione. La pagina sensori mostra contatori cumulativi, validità, quiete riconosciuta, scarti e risultato della calibrazione IMU nativa. `Scarti` conta gli incrementi implausibili respinti; `ultimo` conserva F avanti, L laterale, S/D ruote, G IMU. La pagina ODOM scompone X/Y in avanzamento proiettato, lettura laterale e compensazione degli offset. La quiete non modifica l'heading IMU in questa configurazione. `TEMPI` misura il ciclo completo e conserva le pause oltre 50 ms. [Come interpretarli](docs/localization.md#diagnostica-dellimu).

In `ODOM 2`, `Angolo min/max` conserva gli estremi della rotazione integrata dall'ultimo reset, anche quando l'heading finale torna a zero. `Non usati F/L/G` conta gli aggiornamenti senza incremento utilizzabile di pod avanti, pod laterale e IMU, comprese invalidità e ricostruzione della baseline che possono lasciare `Scarti` a zero. Le misure diagnostiche vengono raccolte a ogni aggiornamento dello stimatore; confrontare i conteggi prima e dopo la prova.

## Localizzazione e controllo

- Stimatore con integrazione SE(2), covarianza, heading IMU nativo, rilevamento del disaccordo e fallback sui motori; compensazione del bias opzionale nella libreria, disattivata su questo robot.
- Correzioni opzionali da distanza: raycasting sul rettangolo campo, gating geometrico/statistico, conferme temporali e aggiornamento limitato. Una storia fissa consente di applicare e ripropagare misure ritardate.
- Recupero della posizione da almeno tre raggi attendibili e geometria non ambigua, da fermo e con heading ancora affidabile. Senza riferimenti assoluti un errore globale non osservabile richiede `setPose` su un riferimento noto.
- Traiettoria quintica, temporizzazione fisicamente vincolata e NMPC iLQR sul modello a tensione con cinque stati, attrito, asimmetrie, batteria e latenza.
- Polling software e stima nominali a 1000 Hz (1 ms, `odometryPeriodMs` in `robot-config.hpp`), ottimizzazione a 50 Hz; memoria fissa nei core, warm start, massimo cinque iterazioni e budget cooperativo di 12 ms. Il buffer PROS dei pod e della IMU si aggiorna ogni 10 ms: il polling a 1 ms non equivale a 1000 misure nuove al secondo. La pagina TEMPI mostra la cadenza effettiva sul Brain.
- Comandi elaborati da task persistenti: l'interruzione di un callback gara non lascia un mutex del controllo occupato. La riconnessione di pod e motori ripristina verso e impostazioni di lettura.

I sensori di distanza sono assenti per default (`port = 0`). IMU e pod possono essere disattivati con porta 0; meno sensori riducono osservabilità e prestazioni. Nessuna configurazione può ricostruire una traslazione laterale non misurata solo dagli encoder motore. Le misure distance del V5 non sono riferimenti millimetrici: rumore, campo reale ed extrinseci devono essere caratterizzati.

Approfondimenti: [localizzazione](docs/localization.md), [controllo](docs/controller.md), [calibrazione](docs/calibration.md), [validazione](docs/validation.md).

Prova odometrica attiva: `useDriveEncodersForOdometry=false` esclude tutti gli encoder della trazione, anche dai fallback; restano i due pod e l'IMU. Il Brain mostra `Encoder motori: ESCLUSI`. Per ripristinarli impostare `true` in `robot-config.hpp` e ricompilare.

Configurazione richiesta: nessun GPS e al massimo due Distance, preferibilmente con fasci perpendicolari. Sono disponibili correzioni locali X/Y, calibrazione individuale dei Distance e compensazione dei ritardi nel controller. La [guida alla precisione](docs/precision-2026-09-22.md) descrive montaggio, parametri, risultati delle prove e limiti del recupero con due sensori.

Per usi avanzati rimane disponibile `robot::chassis()`, con movimenti diretti, avvio asincrono e annullamento. `getPose()` restituisce mm/gradi; `diagnostics()` espone la fotografia interna di stima e controllo in unità SI. `src/robot.cpp`, `include/nexus/` e `src/nexus/` contengono l'implementazione della libreria e normalmente non richiedono modifiche per scrivere tasti o autonomous.

## Calibrazione e log

Per accelerazione, rilascio in coast, rotazioni e ingresso rapido in curva usa la
[caratterizzazione USB](docs/characterization.md): menu L1+R1, poi DOWN. Include
18 prove a intensita crescente, registrazione sul PC e analisi dei parametri
misurati; ogni prova parte soltanto tenendo R1.

Impostare `logTelemetry = true` per scrivere `/usd/nexus-log.csv` a circa 20 Hz in un task a priorità bassa. Il file viene sostituito ad ogni avvio: copiarlo prima della prova successiva. Il log distingue tensione comandata e tensione terminale misurata, con timestamp separato per i campioni motore. L'API PROS non espone timestamp hardware sincronizzati: l'istante di lettura è un'approssimazione, da considerare nel fit delle latenze.

```text
python tools/prepare_dynamics.py nexus-log.csv measurements
nexus_calibrate fit-wheel measurements/left.csv measurements/right.csv nexus.cfg HARDWARE_FINGERPRINT
```

`HARDWARE_FINGERPRINT` è il numero stampato dal robot sulla console seriale. La preparazione usa regressione locale centrata per l'accelerazione ed elimina finestre con dati invalidi o slip rilevato. La velocità motore rappresenta velocità a terra solo in assenza di slittamento; ispezionare il log e validare il modello su prove indipendenti. Per scala, gyro e offset usare misure fisiche esterne e gli altri comandi della [guida](docs/calibration.md).

Il profilo `nexus.cfg` include versione, campi effettivamente calibrati, identificativo hardware e CRC32; viene caricato dalla microSD con recupero dal backup. Se manca, il robot usa i valori iniziali e lo indica sul display.

Per conservare una calibrazione precedente, rinominare il profilo sulla microSD in `nexus.cfg` e l'eventuale backup in `nexus.cfg.bak`. Il formato binario e il fingerprint hardware restano compatibili: la sola rinomina della libreria non richiede una nuova calibrazione. I nuovi log sono salvati in `nexus-log.csv`.

Per migrare da una versione in pollici/metri: convertire esplicitamente i vecchi log prima di elaborarli. Le nuove intestazioni del log e la dichiarazione di unità dei CSV di calibrazione evitano che valori in metri vengano interpretati come mm. Il passaggio della geometria configurabile ai mm cambia anche il fingerprint del robot: i profili con il vecchio identificativo vengono rifiutati. Dopo aver verificato le misure, rigenerare il profilo con l'identificativo corrente stampato sulla console, seguendo la [guida alla calibrazione](docs/calibration.md).

## Compilazione e test

All'avvio: attesa di 1 secondo, preMatch automatico una sola volta anche con field controller disabilitato, **A sul controller** oppure pulsante touch **READY**, poi attesa di 1 secondo prima di inizializzare gyro e odometria. I cambi di modalità non ripetono il preMatch; driver e autonomous rispettano l'abilitazione di gara. Macro di guida, porte sensori, tasti e parametri di taratura: [braccio, lift e intake](docs/mechanisms.md).

Firmware dal terminale PROS:

```text
make quick
```

Per aggiornare IntelliSense dopo aver aggiunto sorgenti o modificato i flag del Makefile, eseguire `python tools/update_compile_commands.py`. Il comando rigenera `compile_commands.json` per tutti i sorgenti del firmware usando il compilatore ARM PROS e i flag effettivi della build. La configurazione C/C++ di VS Code usa C++23 anche per gli header. Se restano diagnostiche precedenti, eseguire **C/C++: Reset IntelliSense Database** dalla palette dei comandi e poi **Developer: Reload Window**.

Per ottenere la libreria ARM separata, eseguire `make library IS_LIBRARY=1`: produce `bin/NEXUS.a` con i soli moduli di `src/nexus/`. I file di configurazione hardware, i tasti e i callback gara restano nell'applicazione. Gli header da usare insieme all'archivio sono in `include/nexus/`.

Test e programma di calibrazione sul PC, con un compilatore C++23 e CMake:

```text
cmake -S tests -B .test-build
cmake --build .test-build --config Release
ctest --test-dir .test-build -C Release --output-on-failure
```

Dodici suite per stimatore, controller, calibrazione offline e automatica, procedura di rotazione, sistema completo, preparazione dei log, conversioni delle unità autonomous, pulsanti, meccanismi, avvio e adattatore Chassis. Il test del controller usa un plant RK4 indipendente dal rollout interno; quello completo aggiunge rumore, mismatch, drift gyro, slip, spinte e disconnessioni. I test stampano errori rispetto alla verità simulata, ripetibilità, tempi e costo del solver. La build e i test host verificano il software: precisione, tempi reali e cambi modalità vanno provati sul robot.

