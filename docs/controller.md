# Controllo del movimento: modello, traiettorie e NMPC

Il nucleo `nexus::Controller` lavora in **metri, secondi, radianti e volt**. L'asse X punta a destra, Y in avanti, l'angolo cresce in senso orario a partire da +Y. Una velocità maggiore delle ruote sinistre produce una rotazione oraria. Il verso elettrico dei singoli motori e le cartucce appartengono alla configurazione hardware: il controllore riceve già velocità e tensioni con questi segni coerenti.

Il codice dell'autonomous si scrive in `src/main.cpp` con `robot::Auton`, come descritto nel [README](../README.md): posizioni e tolleranze in mm, velocità in mm/s, heading in gradi. Questa interfaccia usa lo stesso chassis e lo stesso controllo descritti qui. Anche i valori modificabili in `robot-config.hpp` usano mm, mm/s e mm/s², tranne i diametri delle ruote in pollici. I coefficienti modificabili `kV` e `kA` sono rispettivamente in V/(mm/s) e V/(mm/s²). Il wrapper converte automaticamente verso le unità SI del nucleo. Le strutture qui illustrate sono interne e servono per calibrazione, manutenzione della libreria e simulazione; non occorre modificarle per assegnare i tasti o scrivere l'autonomous.

## Architettura effettivamente implementata

Ogni movimento comprende una traiettoria geometrica, la sua temporizzazione e un'ottimizzazione non lineare ripetuta sullo stato stimato corrente. Il riferimento comprende posizione, orientamento, velocità lineare, velocità angolare e tensioni anticipate dal modello.

La geometria usa una spline quintica di Hermite, con tangenti iniziale e finale e derivate seconde nulle agli estremi. `MoveToPose` impone anche la tangente finale; `MoveToPoint` lascia libero l'orientamento finale. In retromarcia le tangenti di percorrenza sono opposte alla direzione del robot. Quando tangenti opposte e collineari produrrebbero una cuspide, un termine trasversale di sesto grado regolarizza la curva mantenendo le condizioni agli estremi. Le rotazioni sul posto usano una legge quintica temporale. Il riferimento è memorizzato in 129 campioni fissi.

Per un punto nel semipiano davanti alla direzione di percorrenza, la tangente finale libera segue quella dell'arco circolare che congiunge partenza e bersaglio. Per esempio, da heading 0° verso (600, 600) mm viene scelta una tangente finale di 90°: evita una controsterzata per tornare alla diagonale di 45°. `MoveToPose(600, 600, 0)` mantiene invece il muso finale a 0° e richiede una curva a S. La ripianificazione terminale usa la stessa tolleranza dell'arrivo, senza una fascia aggiuntiva in cui il robot potrebbe fermarsi fuori tolleranza senza correggersi.

Da **CTRL 8**, le mosse con heading imposto e distanza iniziale oltre 350 mm completano la curva prima del bersaglio: il tratto finale e' rettilineo, lungo il 15% della distanza e al massimo 120 mm, con il muso gia' orientato come richiesto. La curva e il rettilineo si congiungono a curvatura nulla, senza fermata intermedia; la temporizzazione e i limiti di accelerazione si applicano al percorso completo. Sono usati 96 intervalli per la curva e 32 per l'avvicinamento. Lo stesso criterio vale in retromarcia. Le correzioni corte mantengono la geometria precedente. Questo anticipa l'allineamento senza allargare la tolleranza di arrivo.

La temporizzazione limita velocità delle ruote, velocità angolare e accelerazione laterale. Passaggi avanti/indietro limitano l'accelerazione tenendo conto anche della variazione della curvatura. Questo riferimento è una buona inizializzazione fisica, non il risultato di un'ottimizzazione globale del percorso o del tempo minimo. Non comprende evitamento ostacoli né vincoli rispetto ai bordi del campo: i punti dell'autonomous devono lasciare spazio alla curva, soprattutto quando il bersaglio si trova dietro il robot.

La verifica finale misura anche l'accelerazione di entrambe le ruote nei segmenti del riferimento interpolato. Se supera il limite del modello, dilata il tempo e riduce le velocità in modo coerente. Questo passaggio corregge i picchi dovuti a rapide variazioni della curvatura che le passate di raggiungibilità possono sottostimare. Le tangenti restano proporzionali alla distanza anche nelle correzioni terminali di pochi millimetri, per evitare anse introdotte da una lunghezza minima fissa. La dilatazione può ridurre la velocità iniziale del riferimento nei percorsi più stretti; il solver continua a partire dalla velocità realmente stimata e gestisce il transitorio di frenata.

Il controllo è un NMPC realizzato mediante **iLQR con vincoli sulle tensioni**:

1. Predice 20 intervalli di 75 ms, per un orizzonte di 1,5 s.
2. Riutilizza e trasla temporalmente la precedente sequenza di comandi.
3. Linearizza analiticamente il modello lungo la previsione corrente.
4. Esegue un passaggio Riccati all'indietro con regolarizzazione e soluzione esatta dei piccoli problemi quadratici con due ingressi e limiti di tensione.
5. Verifica i candidati sul modello non lineare mediante ricerca del passo. Accetta soltanto riduzioni del costo.
6. Applica il primo comando e ripete dall'ultima stima disponibile.

Sono ammesse al massimo cinque iterazioni, con passi di ricerca `1, 0.5, 0.25, 0.1, 0.025`. Le matrici e le traiettorie hanno dimensioni fisse; `start()` e `update()` non allocano memoria dinamica. La ripianificazione geometrica si attiva anche dopo grandi deviazioni dal riferimento o quando rimane un errore terminale e il robot è quasi fermo. Questa ripianificazione aiuta il metodo locale a correggere errori laterali che una linearizzazione di un drivetrain fermo non può risolvere direttamente.

La formulazione segue la famiglia di metodi descritta da Tassa, Mansard e Todorov in [Control-Limited Differential Dynamic Programming](https://roboti.us/lab/papers/TassaICRA14.pdf). Qui si usano Jacobiane della dinamica e un'approssimazione di Gauss–Newton, quindi iLQR, senza le derivate seconde della dinamica del DDP completo. I due ingressi consentono di risolvere il sottoproblema vincolato enumerando soluzione interna e quattro lati, senza un solver generico.

## Modello e calibrazione

La velocita' laterale misurata entra nella previsione: `dx/dt = v*sin(theta) + lateralV*cos(theta)` e `dy/dt = v*cos(theta) - lateralV*sin(theta)`. **CTRL 7** manteneva questo disturbo costante nel telaio per tutti gli 1,5 s, anche oltre la frenata: poteva prevedere uno spostamento laterale persistente e indurre controcorrezioni eccessive. **CTRL 8** usa nel solver `lateralV(t) = lateralV_misurata * exp(-t / 0.7 s)`, aggiornato a ogni lettura. Lo stesso valore, precalcolato al centro di ciascun intervallo, entra nei rollout, nelle Jacobiane e nella ricerca del passo. I 0,7 s sono una scelta di persistenza della previsione verificata nelle simulazioni, non una misura dell'aderenza del robot. La compensazione della breve latenza e il metodo diagnostico `predict()` mantengono invece il valore misurato. Non cambia la posa dello stimatore e non e' un modello completo dell'inerzia laterale: non predice lo slittamento nuovo prodotto dalle curve future.

In una ripianificazione la velocita' iniziale viene compensata per la scala di velocita' applicata da `sample()`, evitando di rallentarla due volte. I limiti di raggiungibilita' e la dilatazione temporale possono comunque ridurla se la nuova curva lo richiede.

Per `moveToPoint`, entrando nella tolleranza di posizione il riferimento passa alla frenata terminale senza imporre l'orientamento finale della curva. `moveToPose` e `turnToHeading` mantengono il vincolo angolare.


Lo stato interno è `[x, y, theta, velocitaSinistra, velocitaDestra]`. Per ogni lato:

```text
accelerazione = (tensione - caricoStimato - kV * velocita
                 - kS * tanh(velocita / 0.07)) / kA
```

La velocità interna è quella tangenziale della ruota, in m/s; la costante 0.07 corrisponde a 70 mm/s. Il modello permette valori diversi a sinistra e a destra. L'integrazione usa il metodo del punto medio con sottointervalli non superiori a 10 ms, propagando anche le Jacobiane. L'attrito statico è approssimato con una funzione continua: non è un modello esatto dello stacco da fermo, delle deformazioni delle ruote o della corrente dei motori.

Durante ciascun movimento un osservatore stima un carico aggiuntivo per lato, espresso in volt. Confronta le velocità misurate con quelle previste dalla cronologia dei comandi nello stesso intervallo temporale e aggiorna il carico con filtro a 120 ms, variazione massima di 4 V/s e ampiezza massima pari al minore fra 3 V e il 40% della tensione disponibile. Salta gli aggiornamenti con slip almeno 0,25 o intervallo fuori da 5–100 ms. La stessa compensazione entra nella previsione e nel feedforward: serve a ridurre l'errore persistente quando attrito e risposta reale differiscono dai parametri iniziali, evitando ripetute manovre terminali. Si azzera a ogni nuovo movimento o dato invalido e non modifica il profilo di calibrazione salvato.

La tabella descrive `nexus::DynamicsConfig` nel nucleo SI, dopo la conversione dai valori del file di configurazione del robot. Anche i profili persistenti mantengono questo formato interno.

| Parametro | Significato | Unità |
| --- | --- | --- |
| `trackWidth` | Larghezza efficace durante la rotazione | m |
| `kVLeft`, `kVRight` | Tensione necessaria per la velocità | V / (m/s) |
| `kALeft`, `kARight` | Tensione necessaria per l'accelerazione | V / (m/s²) |
| `kSLeft`, `kSRight` | Termine di attrito | V |
| `maxAcceleration` | Accelerazione longitudinale massima del modello | m/s² |
| `maxLateralAcceleration` | Accelerazione laterale di riferimento e penalizzazione | m/s² |
| `maxSpeed` | Velocità tangenziale massima delle ruote | m/s |
| `maxOmega` | Velocità angolare massima | rad/s |
| `maxVoltage` | Limite della tensione, comunque non superiore a 12 V | V |
| `commandLatency` | Ritardo tra calcolo e applicazione del comando | s |

**I valori predefiniti sono iniziali e non calibrati sul robot.** Prima di attribuire significato fisico a una precisione millimetrica bisogna misurare scale delle ruote, geometria efficace, segni, ritardo e coefficienti del drivetrain. L'identificazione va ripetuta quando cambiano trasmissione, massa, ruote o distribuzione del carico. Verificare separatamente il comportamento a bassa velocità, la frenata e la differenza fra i due lati. Il filtraggio della velocità nello stimatore introduce un ritardo distinto da quello degli attuatori.

## Vincoli e incertezza

Le tensioni sono limitate esplicitamente dal minimo fra tensione batteria e `maxVoltage`, sia nel sottoproblema del solver sia nei rollout. L'accelerazione longitudinale è saturata nel modello. Questa saturazione descrive una capacità fisica presunta; non certifica che le ruote reali non slittino.

Velocità delle ruote, velocità angolare e accelerazione laterale sono vincoli **morbidi** nell'obiettivo dell'NMPC, oltre ai limiti della temporizzazione. Possono quindi essere superati durante correzioni o disturbi; non sono invarianti garantiti del sistema reale. La penalizzazione usa Hessiane di Gauss–Newton.

La covarianza di posizione e angolo, lo slittamento rilevato, lo stato `degraded` e la tensione batteria riducono la velocità del riferimento e i limiti penalizzati. Il controllo non richiede sensori di distanza: utilizza qualunque stima valida prodotta dallo stimatore. Con stato `lost`, dati di stato non finiti, covarianza non finita o con diagonale negativa, slip non finito o batteria non valida, restituisce tensione nulla e segnala un risultato non valido. Lo stesso controllo dei dati impedisce di dichiarare l'arrivo su una stima inutilizzabile.

`MotionOptions::reverse` sceglie il verso della traiettoria iniziale. Per correggere una posizione ancora fuori tolleranza al termine del percorso, a velocita' traslazionale inferiore a 150 mm/s, il planner confronta percorsi nel verso corrente e in quello opposto. Da **CTRL 9** il punteggio comprende la durata, il tempo di frenata se il moto e' contrario al candidato e una penalita' di **0,35 s per radiante di rotazione totale lungo il percorso**. Si conta tutta la variazione dell'heading: una S che finisce allo stesso angolo iniziale non ha costo angolare nullo. E' una preferenza per manovre piu' dolci, non una stima certificata del tempo minimo. Cambia candidato soltanto se il punteggio migliora di oltre il 10% piu' 120 ms; fra ripianificazioni trascorrono almeno 600 ms. Il movimento successivo riparte dal proprio parametro `reverse`.

Per un errore prevalentemente laterale entro 350 mm, confronta anche manovre composte: curva verso una posa oltre il bersaglio, riferimento fermo per 120 ms e ritorno rettilineo nel verso opposto. La distanza di riallineamento parte da `sqrt(4 * trackWidth * erroreLaterale)`, limitata a 120-300 mm; viene valutata anche una distanza maggiore del 40%, fino a 400 mm. Questo sostituisce i 60-220 mm delle revisioni precedenti e offre spazio per raddrizzarsi prima dell'arrivo. Il confronto comprende entrambe le direzioni iniziali. I due tratti condividono i limiti di accelerazione e i 129 campioni disponibili. Al termine della manovra il verso memorizzato per preferire candidati equivalenti e' quello dell'ultimo tratto, non quello del primo.

La manovra selezionata resta il riferimento fino al completamento, salvo uno scarto superiore a 250 mm che richiede una ripianificazione. Il solver continua a correggere durante l'esecuzione: un solo cambio di verso nel piano non garantisce esattamente una sola inversione fisica. Le tolleranze richieste restano quelle dell'utente; una correzione residua puo' ancora servire. La posa intermedia richiede spazio aggiuntivo fino a 400 mm oltre il bersaglio e il planner non rileva ostacoli. La sosta del riferimento al cambio di verso e' temporizzata, non un sensore di arresto del robot.


Il robot con sole ruote omni usa ora un limite iniziale di accelerazione laterale di **1000 mm/s²**, rispetto ai precedenti 3000; limite rettilineo e velocità massima restano invariati. È una scelta iniziale prudente da verificare sul campo, non una misura del coefficiente di attrito. Anche [LemLib distingue il parametro per la velocità in curva](https://lemlib.readthedocs.io/en/stable/api/chassis.html#_CPPv4N6lemlib10DrivetrainE) dai parametri di odometria e raccomanda una regolazione diversa per ruote omni e traction. Il nucleo predittivo resta a cinque stati: non è stato aggiunto un modello di inerzia laterale con parametri non misurati. La velocità laterale osservata entra invece nel riconoscimento dell'arresto e nell'abilitazione delle correzioni.

## Budget temporale e telemetria

Il controllore conserva in un buffer fisso di 260 elementi il tempo di emissione e le due tensioni di ogni comando. La previsione fino all'applicazione del prossimo comando ricostruisce i comandi già in viaggio, invece di applicare l'ultimo comando per l'intera latenza. Il buffer copre la latenza massima di 150 ms e un'età massima dell'osservazione di 100 ms, con aggiornamenti distanziati di almeno 1 ms; non alloca memoria nei task. Il ritardo configurato resta una stima fisica: variazioni reali del ritardo e parametri motore errati producono comunque errore di previsione.

`start()`, `reset()` e la riconfigurazione cancellano la cronologia; prima del primo comando registrato la previsione assume tensione nulla. Comandi manuali o di un movimento precedente ancora in transito non sono ricostruibili da questa cronologia locale. Nei primi `commandLatency + observationAge` secondi di una nuova richiesta questa è quindi un'approssimazione, soprattutto se il robot è già in moto; non viene nascosta attribuendo alla previsione una misura inesistente.

Il quarto argomento facoltativo di `update(estimate, dt, batteryVolts, observationAge)` specifica in secondi quanto è vecchia la fotografia dello stimatore. Il controllo la propaga prima all'istante corrente, poi all'arrivo del nuovo comando, usando la cronologia delle tensioni in entrambi gli intervalli. Non modifica lo stato né la covarianza pubblicati dallo stimatore. `observationAge` deve essere finito e fra 0 e 100 ms; valori invalidi producono comando nullo. Il `dt` passato deve essere il tempo effettivo trascorso fra due chiamate, anche quando differisce dai 20 ms nominali. L'orologio installato con `setClock()` misura soltanto il budget del solver e non sostituisce questi timestamp.

Nei confronti host con plant RK4 indipendente, ciclo variabile fra 13 e 27 ms e ritardi di comando 10/40/80/120 ms, tutti i dodici casi nominali curva/frenata/reverse raggiungono 1 mm e 0,1°. Queste tolleranze molto strette non sono robuste a tutti i mismatch: con errori fino al 5% su `kV` e 15% su `kA`, tre dei dodici casi non convergono entro 14 s; il maggiore errore terminale osservato è circa 6,74 mm e possono restare grandi errori angolari durante le ripianificazioni. Il modello precedente presenta anch'esso fallimenti; il confronto non prova un miglioramento universale. Su nove casi separati, con geometrie e mismatch diversi, ritardi 25/65/105 ms e jitter di trasporto ±2 ms, le tolleranze operative restano raggiunte e il massimo errore terminale simulato passa da 8,59 a 3,91 mm. Nessuno di questi numeri è una precisione misurata sul V5.

Con osservazioni artificialmente vecchie di 5/10/20 ms e ritardo comando di 10 ms, la compensazione temporale riduce l'errore RMS di tracking simulato da 6,85/9,53/18,18 mm a circa 5,82 mm. La taratura della latenza e dei coefficienti dinamici resta necessaria; non viene introdotta un'identificazione automatica dei motori basata su questi soli test.

`setClock()` riceve un puntatore a funzione che restituisce secondi monotoni. Il wrapper VEX fornisce l'orologio della piattaforma. `setBudgetMs()` imposta il budget cooperativo, inizialmente 12 ms. Il limite viene controllato fra blocchi di calcolo, durante le linearizzazioni, nel passaggio all'indietro e durante i rollout della ricerca del passo. Se un candidato è interrotto prima di valutarne l'intero orizzonte, viene scartato: resta la migliore sequenza completa già valutata.

Quando scade il budget, il controllore restituisce la migliore sequenza completa già valutata, inclusa l'inizializzazione se non è stata completata alcuna iterazione. Il budget **non interrompe un'istruzione in corso** e non è una garanzia rigida sul tempo massimo: un rollout iniziale o un blocco appena iniziato possono terminarlo oltre la soglia. Senza un orologio rimangono i limiti sul numero di iterazioni, ma `computeMs` resta zero e non si applica un limite temporale misurato.

`SolverStats` rende disponibili costo iniziale/finale, iterazioni, durata, validità, raggiungimento del budget e convergenza numerica. `converged` indica una piccola riduzione del costo o un piccolo passo del solver locale: non dimostra un ottimo globale, il rispetto esatto dei vincoli morbidi o l'arrivo del robot.

L'arrivo si verifica separatamente con `settled(estimate, dt)`: errore di posizione entro la tolleranza richiesta, angolo entro tolleranza se imposto, velocità traslazionale `hypot(v, lateralV)` inferiore a 35 mm/s e velocità angolare inferiore a 4°/s, mantenuti per `settleTime`. Quindi uno scivolamento laterale non viene più considerato un arresto. Per le rotazioni sul posto non si impone la posizione. Il wrapper chassis gestisce scadenza del movimento, annullamento e applicazione del comando; il nucleo matematico non usa `MotionOptions::timeout` come orologio dell'autonomous. Il display e il CSV esprimono distanze e velocità lineari in mm e mm/s; `diagnostics()` restituisce invece la fotografia interna dello stimatore e del solver in SI.

## Task PROS e cambi di modalità

Il callback gara può essere eliminato dal sistema quando cambia modalità. Per questo i callback pubblici inviano richieste a task persistenti: l'attesa dell'utente non possiede un mutex del nucleo che possa restare bloccato quando il callback viene eliminato. Il worker gestisce i comandi, le generazioni delle richieste e l'annullamento; i task di stima e controllo continuano a esistere. Il cambio modalità e `disabled()` fermano il movimento. Le copie della diagnostica non richiedono che il callback utente conservi un lock interno.

La modalità calibrazione acquisisce separatamente il drivetrain, limita la tensione e richiede rinnovi del comando entro 100 ms. Il suo task gestisce campionamento, fit e persistenza senza usare il task NMPC per scrivere file. Il driver normale resta semplice in `main.cpp`; il controllo `calibrationActive()` sospende le sue assegnazioni mentre la procedura è attiva.

`Sequence` conserva modalità e generazione dalla costruzione e aggiorna la generazione per i propri comandi. Un comando diretto esterno che prende il controllo interrompe i passi successivi della sequenza anche nella stessa modalità. Una sequenza precedente non può riprendersi il drivetrain o azzerare la posa del nuovo movimento; le richieste accodate conservano modalità e token originali, verificati prima di modificare lo stato. Nei callback gara resta necessario l'arresto di `disabled()` presente nel progetto.

La scadenza (`timedOut`) arresta le ruote e conclude soltanto la mossa corrente. `Sequence` e `robot::Auton` continuano con la mossa, l'attesa o l'azione successiva, senza azzerare la posa; `result()` conserva il timeout finché una nuova mossa produce il proprio esito. Annullamenti, richieste non valide e guasti ai sensori o al solver bloccano ancora la sequenza. Le regressioni dell'adattatore verificano tre timeout consecutivi (punto, posa, rotazione), le azioni intermedie, un successivo arrivo riuscito e il rispetto degli annullamenti.

Su questo robot il pod laterale configurato è necessario durante i movimenti automatici: `requireLateralPodForMotion` arresta con `sensorFault` dopo 150 ms di letture L mancanti e impedisce di dichiarare l'arrivo durante l'assenza. La libreria lascia questa opzione disattivata per i robot che usano intenzionalmente un'altra configurazione sensori.

## Validazione riproducibile

`tests/controller_test.cpp` simula un impianto RK4 indipendente con passi di 1 ms, mentre il solver usa il proprio integratore del punto medio. Comprende rettilineo, retromarcia, curva con angolo finale, punto con angolo libero, bersaglio dietro il robot, movimenti corti, frenata da 1400 mm/s, rotazione di 170°, batteria a 7,5 V, spinta, collisione/slittamento, asimmetria dei coefficienti motore, ritardo attuatori, alta velocità, tolleranza di 1 mm e ripetibilità deterministica. Controlla inoltre tensioni, monotonia dei costi accettati, budget esaurito, stima persa, dati non finiti e riduzione dell'aggressività con incertezza.

Otto casi aggiuntivi verificano l'arrivo al primo avvicinamento con attrito più alto e asimmetrico, sia continuo sia con soglia di stacco da fermo, errori su kV/kA, rumore di velocità, latenza di 10 ms e una spinta durante la curva. Usano pose a destra, a sinistra, in retromarcia e con orientamento finale diverso. Richiedono arrivo entro 10 mm in meno di 4 secondi, meno di 1,2 secondi dopo il primo ingresso a 50 mm e meno di 85 mm percorsi da quel momento.

Le sedici regressioni sulla scelta del verso aggiungono un errore terminale di 90 mm longitudinali, 25 mm laterali e 5° dopo un primo avvicinamento completo. Coprono sia punti con heading libero sia pose con heading imposto, bersagli davanti e dietro, richieste iniziali avanti e reverse, attraversamento di ±180°, attrito asimmetrico e rumore. Verificano l'arrivo senza un giro di allontanamento, la stabilità del verso scelto e il ripristino del verso esplicito alla richiesta successiva.

L'output misura errore finale, errore angolare, tempo simulato, errore RMS dal riferimento e tempi medi/massimi del solver host. Le prove del solo controllore assumono uno stato disponibile e corretto; la validazione con sensori e stimatore è separata. Non costituiscono una misura di accuratezza sul V5, né dimostrano superiorità su ogni altro controllore. Prima dell'uso in gara occorre misurare la durata sul V5, inclusi casi di ripianificazione, e confrontare errori, ripetibilità e frequenza di timeout su prove fisiche ripetute.

### Diagnostica CTRL 9

La pagina principale identifica la build con `CTRL 9`; il campo versione di `NXCTRL` vale 9. Quando lo stream USB odometrico e' attivo, le righe aggiuntive `NXCTRL` a 10 Hz riportano obiettivo, riferimento, velocita' avanti/laterale, yaw rate, tensioni comandate, stato, iterazioni e budget del solver. L'header specifica unita' e ordine. Il timestamp e la generazione del controllo identificano il riferimento; le velocita' sono l'ultima fotografia disponibile dello stimatore, non una misura atomica al medesimo istante. La diagnostica resta nel task USB separato; il formato `NXOD` rimane compatibile.

La regressione omni esegue otto diagonali da 600x600 mm, speculari, su un impianto indipendente con quattro resistenze laterali sintetiche, ritardo di 10 ms e filtro della velocita'. Verifica arrivo e assenza di giri completi ripetuti; questi parametri non sono misure del robot. Il confronto non identifica da solo la causa fisica dell'errore odometrico.


### Arrivo anticipato: confronto CTRL 7 / CTRL 8

Le dodici regressioni `checkAnticipatedArrival` usano un plant RK4 indipendente, filtro velocita' a 70 ms, ritardo di 10 ms, attrito asimmetrico e coefficienti motore diversi da quelli del controller. Comprendono diagonali speculari 600 x 600 mm, rettilinei, resistenza laterale sintetica e spinte di 45 mm / 9 gradi durante l'avvicinamento. La tolleranza resta 10 mm con heading imposto. Si misurano tempo e percorso dopo il primo ingresso a 50 mm dal bersaglio, non soltanto l'errore finale.

| Quattro diagonali con deriva laterale | CTRL 7 | CTRL 8 |
| --- | --- | --- |
| Tempo totale | 4,34-4,58 s | 3,16-3,24 s |
| Assestamento dopo il primo ingresso a 50 mm | 2,02-2,24 s | 0,54-0,64 s |
| Percorso da quel momento | 167-174 mm | 50-55 mm |
| Retromarcia da quel momento | 57-59 mm | 0 mm |

Tutti i dodici casi raggiungono la tolleranza. I due rettilinei con spinta tardiva richiedono ancora una correzione terminale; l'allineamento anticipato non garantisce l'arrivo al primo passaggio in qualunque condizione. Nelle diagonali senza deriva e senza spinta, il tratto di allineamento aggiunge circa 0,4 s: il planner privilegia un arrivo regolare, non il tempo minimo teorico. Le otto regressioni omni precedenti, con quattro resistenze laterali, restano verificate. I numeri sono simulazioni e non una validazione fisica sul V5. I confronti numerici delle sezioni precedenti descrivono le rispettive revisioni precedenti.

Questa modifica non corregge un eventuale errore della posizione stimata. Inoltre una richiesta finale di 1 mm, come quella attualmente presente in `main.cpp`, puo' ancora richiedere correzioni fini: la tolleranza scelta dall'utente resta invariata.


### Correzione ampia: confronto CTRL 8 / CTRL 9

Le 24 regressioni `checkWideCorrection` partono da residui laterali speculari di 3, 6, 12, 25, 50 e 80 mm, con tolleranze di 1 e 5 mm, disallineamento iniziale di 3 gradi, filtro delle velocita', ritardo e motori asimmetrici. Il plant indipendente comprende inerzia laterale sintetica. Verificano arrivo, massimo allontanamento, rotazione totale e cambi di verso effettivi e del riferimento. Per residui di almeno 25 mm il riferimento ha un solo cambio di verso; la velocita' reale puo' includere una seconda inversione breve. Tutti i 24 casi raggiungono la tolleranza in meno di 5,5 s, entro 350 mm dal bersaglio e con meno di 140 gradi di rotazione totale.

Nei quattro casi con residuo di 50 mm:

| Misura simulata | CTRL 8 | CTRL 9 |
| --- | --- | --- |
| Rotazione totale | 169-203 gradi | 59-65 gradi |
| Massimo allontanamento dal bersaglio | circa 56 mm | 212-213 mm |
| Distanza percorsa per correggere | 148-151 mm | 486-491 mm |
| Tempo | 2,60-3,78 s | 2,70-3,58 s |

La scelta percorre piu' strada con meno sterzate e non e' sempre piu' veloce: nel caso piu' impegnativo da 80 mm / tolleranza 1 mm il tempo passa da 3,74 a 5,00 s, riducendo la rotazione da 208 a 123 gradi. Le regressioni degli avvicinamenti introdotte con CTRL 8 continuano a passare. La validazione fisica sul robot resta necessaria.
