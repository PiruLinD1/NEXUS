# Macro braccio, lift e intake

Collegare il **V5 Rotation alla porta 19**, il motore braccio alla **20** e i motori lift alle **13 e 14** (14 invertita). L'ottico non viene usato: il conteggio degli oggetti è manuale. Porte e parametri sono in `include/robot-config.hpp` e `include/robot/mechanism_config.hpp`.

Il Rotation usa la posizione continua. Il rapporto encoder/braccio è 7:1: 630°, 1050° e 1008° encoder corrispondono a 90°, 150° e 144° del braccio. Gli angoli richiesti restano espressi in gradi fisici del braccio; la posizione encoder di passaggio è ricavata da `armClearanceDeg * encoderDegreesPerArmDegree`. Il preMatch apprende il verso dell'encoder; il secondo finecorsa è quello fisico misurato durante la calibrazione, non un angolo fisso inventato.

Dopo il contatto confermato con **entrambi i finecorsa del braccio**, il motore passa subito in **COAST**, senza BRAKE né HOLD. Resta libero durante le attese, i movimenti del solo lift, l'annullamento e la disabilitazione, fino al prossimo movimento del braccio o a un nuovo preMatch. Questa regola prevale sui mantenimenti HOLD descritti sotto quando il braccio è già fermo a un finecorsa. Nelle posizioni intermedie resta il normale mantenimento HOLD; una nuova manovra manuale lo ripristina anche al successivo rilascio dei tasti.

Dopo il preMatch, **A sul controller** conferma READY come il pulsante sul Brain. Posiziona prima il robot e poi premi A: dopo un secondo comincia la calibrazione della navigazione, durante la quale il robot deve restare fermo. Serve una nuova pressione nella schermata READY; tenere A durante il preMatch non conferma automaticamente. A non riavvia un preMatch interrotto.

## Comandi in driver

| Tasto | Azione |
| --- | --- |
| R1 | Chiude pinza, spegne intake e attende **300 ms** prima di muovere lift/braccio; alterna **150° con profilo rapido ↔ secondo finecorsa alto** |
| R2 | Da 150° porta rapidamente il braccio a circa **90° senza PID**, aspetta **200 ms** e apre la pinza; dal secondo finecorsa **apre subito senza passare a 90°**. Poi avvia lo stesso ritorno automatico |
| X | Clic breve: accende/spegne intake al rilascio fuori dalle macro. Tenuto per almeno **0,4 s**: gira al contrario finché premuto, poi subito avanti |
| L1 | Apre/chiude la pinza anche durante le macro, senza interrompere i movimenti |
| B | Ogni pressione aggiunge **1,38 rotazioni** dalla quota corrente o già richiesta, anche dal fondo; tratto residuo solo al limite di corsa |
| GIÙ | Ogni pressione abbassa il lift di **1,38 rotazioni**, fino al fondo; dal fondo permette altre **0,68 rotazioni** solo con il braccio al secondo finecorsa alto |
| DESTRA/Y | Muovono il lift nei due versi finche' restano premuti |

L'intake riparte automaticamente in avanti quando la pinza raggiunge la posizione bassa completa: braccio al finecorsa zero e lift al fondo normale, dopo il preMatch all'ingresso in driver oppure al termine del ritorno R2. La ripartenza avviene una volta all'arrivo, quindi un clic breve su X può ancora spegnerlo mentre resta in basso. R1 e un nuovo rilascio R2 fermano l'intake; la pausa R1 mantiene la precedenza. La pressione lunga di X è disponibile anche durante le macro di guida, eccetto la pausa R1 e gli arresti per errore. Dopo **400 ms** (`intakeReverseHoldMs`) comanda l'inversione per tutta la pressione e al rilascio avvia subito il verso normale, anche se prima era spento. Disabilitazione, perdita del controller, calibrazione telaio e letture meccaniche non valide fermano l'intake e annullano la pressione lunga.

In driver abilitato e con controller collegato, fuori dal menu di calibrazione del telaio e dopo il preMatch, B e GIÙ selezionano l'altezza del lift. Ogni nuova pressione aggiunge o sottrae un passo dalla quota corrente o già richiesta; tenere premuto non ripete il comando. I comandi sono disponibili a robot pronto, in manuale, durante i movimenti R1 del braccio e dopo un arresto riprendibile. Durante la pausa R1 e la sequenza R2 non avviano regolazioni. R1/R2 hanno priorità sui comandi di altezza nello stesso ciclo; B e GIÙ premuti insieme si annullano.

Dopo il preMatch o un ritorno automatico completo a zero, la prima pressione di R1 seleziona 150°, la seconda il finecorsa, la terza 150° e così via. Le pressioni sono accettate anche durante il movimento: non c'è una finestra speciale per il doppio clic e tenere premuto conta una sola volta. Due pressioni rapide da zero selezionano direttamente il finecorsa, senza sosta intermedia a 150°.

R1 può interrompere la presa, il rilascio o qualsiasi fase del ritorno: chiude subito la pinza, ferma il lift e mantiene il braccio in HOLD per almeno **300 ms** (`pickupClampPauseMs`). Solo dopo prosegue verso la nuova destinazione. Ogni nuova pressione di R1 aggiorna la destinazione e fa ripartire l'intera pausa, anche durante una salita o un movimento del braccio; tenere premuto non la riavvia. Durante la pausa B e GIÙ non avviano movimenti, mentre R2 viene memorizzato per dopo la pausa e l'eventuale rialzo del lift. La guida del telaio resta disponibile.

Terminata la pausa, se il lift è sotto la base, prima lo porta a 0,18 rotazioni sopra la posizione bassa; se è già alla base o più alto, mantiene l'altezza selezionata mentre muove il braccio. Una regolazione di altezza già in corso riprende dopo la pausa. Questa è l'altezza base dopo la presa; ogni pressione di B seleziona il livello superiore a passi di 1,38 rotazioni. I livelli sono limitati dal finecorsa superiore. Dopo il rilascio R2 e il ritorno a zero, il ciclo successivo riparte da 150°. Se R1 e R2 arrivano nello stesso ciclo, R1 ha priorità.

Tutte le quote partono dallo stesso fondo calibrato: **finecorsa alto misurato meno 3,3 rotazioni (`liftLowerRotations`)**, come configurato in `include/robot/mechanism_config.hpp`. Lo zero numerico grezzo degli encoder non è il fondo: anche coordinate negative sono valide. R1 non cerca il finecorsa del lift e non riscrive i riferimenti misurati nel preMatch.

| Posizione | Rotazioni sopra il fondo calibrato |
| --- | --- |
| Extra basso con GIÙ dal fondo, solo al secondo finecorsa del braccio | −0,68 |
| Fondo | 0 |
| Base dopo R1 (altezza 0 della presa) | 0,18 |
| Primo B dal fondo / dalla base dopo R1 | 1,38 / 1,56 |
| Secondo B dal fondo / dalla base dopo R1 | 2,76 / 2,94 |
| Passaggio R2 prima del finecorsa davanti | 0,30 (3 rotazioni sotto il finecorsa alto) |
| Ultimo livello | Contatto fisico superiore, circa `liftLowerRotations` |

Solo una pressione di B che raggiunge il limite alto comanda la salita fino al contatto a sforzo. Dal fondo il primo B richiede subito 1,38 rotazioni: la quota di presa a 0,18 è separata dagli step. Pressioni rapide sommano gli step alla quota già richiesta. Le regolazioni con GIÙ usano lo stesso passo, limitato dal fondo; dopo un arresto ripartono dalla posizione effettiva. Al completamento di un movimento il lift torna in BRAKE: R1 da solo non avvia altre salite.

Dal fondo, una nuova pressione di GIÙ seleziona il livello extra di **0,68 rotazioni sotto il limite basso normale** (`liftEndstopExtraLowerRotations`), solo con il braccio **arrivato al secondo finecorsa**. Il preMatch conserva il fondo normale a 3,3 rotazioni sotto il finecorsa alto del lift: la quota extra non modifica i riferimenti. Il Rotation deve confermare che il braccio è entro 4° dal secondo finecorsa, anche durante la discesa extra. In altre posizioni il limite resta il fondo normale.

R1 annulla una discesa extra ancora in corso e, se il lift è sotto la base di presa, lo rialza prima di ruotare il braccio. R2 al secondo finecorsa apre subito la pinza anche sotto la baseline; prima di ruotare il braccio rialza il lift alla base di presa, poi completa il ritorno tramite 144°, quota 3 dal finecorsa alto, finecorsa davanti e fondo a 3,3.

La ricerca dell'ultimo livello usa 80/127, con almeno 1.200 mA su entrambi i motori e velocità entro 5 rpm per 150 ms. Un contatto oltre 0,05 rotazioni prima del finecorsa calibrato è trattato come ostacolo. Durante la salita, una corrente massima su uno dei motori di almeno **2.200 mA per 150 ms** interrompe il movimento; la conferma del contatto finale ha priorità. Ogni richiesta di altezza ha un timeout di 7 secondi. Dopo un arresto il calo della corrente non riavvia il lift: serve un nuovo comando. Il contatto finale non modifica la calibrazione.

## Rilascio veloce con R2

Se il braccio è già al secondo finecorsa, R2 apre subito la pinza lasciando il braccio in COAST. Salta interamente il movimento a 90° e attende sia la pausa di 250 ms sia lo spostamento di mezza rotazione della ruota di odometria avanti/indietro prima del ritorno tramite 144°, discesa lift e zero. Se il lift è sotto il fondo normale, dopo questa attesa e prima del movimento a 144° risale alla base di presa con il braccio ancora in COAST. Se R1 ha selezionato il finecorsa ma il braccio non lo ha ancora raggiunto, R2 usa ancora il movimento rapido a 90°. La posizione viene confrontata con il finecorsa misurato nel preMatch.

R2 usa `armReleasePower = 80/127` costante verso `armReleaseDeg = 90`, senza rampa e senza PID di posizione. Si ferma quando entra entro `armReleaseToleranceDeg = 5°` dal target oppure lo attraversa tra due letture. Attiva subito HOLD, mantiene lo stato della pinza per **200 ms** (`releaseClampPauseMs`) e poi la apre. La pausa lascia disponibile la guida del telaio; R1 può interromperla. Non inverte il motore per correggere gli ultimi gradi: questa fase privilegia la velocità.

Dopo almeno 250 ms con pinza aperta, il ritorno parte soltanto se la ruota di odometria avanti/indietro è distante almeno **180° (0,5 rotazioni) dalla lettura alla pressione accettata di R2** (`releaseReturnPodDegrees`). Vale in entrambi i versi; i piccoli movimenti avanti e indietro non vengono sommati. Prima di questa soglia pinza e lift restano fermi, con il braccio in HOLD a 90° oppure in COAST al secondo finecorsa; la guida del telaio resta disponibile e l'attesa non ha timeout. Il display indica **R2: spostati 0.5 giri**. Se la lettura della ruota manca, il ritorno attende; al recupero prende un nuovo riferimento e richiede mezza rotazione da lì.

Quando entrambe le condizioni sono soddisfatte, il braccio torna verso 144° con il profilo rapido. In questa posizione di passaggio accetta **139°–149°** (`armReturnClearanceToleranceDeg = 5°`), senza correggere fino alla precisione ordinaria di 2°. Dopo l'assestamento a velocità bassa per 40 ms, tiene fermo il braccio e porta il lift a **3 rotazioni sotto il finecorsa alto** (`returnLiftRotations`), salendo o scendendo secondo la quota corrente. Solo dopo l'arrivo del lift muove il braccio al **finecorsa davanti/zero** e aggiorna lo zero. Dopo il contatto confermato abbassa il lift al **fondo a 3,3 rotazioni sotto il finecorsa alto**, lasciando il braccio in COAST. Al termine la pinza resta aperta. R2 è accettato da 150°, dal finecorsa, durante entrambi i movimenti R1, dallo stato pronto, dal manuale e da un arresto riprendibile. Se manca l'altezza del lift, la pressione viene memorizzata fino al termine della salita insieme al riferimento della ruota. Pressioni R2 ripetute durante il rilascio non riavviano la sequenza; R1 la può interrompere in qualsiasi fase.

## Recupero durante la guida

Gli arresti in driver **non cancellano più la calibrazione e non impongono un nuovo preMatch**:

- Una lettura non disponibile ferma subito i comandi e mantiene il braccio in HOLD. Se torna valida prima di 200 ms, la richiesta riprende automaticamente con i controlli di movimento e assestamento reinizializzati.
- Se la mancanza dura almeno 200 ms, resta fermo finché arriva un nuovo comando R1/R2 o una nuova selezione di altezza B/GIÙ. La ricomparsa dei dati da sola non riavvia una sequenza interrotta a lungo.
- Una differenza superiore a 0,25 rotazioni fra encoder lift validi è un avviso, non il vecchio errore generico. Una lettura mancante di uno dei due motori resta distinta e non viene nascosta dalla media.
- Un blocco persistente, un movimento opposto o un timeout fermano lo sforzo. Pinza e riferimenti restano conservati. R1/R2 possono subito selezionare una nuova richiesta e B/GIÙ restano disponibili per regolare il lift.
- Se si ferma il braccio per le correzioni fuori tolleranza, B/GIÙ restano attivi per regolare il lift; R1/R2 possono riprovare il braccio.
- Il mancato avanzamento durante il profilo in driver ha una finestra di 1.500 ms e richiede anche corrente e velocità coerenti con un blocco. La mancanza di arrivo ha comunque un limite di 7 secondi. Il movimento R2 ha controlli di avanzamento e durata.

Gli avvisi distinguono Rotation braccio, motore braccio, primo o secondo motore lift. Il controller aggiorna una riga ogni 100 ms: stato in alto, dettaglio sulle due righe successive. Al recupero i dettagli precedenti vengono cancellati. Sul Brain resta visibile lo stato della macro. La perdita del controller e la disabilitazione fermano i comandi.

## PreMatch e READY

All'avvio parte automaticamente la sequenza di preparazione, una sola volta per esecuzione. Dopo un secondo avvia i movimenti se i motori sono abilitati. In **disabled**, VEXos impedisce fisicamente i comandi ai motori: il display mostra **PREMATCH IN ATTESA** e la ricerca dei finecorsa non parte, evitando timeout o falsi riferimenti. Abilitando il robot o scollegando il field controller, il prematch in attesa parte automaticamente. Se viene disabilitato durante la ricerca, interrompe i comandi e ricomincia il solo avvio incompleto quando i motori tornano disponibili. Un prematch completato non si ripete ai cambi di modalita'. Questo limite e' descritto dal tecnico VEX nella [spiegazione del controllo gara](https://www.vexforum.com/t/competition-code-and-user-created-tasks/88616).

1. Avvia subito il reset del lift: cerca due volte il finecorsa alto, con distacco di 0,15 rotazioni e verifica di ripetibilità.
2. Tiene fermo il braccio in HOLD per **1 secondo** (`preMatchArmDelayMs`), poi ne avvia il reset mentre quello del lift prosegue in parallelo.
3. Cerca due volte il finecorsa zero del braccio, con distacco di 12° a potenza 25/127. Il primo distacco apprende il verso dell'encoder.
4. Cerca due volte il secondo finecorsa del braccio, con lo stesso distacco. Verifica ripetibilità e corsa sufficiente per 144°.
5. Porta il braccio a 144° con il profilo rapido guidato dal Rotation; per questa sola posizione di passaggio usa da subito una tolleranza di 5°. Attende anche il completamento del reset del lift prima della discesa. Ogni reset usa tempi di contatto e timeout indipendenti; un errore ferma entrambi i meccanismi.
6. Abbassa il lift di `liftLowerRotations` rotazioni dal finecorsa alto misurato, anche quando il fondo ha una coordinata encoder negativa.
7. Cerca nuovamente lo zero del braccio a sforzo e lascia la pinza aperta.

Solo dopo compare **READY**. Premendolo parte un'altra attesa di un secondo, poi inizializzazione del giroscopio, odometria e servizi di navigazione. Driver e autonomous attendono il completamento dell'avvio e restano soggetti all'abilitazione di gara. Un errore o R2 durante il preMatch arrestano la sequenza; il pulsante touch **RIPROVA** permette di ripetere l'avvio dopo l'errore. SU non riavvia il preMatch durante il driver. Queste verifiche iniziali stabiliscono zero, verso e corsa reali.

La ricerca iniziale dei finecorsa e il ritorno allo zero del braccio usano 127/127. Il rilevamento del contatto richiede corrente almeno 400 mA, velocità motore entro 5 rpm e Rotation fermo entro 0,4°. Nel preMatch restano **350 ms di grazia iniziale e 400 ms di conferma continua**. Nelle macro già calibrate, sia al finecorsa alto sia nel ritorno a zero, i tempi sono **100 ms di grazia e 100 ms di conferma** (`armDriverHomeGraceMs`, `armDriverHomeStillMs`). Partendo già a contatto, il minimo passa da 750 a 200 ms; quando la corsa assorbe la grazia, la sola conferma scende da 400 a 100 ms. Il lift usa 80/127 e richiede lo sforzo di entrambi i motori. Il secondo finecorsa selezionato con R1 è già noto: usa il profilo rapido durante l'avvicinamento e 40/127 negli ultimi 4° per completare il contatto, poi COAST. Il contatto deve essere vicino alla posizione misurata: un ostacolo incontrato prima produce un arresto riprendibile.

I riferimenti restano in RAM durante il rilascio R2, dopo un arresto, attraverso disabilitazione e rientro in driver. Arresto e disabilitazione annullano anche la richiesta di altezza, senza ripresa automatica. Un nuovo preMatch invalida i riferimenti fino al suo completamento; lo spegnimento li perde. La pinza è aperta all'avvio e al termine del preMatch. L1+R1 apre la calibrazione del telaio nelle prove senza field controller, come prima.

## Parametri e verifiche

Il PID esterno del braccio è stato rimosso. `ArmMotion` applica un profilo di potenza a distanza, usando il Rotation per sapere quando rallentare e fermarsi:

- Picco **127/127**, accelerazione limitata a **1.600 unità/s**: circa 80 ms per salire a piena potenza.
- Rallentamento negli ultimi **24° prima della tolleranza di 2°** (prima 12°), con curva a radice della distanza residua e riduzione della potenza fino a 5.000 unità/s.
- Velocità stimata dalle posizioni, filtrata a 20 ms. Anticipo iniziale di frenata **70 ms** (prima 25 ms), con margine di 0,5°. Il movimento veloce termina con BRAKE passivo; HOLD viene inserito a velocità bassa, evitando di chiedere al mantenimento di posizione di arrestare tutta l'inerzia. Dopo al massimo 200 ms viene comunque ripristinato HOLD, anche se il carico impedisce l'arresto passivo.
- Fuori tolleranza, dopo almeno 60 ms in HOLD e a velocità bassa, sono consentite **al massimo due correzioni**, con potenze iniziali **30/127** e poi **20/127**. Il loro anticipo viene stimato dallo spazio effettivamente percorso nella frenata precedente. Nessun termine proporzionale, integrale o derivativo. Se restano fuori target, le macro si fermano mantenendo il carico: R1/R2 possono subito dare una nuova richiesta, senza nuovo preMatch. Non viene dichiarato un arrivo fuori tolleranza.
- **Assistenza sotto carico:** durante un avvicinamento motorizzato fuori tolleranza, se la velocità nel verso richiesto resta sotto **20°/s** per almeno **60 ms**, dopo il completamento della rampa al comando richiesto, aumenta la potenza di **400 unità/s**, fino a **127/127**. Il minimo raggiunto viene conservato per quel verso e quella richiesta, anche nelle correzioni successive; un nuovo target azzera l'apprendimento. L'assistenza non accumula errore di posizione e non interviene durante BRAKE/HOLD: il criterio di frenata ha precedenza. Evita che il peso lasci fermo il braccio quando 20–30/127 non bastano, senza alzare sistematicamente la potenza delle correzioni a vuoto.
- Arrivo confermato entro **2°** e **5°/s** per **40 ms**. Il motore mantiene poi la posizione con HOLD, anche durante la salita/discesa del lift. Questo controllo interno del motore è ancora attivo: la rimozione del PID software non elimina il mantenimento interno.

Lo stesso profilo serve per **150°**, **144° di passaggio** e l'avvicinamento al **secondo finecorsa già calibrato**. La tolleranza è 2°, salvo i 5° della posizione di passaggio nel preMatch e nel ritorno dopo R2. La ricerca iniziale dei finecorsa resta separata perché non ne conosce ancora la posizione. R2 conserva la sua corsa rapida a potenza costante.

Il lift usa guadagno salita 200, minimo salita 70/127, guadagno discesa 220 e limite 127/127. Entrambi i motori del lift sono in modalità **BRAKE**: a comando zero frenano passivamente, senza mantenimento attivo HOLD. Prima del movimento del braccio, l'altezza minima di presa è 0,18 rotazioni sopra la posizione bassa.

La simulazione dei meccanismi usa lo stesso ciclo driver del firmware, compreso il controllo delle altezze. Le regressioni del lift coprono diversi zeri grezzi degli encoder, R1 senza salite aggiuntive, inversioni del braccio a lift alto, passi di 1,38 anche dal fondo e con pressioni rapide, tratti residui solo ai limiti, riferimenti invariati dopo il contatto, R2 verso quota 3 da entrambi i lati prima del finecorsa davanti, fondo finale a 3,3 e arresti per sovraccarico, ostacolo e timeout.

I test host coprono pressioni R1 singole, doppie, triple e quadruple, inversioni durante la corsa, encoder invertito, interruzione del rilascio e del ritorno, carico/attrito, HOLD, letture mancanti brevi e persistenti, ripartenza senza preMatch e ostacoli prima del finecorsa. Coprono anche R1/R2 durante BRAKE e l'arresto riprendibile dopo overshoot ripetuti. Nel plant delle macro, la corsa di 40° richiede 680 ms; R2 richiede 360 ms. Il test separato simula carico, attrito, rumore e intervalli variabili: 0→150° richiede 1.170 ms nel caso nominale; nella matrice di 72 combinazioni, le quattro corse per caso completano entro 1.995 ms e circa 2° all'arrivo. Un ulteriore modello di HOLD sottosmorzato verifica l'arrivo anche con rimbalzo. Le regressioni sotto carico aggiungono sforzi equivalenti di ±35 e ±55 unità con attrito 10 (otto corse, massimo 2.055 ms), rimozione del peso durante la corsa, saturazione a 127 su un ostacolo e reset dell'assistenza al cambio target. La sequenza R1/R2 completa viene verificata anche con attrito 55, superiore ai vecchi comandi finali. Queste unità sintetiche non corrispondono a chilogrammi. Questi non sono tempi né precisioni misurati sul robot. Verificare soprattutto anticipo di frenata, inerzia e mantenimento con il carico reale; i test non garantiscono l'assenza di oscillazioni del controllo interno o della meccanica.
