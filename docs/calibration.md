# Calibrazione ripetibile, stagione dopo stagione

La calibrazione modifica i parametri fisici usati da stimatore e controllo. Il robot offre una procedura guidata per offset dei pod e carreggiata effettiva; il programma PC conserva le misure con riferimento esterno e l'identificazione della dinamica. Da ODOM 7 usa soltanto la calibrazione nativa dell'IMU all'avvio: nessuna misura o sottrazione aggiuntiva del bias. La libreria conserva opzionalmente la modalità compensata per altre configurazioni. Non è un autotune PID. Il nucleo matematico elabora dati senza comandare motori; la procedura PROS separata gestisce i movimenti controllati e gli arresti.

Le dimensioni e i coefficienti presenti nel template sono valori iniziali, **non misure del vostro robot**. Il profilo nasce con `calibratedMask = 0`. Ripetere le prove dopo modifiche a ruote, rapporto di trasmissione, distribuzione dei pesi, pod, montaggi o massa del robot. Conservare i CSV insieme alla configurazione meccanica e alla data della prova.

Le porte e i parametri fisici si modificano in `include/robot-config.hpp`; i movimenti di prova e i tasti in `src/main.cpp`. Non serve modificare le funzioni interne che costruiscono la configurazione della libreria. Per la prima prova breve, partire dall'esempio nel [README](../README.md).

## Convenzioni

CSV, risultati del programma `nexus_calibrate` e configurazione del robot usano **millimetri** per le lunghezze, **mm/s** per le velocità e **mm/s²** per le accelerazioni. I soli diametri delle ruote rimangono in pollici. Gli autonomous usano millimetri e gradi; gli angoli dei CSV di calibrazione restano in radianti, i tempi in secondi e le tensioni in volt.

Il programma converte automaticamente i millimetri nelle unità SI interne del modulo matematico `nexus::calibration` (metri e radianti) e riconverte in millimetri i risultati visualizzati. Non convertire manualmente i CSV in metri. I vecchi CSV vengono rifiutati: per migrarli, moltiplicare tutte le distanze, velocità e accelerazioni per 1000, aggiornare le intestazioni e aggiungere la dichiarazione di unità prevista per i CSV della dinamica. Il profilo binario conserva le unità SI interne e non richiede questa conversione.

- Asse X a destra, Y in avanti; angolo positivo in senso orario partendo da Y.
- Spostamenti, velocità e tensioni positivi fanno avanzare il relativo lato del drivetrain.
- `forwardOffset` è la posizione X del pod che misura l'avanzamento.
- `lateralOffset` è la posizione Y del pod che misura lo spostamento laterale.
- In una rotazione pura: `forward = -forwardOffset * angle`, `lateral = lateralOffset * angle`, `left - right = trackWidth * angle`.

Controllare prima i segni: spostare a mano il robot in avanti, a destra e in senso orario deve produrre i segni attesi. Una calibrazione non corregge un cablaggio o un segno sbagliato.

## Prova offset con due riferimenti (tasto Y)

**Esito sul robot, 4 ottobre 2026:** la procedura non e' ancora ripetibile e i suoi risultati non vanno usati per sostituire le misure fisiche. L'utente ha misurato `forwardPodX = +20 mm` e `lateralPodY = -70 mm` rispetto al centro geometrico; questi sono ora i riferimenti in `robot-config.hpp`. Le cinque stime ottenute con offset iniziali +20/-33 erano (74,-23), (78,-63), (99,-46), (54,-46), (57,-8) mm; media +72,4/-37,2 mm. Con offset iniziali +13,67/-78,17 erano (76,-29), (92,-10), (70,-39), (79,2), (63,42) mm; media +76/-6,8 mm. Sono due gruppi distinti. Il fit non usa gli offset iniziali, ma soltanto letture grezze, scale e riferimenti. La discrepanza dalle misure fisiche e la dispersione non identificano da sole la causa: mancano i dati per sosta e la relativa concordanza. I test sintetici verificano il calcolo sotto le assunzioni del modello, non l'accuratezza fisica della procedura. Per le prossime verifiche usare le misure fisiche, senza applicare le stime automatiche con X.

Questa e' la prova per la configurazione attuale, con **tutti gli encoder motore esclusi**. Usa soltanto i due pod e l'angolo continuo dell'IMU nativa. I motori sinistri e destri ricevono tensioni opposte; obiettivo 90 gradi/s nelle due rotazioni di misura, 45 gradi/s nel ritorno, rampa di 8 V/s e limite di 4 V. La tensione aumenta gradualmente per vincere l'attrito e segue la velocita' misurata su finestre di almeno 40 ms. Il ritorno anticipa l'arresto usando la rotazione residua osservata nelle soste precedenti. Le velocita' sono obiettivi del controllo, da verificare sul robot.

### Procedura passo passo

1. Carica e avvia il firmware, lasciando il robot fermo durante la calibrazione IMU. Completa il normale READY/prematch. Usa DRIVER ENABLED, senza field controller collegato.
2. Individua con un righello il **centro geometrico della trazione**, il punto rispetto al quale vuoi misurare gli offset. La distribuzione dei pesi non serve. Rendilo riconoscibile sul telaio e prepara una croce sul pavimento esattamente sotto quel punto; un piccolo riferimento verticale solidale al telaio aiuta ad allinearlo senza parallasse. Segna anche l'orientamento iniziale. I pod devono restare a terra durante tutta la prova.
3. Tieni **L1+R1 per un secondo**, rilasciali e premi **Y**. Compare `OFFSET POD`. L'orientamento corrente diventa lo zero relativo della prova; non occorre azzerare la posa sul display principale.
4. Tieni **R1**: il robot gira in senso orario fino a circa **+90 gradi**, poi si arresta. Aspetta `FERMO: rilascia R1`, quindi rilascialo. A motori fermi, **trasla a mano il robot fino a riportare lo stesso punto del telaio sulla stessa croce**, conservando l'orientamento di questa sosta. Non sollevarlo. Quando e' fermo sul segno premi **A**; compare `Riferimento registrato`.
5. Tieni nuovamente **R1**: gira in senso antiorario fino a circa **-90 gradi rispetto alla partenza**, quindi circa mezzo giro dalla prima sosta. Aspetta di nuovo `FERMO`, rilascia R1, ricentra lo stesso punto sulla croce mantenendo questo secondo orientamento e premi **A**.
6. Tieni ancora **R1** per il ritorno automatico vicino all'orientamento iniziale. Alla schermata `RISULTATI` rilascialo. Leggi **Avanti X**, **Laterale Y** e **Differenza prove F/L**, tutti in mm. L'heading finale effettivo viene mostrato: il ritorno ha tolleranza di 2 gradi e al massimo quattro ulteriori correzioni, quindi non promette uno zero esatto.
7. Se compare `Stima concorde`, **X** applica gli offset solo per questa accensione, senza microSD. **B** esce, **A** torna al menu. Prima di un autonomous riposiziona il robot sui riferimenti previsti per quella routine; la calibrazione non azzera X/Y. Per conservare i valori dopo il riavvio, riporta Avanti X in `forwardPodX` e Laterale Y in `lateralPodY` dentro `robot-config.hpp`, dopo una verifica indipendente. Un eventuale profilo sulla microSD puo' prevalere sui valori del codice.

Non occorre ottenere esattamente +90 o -90: il calcolo usa l'angolo realmente misurato. Deve invece essere accurato il ricentraggio dello **stesso punto**. Scegliere un altro punto del telaio cambia l'origine degli offset; per usarli nel modello del controller deve essere il suo centro geometrico di riferimento.

I precedenti blocchi basati su encoder motore, quiete, velocita' minima/massima e traslazione non sono usati da questa prova. Restano R1 durante le rotazioni, B per annullare, telecomando e sensori validi, scadenza dei comandi a 100 ms e limite di **20 secondi per ogni tratto motorizzato**, comprese eventuali correzioni del ritorno. Il ricentraggio manuale non ha quel limite di durata, ma continua a registrare i sensori. Se R1 viene rilasciato prima della schermata `FERMO`, la prova si annulla e si riparte dal menu.

### Perche' non dieci giri e un solo ricentraggio finale

Dividere la distanza totale di un pod per l'angolo misura il suo offset solo assumendo una rotazione pura attorno al punto scelto. Tornare alla stessa posizione dopo dieci giri non verifica questa assunzione: una traslazione espressa negli assi del robot puo' accumularsi nei pod pur chiudendo esattamente il percorso sul pavimento.

Questa prova integra tutti gli incrementi, **compreso il ricentraggio manuale**, senza usare gli offset correnti. Se `u` e' la posizione ottenuta dai pod senza compensazione degli offset e `theta` l'angolo finale relativo, il ritorno dello stesso punto alla croce fornisce:

```text
0 = u + [ 1-cos(theta)   -sin(theta) ] [ forwardOffset ]
        [ sin(theta)    1-cos(theta) ] [ lateralOffset ]
```

A +90 e -90 gradi il sistema determina entrambi gli offset; a un numero intero di giri la matrice e' zero e non li determina. Le due soste producono due stime e una media pesata. Una differenza superiore a **5 mm** per uno dei due offset viene segnalata; i risultati restano visibili ma il tasto di applicazione non viene abilitato. Questo confronto misura la concordanza, non garantisce 5 mm di precisione. Le scale dei pod e dell'IMU restano quelle configurate: slittamenti, flessione, errori delle scale o dei riferimenti possono ancora falsare la misura. Verifica su un percorso diverso dopo l'applicazione.

Implementazione: `include/nexus/pod_offset_calibration.hpp` e `src/robot-calibration.cpp`. La console USB stampa anche una riga `PODOFFSET` con entrambe le stime, gli angoli e la concordanza. Il test host `pod_offset` usa una cinematica indipendente con traslazione durante le rotazioni, ricentraggio manuale, attriti diversi, dati a scatti, sensori mancanti e timeout. La build verifica il software; questa nuova procedura deve ancora essere validata fisicamente sul robot.

## Calibrazione geometrica con encoder motore (tasto X)

Questa procedura precedente richiede gli encoder della trazione. Con `useDriveEncodersForOdometry=false` usa la nuova prova **Y** descritta sopra.

La guida normale usa sempre i tasti definiti in `main.cpp`. Per aprire la calibrazione devono essere attivi DRIVER e ENABLED, il telecomando deve essere collegato e il field controller/competition switch deve essere scollegato. Tieni **L1+R1 per un secondo** e poi rilasciali. Finché il menu è attivo, i normali comandi del drivetrain, lift, braccio, intake e pinza sono sospesi.

| Nel menu | Azione |
| --- | --- |
| A | In modalità nativa informa che l'IMU viene calibrata all'avvio; nella modalità compensata misura il bias per cinque secondi |
| X | Prepara la calibrazione di offset pod e carreggiata |
| Y | Nuova prova offset con soli pod e IMU, descritta sopra |
| B | Annulla e torna alla guida |

Per la geometria, dopo X tieni **R1 premuto per tutta la prova**. In modalità nativa viene saltata la misura aggiuntiva del bias; resta l'attesa iniziale della routine di rotazione. La modalità compensata richiede anche cinque secondi iniziali da fermo. Il robot esegue poi circa 360° in un verso e 360° nel verso opposto, a tensione ridotta. R1 rilasciato, B, perdita del telecomando, cambio modalità o dati sensori mancanti/non recenti fermano e annullano la procedura. Il comando motori scade comunque dopo 100 ms senza rinnovo. La prova ha anche limiti di durata, rotazione e traslazione: il comando non supera 6 V e la procedura si arresta se manca progresso nella rotazione.

Usa il pavimento di gara, ruote appoggiate e spazio sufficiente all'intera sagoma del robot. La tensione viene regolata usando la velocità misurata dal gyro, con obiettivo 120°/s. Il comando può salire fino a 6 V per vincere gli attriti iniziali e diminuisce quando il robot prende velocità; la salita è limitata a 20 V/s e anche il telaio limita i comandi a 6 V. La velocità angolare massima consentita è 360°/s e la durata massima della fase di rotazione è 45 s, oltre ai cinque secondi iniziali da fermo. Se a questa tensione il robot non gira, la prova non è valida: verifica attriti, carico e segni prima di modificare i limiti.

Al termine i motori sono fermi e sul Brain compaiono **offset avanti, offset laterale e carreggiata in mm**. I risultati non sono ancora applicati:

| Risultato proposto | Azione |
| --- | --- |
| A | Applica il profilo e tenta il salvataggio in `/usd/nexus.cfg` |
| X | Applica soltanto in RAM, fino al riavvio |
| B | Scarta il risultato |

Un errore microSD viene indicato esplicitamente: un risultato applicato soltanto in RAM non è un salvataggio riuscito. Il profilo precedente resta recuperabile; non rimuovere la scheda durante la scrittura. L'applicazione della geometria conserva la posa e la sua incertezza: non costituisce una misura della posizione assoluta. Prima dell'autonomous dichiara la posa reale con `auton.setPose(...)`. Dalla schermata di esito, A torna al menu e B torna alla guida. Scartare la geometria non annulla il bias da fermo già accettato per questa accensione.

### Che cosa viene misurato

- **Bias IMU:** pendenza dell'angolo continuo mentre encoder e pod confermano che il robot è fermo. Il confronto fra prima e seconda parte della misura rifiuta una deriva non stabile. Si misura anche il rumore residuo, ma non si abbassa automaticamente il rumore operativo dello stimatore: vibrazioni e movimento sono condizioni diverse. Il bias vale per questa accensione e non viene scritto nel profilo.
- **Geometria:** rotazioni aggregate in finestre di almeno 0,20 rad, con almeno π rad di dati accettati per ogni verso. Il fit robusto rifiuta outlier e confronta stime indipendenti orarie e antiorarie. Usa gyro e due lati del drivetrain; un pod configurato deve rimanere valido. Un pod disattivato con porta 0 viene omesso.
- **Controlli sui dati:** timestamp strettamente crescenti, intervalli non superiori a 100 ms, valori finiti, salti plausibili e memoria fissa. Una piccola deriva avanti misurata dai motori viene compensata nel fit dell'offset; archi marcati, incoerenza fra versi o slittamenti variabili fanno fallire la prova.

Le scale già configurate vengono usate per convertire le letture. **Questa prova non identifica le scale assolute dei pod o del gyro**, né può riconoscere ogni slittamento proporzionale e sistematico. Gli offset stimati sono validi rispetto alle scale usate. Controlla diametri e rapporti, quindi confronta una distanza e una rotazione con riferimenti esterni. Se cambi una scala in seguito, ripeti la geometria. `kS`, `kV` e `kA`, latenze ed estrinseci dei distance richiedono le prove successive.

All'avvio ODOM 7 esegue `imu->reset(true)` con il robot fermo e attende la calibrazione nativa. La precedente misura aggiuntiva di tre secondi è stata rimossa. Con `nativeImuHeading=true`, A non misura né applica un bias; anche la geometria usa bias locale zero. Il rilevamento della quiete non sostituisce l'heading restituito dall'IMU. Scale e offset geometrici restano configurabili e il profilo non memorizza bias di avvio.

Il campionamento e la gestione del menu sono affidati a un task persistente separato; fit e scrittura microSD non girano nei task di stima o NMPC. Le classi `StationaryCalibration` e `RotationCalibration` usano memoria fissa; le API PC descritte sotto sono invece offline.

## Compilare il programma di calibrazione sul PC

Il target CMake `nexus_calibrate` è un programma host, separato dal firmware PROS:

```powershell
cmake -S tests -B build/host
cmake --build build/host --config Release --target nexus_calibrate
```

Con Visual Studio il programma è `build/host/Release/nexus_calibrate.exe`; con generatori a configurazione singola è generalmente `build/host/nexus_calibrate`.

L'argomento `hardware_id` deve essere uguale al valore stampato dal robot nella console seriale alla voce **`NEXUS hardware fingerprint`**. Questo hash combina `robotRevision` e la configurazione hardware: non passare direttamente `robotRevision`. Accetta un intero decimale o esadecimale. Negli esempi, `0x5341575032303236` è un segnaposto da sostituire con il valore reale. Incrementare `robotRevision` quando una modifica meccanica non già rappresentata nella configurazione invalida le misure. Un file con un identificatore diverso viene rifiutato.

## 1. Scala dei pod con un riferimento fisico

Per ogni asse, misurare almeno sei spostamenti indipendenti di almeno 200 mm, con almeno due prove positive e due negative. È preferibile usare varie distanze, 10–20 prove e una misura esterna precisa della traslazione reale. Durante le prove non ruotare il robot. La distanza misurata dal pod deve essere quella **prima** di applicare la scala del profilo: applicarla due volte produce una calibrazione errata.

Formato `forward.csv` oppure `lateral.csv`:

```csv
measured_mm,reference_mm
1000,1024
700,717
-1000,-1024
```

L'esempio mostra il formato, non un insieme sufficiente per calibrare. `measured_mm` è la distanza grezza convertita in millimetri con diametro/rapporto nominali; `reference_mm` è la distanza realmente percorsa in millimetri, con lo stesso segno.

```powershell
build/host/Release/nexus_calibrate.exe fit-travel forward forward.csv nexus.cfg 0x5341575032303236
build/host/Release/nexus_calibrate.exe fit-travel lateral lateral.csv nexus.cfg 0x5341575032303236 nexus.cfg
```

L'ultimo argomento facoltativo è il profilo di base: consente di conservare tutte le calibrazioni già eseguite. Può coincidere con il file di uscita. Senza profilo di base, il file contiene solo le categorie appena nexus_calibrate. Cambiare la scala di un pod invalida il suo offset precedentemente stimato: il programma ne rimuove il bit di calibrazione e la prova di rotazione va ripetuta.

## 2. Rotazioni: gyro, offset e larghezza effettiva

Raccogliere almeno sei rotazioni pure del centro di riferimento del robot, almeno due per verso, con angoli di almeno 0,30 rad. Alternare ampiezze e versi. Usare un riferimento angolare **esterno**, per esempio orientamento misurato da una dima o da un sistema ottico. Non usare il gyro stesso come verità: così la sua scala non sarebbe identificabile. Per angoli oltre un giro, conservare l'angolo totale senza ridurlo a 0–360°.

```csv
reference_angle_rad,gyro_angle_rad,forward_travel_mm,lateral_travel_mm,left_travel_mm,right_travel_mm
1.570796327,1.552170284,-45.553094,84.823002,237.190245,-237.190245
-1.570796327,-1.552170284,45.553094,-84.823002,-237.190245,237.190245
```

Ogni riga contiene gli incrementi durante una sola prova. Le letture dei pod sono grezze; il programma applica le scale del profilo di base. Tutte le distanze devono essere già convertite in millimetri con diametro e rapporto di trasmissione verificati; gli angoli sono in radianti. Se una categoria di sensori manca, scrivere `nan` nei relativi campi: le altre categorie rimangono utilizzabili.

```powershell
build/host/Release/nexus_calibrate.exe fit-rotation rotation.csv nexus.cfg 0x5341575032303236 nexus.cfg
```

Il programma calibra gli offset dei pod solo se esiste già una scala accettata per il relativo pod. Una rotazione da sola non separa scala del pod e offset. La larghezza ricavata dai motori è **effettiva** e dipende da ruote, carico e superficie: ripetere a velocità rappresentative, evitare grandi slittamenti e verificare che le prove siano ripetibili.

Quando entrambi gli encoder motore sono disponibili e correttamente convertiti, il fit offline compensa una piccola deriva longitudinale del centro usando la media dei loro spostamenti. Esclude dalla geometria dei pod i campioni con `abs(left + right) > max(2 mm, 0.06 * abs(left - right))`; se gli outlier superano il 25% o mancano abbastanza rotazioni pure, gli offset non vengono accettati. La scala gyro e la carreggiata vengono valutate separatamente. Senza dati motore resta necessaria una rotazione pura verificata esternamente. Una traslazione laterale non osservata o uno slittamento sistematico proporzionale possono ancora confondersi con un offset diverso: il controllo software non sostituisce il riferimento fisico.

Se cambia la scala del gyro, il programma rimuove i bit degli offset e della carreggiata precedenti, che potrebbero derivare da una calibrazione automatica relativa alla vecchia scala. Conserva come calibrati solo i parametri geometrici rimisurati e accettati nella stessa prova. Con un CSV che contiene soltanto il gyro, ripetere quindi la calibrazione della geometria.

## 3. Dinamica separata di lato sinistro e destro

Il modello stimato per ciascun lato è:

```text
voltage = kS * sign(velocity) + kV * velocity + kA * acceleration
```

Nel report e nella configurazione pubblica, `kS` è in V, `kV` in V/(mm/s), `kA` in V/(mm/s²). Raccogliere accelerazioni, tratti a velocità diverse e decelerazioni in entrambi i sensi, sul pavimento usato dal robot. Usare la stessa configurazione meccanica e massa delle partite. Prima partire con tensioni moderate e verificare manualmente spazio disponibile, arresto e segni. I dati devono essere rilevati durante prove deliberate; il comando `fit-wheel` non comanda il robot.

Per ogni lato, preparare un CSV:

```csv
# units: voltage=V,velocity=mm/s,acceleration=mm/s^2
voltage,velocity,acceleration
4.21,500,800
4.40,560,580
-3.91,-510,-380
```

La prima riga di dichiarazione delle unità è obbligatoria e va riportata esattamente come nell'esempio, prima dell'intestazione. `voltage` è in V, `velocity` in mm/s, `acceleration` in mm/s². `tools/prepare_dynamics.py` genera questo formato dai log del robot: legge `left_motor_mmps` e `right_motor_mmps`, usa i tempi in secondi e calcola accelerazioni in mm/s². I log precedenti con campi `*_motor_mps` sono rifiutati.

Servono almeno 30 campioni validi per lato; molti segmenti a velocità diverse sono preferibili a tante righe quasi identiche. Servono almeno cinque campioni per verso, un intervallo di velocità di almeno 150 mm/s e accelerazione RMS di almeno 150 mm/s². Questi limiti minimi non garantiscono da soli una buona prova: la verifica di condizionamento controlla anche che velocità e accelerazione non siano indistinguibili nel log.

I limiti di numerosità e di eccitazione devono essere rispettati anche dopo aver escluso gli outlier. Nei fit offline di traslazione e rotazione servono almeno sei inlier, di cui almeno due per verso; un verso composto soltanto da misure rifiutate non convalida il risultato.

Usare la tensione **realmente applicata**, rilevata dai motori e convertita da mV a V. Il comando richiesto può differire per saturazione, batteria e limiti interni. Mediare i motori del lato solo dopo aver normalizzato i segni. La velocità deve rappresentare la velocità a terra della ruota; durante pattinamenti l'encoder del motore non è una misura valida della velocità del robot.

Velocità, tensione e accelerazione devono riferirsi allo stesso istante. Per ottenere accelerazione da encoder rumorosi, usare una derivata da un fit polinomiale locale o un filtro offline senza sfasamento; evitare la semplice differenza tra letture rumorose non allineate. Eliminare transitori di sensori, collisioni, reset e campioni non sincronizzati. La soglia di velocità esclude la zona di stizione, nella quale il modello lineare non rappresenta il distacco statico.

```powershell
build/host/Release/nexus_calibrate.exe fit-wheel left.csv right.csv nexus.cfg 0x5341575032303236 nexus.cfg
```

Il fit usa minimi quadrati robusti iterativi con pesi Huber, colonne normalizzate, controllo degli autovalori della matrice d'informazione e limiti di plausibilità fisica. Riporta campioni, outlier, errore RMS completo, RMS degli inlier e condizionamento. I residui sono espressi in mm per le distanze, rad per il gyro e V per la dinamica; le scale sono adimensionali. Un errore eccessivo, troppi outlier, dati non osservabili o coefficienti non fisici causano il rifiuto. Una prova rifiutata non sovrascrive il file di uscita.

La regressione presume che velocità e accelerazione siano abbastanza precise: il filtro robusto riduce l'effetto degli outlier, ma non elimina il bias sistematico causato da derivate rumorose o timestamp errati. Validare sempre i coefficienti su un **secondo log** non usato nel fit, confrontando tensione prevista e misurata e poi ripetendo movimenti reali. La precisione finale del robot non si deduce dall'RMS di questo modello.

## 4. Scala e offset dei sensori Distance

Calibrare ogni sensore separatamente: la misura corretta è `distanceScale * lettura_grezza + distanceOffset`. Nel file `robot-config.hpp`, `distanceOffset` è in mm. Il riferimento fisico deve essere la distanza dalla faccia del sensore al bersaglio lungo il suo asse, non la distanza dal centro del robot. Posizione e orientamento del montaggio (`x`, `y`, `heading`) rimangono misure separate.

Usare una parete o un pannello piano, perpendicolare al fascio, senza oggetti davanti. Raccogliere almeno 12 misure indipendenti a più distanze: consigliate cinque o più posizioni, con almeno tre ripetizioni ciascuna, distribuite nell'intervallo usato in gara. Il fit richiede un intervallo di almeno 500 mm e almeno tre inlier in ciascuno dei tre terzi dell'intervallo; una sola distanza o due gruppi alle estremità non bastano. Le ripetizioni devono corrispondere a letture nuove, non copie dello stesso campione memorizzato.

Il formato di `distance-front.csv` è lo stesso del riferimento di traslazione, ma entrambe le distanze sono positive. Queste quattro righe mostrano soltanto il formato: per eseguire il fit occorre il numero di misure indicato sopra.

```csv
measured_mm,reference_mm
100,89.5
500,503.5
1000,1021
1500,1538.5
```

```powershell
build/host/Release/nexus_calibrate.exe fit-distance distance-front.csv
```

Il comando stampa scala, offset, RMS e outlier, insieme al minimo e massimo delle letture **grezze** accettate. Riporta anche i campi `.distanceScale`, `.distanceOffset`, `.minimumDistance` e `.maximumDistance` da copiare nel relativo elemento di `wallSensors`. Non modifica `nexus.cfg`: la calibrazione Distance è configurazione per sensore e non estende il profilo binario v1. Non applicare due volte la correzione ai CSV di partenza.

Il fit affine usa un'inizializzazione robusta e regressione Huber, controllando nuovamente la copertura dopo gli outlier. Rifiuta scale fuori da 0,8–1,2, offset oltre ±100 mm, oltre il 20% di outlier o RMS degli inlier oltre 25 mm. Questi sono controlli software di plausibilità e qualità del dataset, non specifiche di precisione. Per il V5 Distance il campo nominale è 20–2000 mm; VEX indica circa ±15 mm sotto 200 mm e circa il 5% oltre 200 mm. [Specifiche e utilizzo VEX](https://kb.vex.com/hc/en-us/articles/360050696511-Using-the-Distance-Sensor-with-VEX-V5).

**L'RMS del fit non è la precisione assoluta né il rumore operativo dello stimatore.** Verificare scala e offset su distanze nuove, altre superfici e angoli rappresentativi; riflessi, inclinazione e occlusioni possono introdurre errori sistematici non presenti nel CSV. Conservare i valori predefiniti di `.distanceStd` e `.distanceRelativeStd` finché questa validazione indipendente non giustifica una modifica. Il software non li riduce automaticamente. Il dominio raw riportato permette di escludere misure esterne alle distanze provate; non prova che sia sicuro extrapolare oltre di esse.

## File persistente e uso nel firmware

Copiare `nexus.cfg` nella root della microSD: il percorso PROS è `/usd/nexus.cfg`. Il file binario v1 contiene identificatore del robot, maschera delle categorie nexus_calibrate, parametri fisici nelle unità SI interne e CRC32. Le lunghezze memorizzate restano in metri e i coefficienti dinamici in V/(m/s) e V/(m/s²); la conversione riguarda solo CSV e report. Non modificare i valori binari per convertirli in mm. Non contiene porte, pose iniziali o comandi autonomous.

Il salvataggio scrive un file `.tmp`, verifica la rilettura, conserva il precedente profilo valido in `.bak` e rinomina il file nuovo. Un file principale corrotto può essere recuperato dal backup solo se versione, CRC, identificatore e valori fisici sono validi. Un caricamento fallito lascia inalterata la configurazione in memoria. `fflush` e rinomina sono disponibili via C standard; la resistenza completa a un'interruzione elettrica dipende dal filesystem e dal controller della microSD. Non rimuovere la scheda durante una scrittura.

Esempio avanzato indipendente dall'adattatore hardware, con le strutture matematiche interne in SI:

```cpp
#include "nexus/calibration.hpp"

nexus::EstimatorConfig estimator;
nexus::DynamicsConfig dynamics;
nexus::calibration::CalibrationProfile profile;
constexpr std::uint64_t hardwareId = 0x5341575032303236ULL;

bool recoveredBackup = false;
if (nexus::calibration::loadProfile("/usd/nexus.cfg", hardwareId,
                                    profile, &recoveredBackup) ==
    nexus::calibration::StorageStatus::ok) {
    nexus::calibration::applyProfile(profile, estimator, dynamics);
}
// Costruire lo chassis con estimator e dynamics prima di avviare i task.
```

Le API di fit dei CSV usano `std::vector` perché lavorano offline. Non chiamarle, né leggere/scrivere file, nel task di controllo o di localizzazione. La procedura guidata usa invece i collettori a capacità fissa e un task separato. Lo stimatore continua ad apprendere il bias del gyro nei periodi di quiete verificata. Accelerometro, latenze ed estrinseci dei sensori distance richiedono prove e riferimenti aggiuntivi, non ricavabili con affidabilità dai CSV descritti qui.

## Verifiche prima di riutilizzare un profilo

1. Controllare identificatore, diametri, rapporto di trasmissione, segni e offset meccanici.
2. Fare una corsa rettilinea e una rotazione misurate esternamente, differenti dalle prove di calibrazione.
3. Provare il modello su log separati e confrontare i due lati.
4. Misurare errori di posizione e heading finali su più ripetizioni autonomous.
5. Conservare log e risultati: ogni nuova stagione parte da misure verificabili, senza ereditare coefficienti non controllati.
