# Precisione: due Distance, senza GPS

Configurazione richiesta: compatibile VEX V5RC, nessun GPS e al massimo due sensori Distance. L'hardware dichiarato ha già un'IMU e due pod Rotation, uno longitudinale e uno laterale. Le porte Distance restano a zero finché non vengono installati e misurati i sensori: non sono stati inventati porte, offset o coefficienti fisici.

## Sensori e montaggio consigliati

La prima aggiunta utile è una coppia di **VEX Distance con fasci perpendicolari**, per esempio anteriore e destro. Quando vedono due pareti perpendicolari, possono correggere entrambe le coordinate usando l'heading dell'IMU. Scegliere i lati in funzione del percorso autonomous e della visibilità delle pareti. Due fasci paralleli informano soprattutto la stessa coordinata; non sono la prima scelta per correggere X e Y.

Misurare posizione e angolo della faccia ottica rispetto all'origine odometrica; fissarla rigidamente, senza meccanismi davanti. Misurare anche il rettangolo interno effettivamente visto dai raggi. Oggetti di gioco o altri robot possono interrompere la vista: i controlli robusti riducono gli errori, ma un oggetto statico che imita una parete può rimanere ambiguo.

VEX dichiara portata 20–2000 mm, accuratezza approssimativa ±15 mm sotto 200 mm e circa 5% oltre 200 mm. La calibrazione può correggere parte dell'errore sistematico, ma queste specifiche non giustificano promesse di accuratezza millimetrica. [Documentazione Distance VEX](https://kb.vex.com/hc/en-us/articles/360050696511-Using-the-Distance-Sensor-with-VEX-V5).

Come aggiunta successiva, un terzo **VEX Rotation su un secondo pod longitudinale**, separato lateralmente dal primo, fornirebbe una misura indipendente della rotazione del telaio. È un'inferenza cinematica: la differenza dei due percorsi divisa per la loro separazione misura la rotazione in assenza di slittamento. Potrebbe migliorare ridondanza e diagnosi dell'IMU, ma richiede contatto affidabile, meccanica calibrata e un'estensione dello stimatore: **il terzo pod non è attualmente acquisito o fuso**. Il Rotation misura angolo e giri; la sua specifica angolare non comprende l'errore della ruota sul pavimento. [Documentazione Rotation VEX](https://kb.vex.com/hc/en-us/articles/360051368331-Using-the-Rotation-Sensor-with-VEX-V5).

Prima di altri sensori, verificare precarico dei pod, assenza di gioco, diametro effettivo sotto carico e ripetibilità degli offset. Una seconda IMU non correggerebbe da sola la deriva di posizione. La configurazione proposta usa componenti VEX; l'ammissibilità dell'intero robot resta soggetta al manuale della stagione e all'ispezione. [Manuali ufficiali V5RC](https://www.vexrobotics.com/26-27-manuals).

## Miglioramenti implementati

1. **Velocità con meno ritardo.** Due stadi continui da 70 ms con compensazione del ritardo, integrati esattamente anche quando il replay divide un intervallo. Posa e covarianza non vengono alterate artificialmente. Nel confronto dinamico sintetico l'RMS della velocità scende da 40,41 a 12,06 mm/s e quello della velocità angolare da 0,05916 a 0,01576 rad/s. Il rumore a velocità costante aumenta del 2,4% nella prova utilizzata.
2. **Cronologia effettiva dei comandi.** Il controller ricostruisce quali tensioni raggiungono i motori durante la latenza, con memoria fissa. Compensa anche l'età della posa passata dal task di stima. Nel caso sintetico con 10 ms di ritardo comando e posa vecchia di 20 ms, l'RMS di tracking passa da 18,18 a 5,82 mm. La latenza configurata deve essere misurata sul robot.
3. **Calibrazione Distance individuale.** Nuovo comando `nexus_calibrate fit-distance distance.csv`: stima robusta di scala e offset, controllo della distribuzione delle distanze, outlier e intervallo validato. Non modifica automaticamente il profilo o i parametri di rumore. Il driver applica i coefficienti e rifiuta l'estrapolazione oltre l'intervallo provato.
4. **Rumore per sensore.** Ogni Distance può avere parametri diversi, usati coerentemente nelle correzioni e nel replay. I valori iniziali conservativi restano attivi finché misure indipendenti non giustificano una modifica.
5. **Letture vicine utilizzabili.** La confidenza PROS non è disponibile fino a 200 mm: viene rappresentata esplicitamente come assente, senza trasformarla in alta confidenza. Servono almeno cinque conferme e un peso prudente. Errori PROS e bassa confidenza effettivamente misurata oltre 200 mm continuano a essere respinti. [API PROS Distance](https://pros.cs.purdue.edu/v5/api/cpp/distance.html#get-confidence).

I miglioramenti numerici sopra riguardano modelli sintetici, non misure fisiche sul robot. Le prove estreme del controller a 1 mm / 0,1° migliorano complessivamente da 18/24 a 21/24 arrivi: rimangono tre mancati arrivi con modello errato. Non è un miglioramento universale su ogni scenario. Dettagli e compromessi in [localizzazione](localization.md) e [controllo](controller.md).

Sono conservati gli output del confronto controller [prima](precision-controller-before-2026-09-22.txt) e [dopo](precision-controller-after-2026-09-22.txt). La validazione finale supera tutte le 12 suite host, incluse 29.981 verifiche dello stimatore e il test specifico con due Distance. In quest'ultimo, con sensori sintetici calibrati e ritardo di 85 ms, l'errore X passa da 36,00 a 0,18 mm e Y da 81,00 a 9,38 mm rispetto alla sola odometria. È un confronto della correzione geometrica, non una misura dell'accuratezza dei Distance reali. [Registro completo dei test](precision-tests-2026-09-22.txt).

Firmware PROS hot/cold ricompilato e archivio `bin/NEXUS.a` rigenerato. Il primo passaggio ARM ha rilevato un avviso di nome locale mascherato: corretto, ricompilato e ricollegato senza nuovi avvisi. [Registro build ARM](precision-arm-2026-09-22.txt). Nessun firmware è stato caricato e il robot non è stato mosso in questa sessione.

## Configurazione e calibrazione

In `include/robot-config.hpp`, compilare al massimo due elementi di `wallSensors`. Le quattro posizioni della libreria sono mantenute per compatibilità del fingerprint; un controllo a compilazione impedisce più di due porte attive per questo robot. Ogni elemento contiene:

| Campi | Significato e unità |
| --- | --- |
| `port`, `x`, `y`, `heading` | Porta, posizione in mm e angolo in gradi; 0° avanti, +90° a destra |
| `latencyMs` | Ritardo stimato della lettura in ms |
| `distanceScale`, `distanceOffset` | `corretta = scala * grezza + offset`; offset in mm |
| `minimumDistance`, `maximumDistance` | Intervallo delle letture grezze validato in mm |
| `distanceStd`, `distanceRelativeStd` | Rumore base in mm e relativo adimensionale; -1 eredita i valori globali |

Il nucleo matematico usa SI e converte automaticamente. La disponibilità della confidenza dipende dalla lettura grezza, anche quando la correzione attraversa i 200 mm. Le calibrazioni Distance restano nella configurazione; formato binario e caricamento di `nexus.cfg` restano invariati.

Usare CSV `measured_mm,reference_mm`, almeno 12 inlier distribuiti su almeno 500 mm e almeno tre in ciascun terzo dell'intervallo. Verificare poi su distanze, superfici e inclinazioni non usate nel fit. Non copiare l'RMS del fit nel rumore del filtro. Procedura completa nella [guida alla calibrazione](calibration.md#4-scala-e-offset-dei-sensori-distance).

`commandLatencyMs` rende esplicito il ritardo del comando, inizialmente 10 ms. `Controller::update` accetta anche l'età della posa e Chassis la passa automaticamente. Il modello presume latenza costante; non conosce i comandi emessi prima di `start`/`reset`, per i quali assume zero durante il primo intervallo di latenza. Il filtro delle velocità non recupera timestamp hardware che PROS non espone.

## Limite con due Distance

Le correzioni locali X/Y sono disponibili. Il recupero globale esistente richiede almeno tre raggi attendibili verso pareti con geometria non ambigua: **con due Distance non è disponibile**. Il robot deve partire da una posa nota con `setPose`; se perde completamente il riferimento serve una nuova posa nota. Non viene azzerata l'incertezza per simulare un recupero.

Per misurare la precisione raggiunta, eseguire ripetizioni di rette, rotazioni, curve e autonomous completo, confrontando posa finale con riferimenti esterni indipendenti. Registrare errore massimo oltre a media/RMS, con batteria e carico diversi. I limiti finali dipendono da slittamento, contatto dei pod, gioco, visibilità delle pareti e calibrazione, oltre che dal software.
