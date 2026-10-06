# Prove dinamiche del robot via USB

Questa modalita misura la risposta reale del drivetrain: impulso di tensione,
rilascio in **COAST**, rotazione libera e ingresso rapido in curva. Si trova nel
firmware normale; non occorre sostituire `autonomous()` in `main.cpp`.
L'analisi produce parametri candidati e motivi di eventuale rifiuto. Non applica
automaticamente valori al robot o al profilo microSD.

## Prima registrazione

1. Compila e carica il firmware aggiornato con la normale procedura PROS.
   Avvia la guida libera, senza collegamento al controllo gara, e attendi READY.
2. Metti il robot sul campo con spazio libero attorno, lift basso e carico noto.
   Il cavo USB deve restare lasco senza trascinare il robot, soprattutto nelle
   rotazioni; una trazione del cavo falsifica la misura del coast.
3. Chiudi gli altri terminali USB. Dalla cartella del progetto esegui:

   ```powershell
   .\tools\capture_characterization.ps1
   ```

   PROS individua la porta. Se serve sceglierla, aggiungi `-Port COM9`, usando
   il numero effettivo della porta del tuo Brain. Il programma usa il terminale
   PROS per decodificare i pacchetti USB; i vecchi script `usb_odom_*` usano un
   protocollo diverso e non servono per queste prove.
4. Sul controller tieni **L1 + R1 per un secondo**, rilascia entrambi, poi premi
   **DOWN**. **LEFT / RIGHT** selezionano la prova mostrata sul Brain.
5. Parti da `Avanti 3 V`: tieni **R1** per tutta la prova. Dopo mezzo secondo
   fermo parte l'impulso; segue coast fino al riposo. Attendi `Prova conclusa`,
   poi rilascia R1. Rilasciarlo prima o premere **B** annulla la prova e frena.
6. Riposiziona il robot e ripeti. Ogni prova richiede una nuova pressione di R1;
   nessuna prova successiva parte da sola. B dalla selezione torna al menu.
7. Finite le prove, rilascia R1 e chiudi il terminale sul PC con **Ctrl+C**.
   Il percorso del log viene stampato all'avvio dello script.

La registrazione sul PC non avvia il robot e chiuderla non sostituisce il tasto
di arresto sul controller. Una perdita del controller, dati sensore mancanti,
comandi non rinnovati per 100 ms o cambio modalita revocano la prova. Un guasto
alla coda di registrazione annulla la prova. Dati mancanti nel file rendono la
prova inutilizzabile per il fit, anche se il robot ha completato il movimento.

## Serie da raccogliere

Sono disponibili 18 profili: sei movimenti a **3 V**, poi **4,5 V**, poi **6 V**.
Per ciascun livello: avanti, indietro, giro orario, giro antiorario, curva
sinistra, curva destra. Le curve accelerano dritte per 0,20 s, poi commutano
rapidamente al comando asimmetrico; il lato interno riceve un quarto della
tensione del lato esterno. Ogni profilo termina in coast.

Gli impulsi durano complessivamente 0,40 / 0,45 / 0,50 s secondo il livello.
Ripeti ogni profilo almeno tre volte, ripartendo sempre da fermo. Completa e
controlla prima la serie a 3 V. Registra in sessioni distinte robot vuoto, carico
usuale ed eventuale lift alto: mescolare condizioni diverse falsifica il modello.

Il programma interrompe una prova oltre 1,2 m di percorso misurato, 360 gradi
di rotazione totale, 720 gradi/s o 3 s di coast senza riposo. Sono limiti basati
sui sensori, non una garanzia di arresto entro una distanza dal bordo: serve
spazio libero anche per la frenata. La prima serie caratterizza fino a 6 V,
non certifica il comportamento a piena tensione.

## Analisi

Sostituisci il nome del log con quello creato dalla registrazione:

```powershell
python tools/analyze_characterization.py docs/characterization-AAAAMMGG-HHMMSS-mmm.log --output docs/characterization-results
```

Se conosci il peso complessivo di robot e carico, aggiungi `--mass-kg` seguito
dal valore in kg (punto per i decimali). Non e necessario per stimare la risposta
in tensione, velocita e accelerazione. Usa una cartella di output nuova per ogni
analisi: i risultati precedenti non vengono sovrascritti.

- `report.json`: prove accettate/rifiutate, motivi, qualita dei fit, decelerazione
  e decadimento in coast, risposta laterale.
- `candidates.csv`: solo se il fit dei due lati supera i controlli, propone
  `kSLeft/Right`, `kVLeft/Right`, `kALeft/Right` nelle unita di `robot-config.hpp`.
- `left.csv`, `right.csv`: campioni usati nel fit accettato, per l'analisi e gli
  strumenti di calibrazione esistenti.

Il fit motore usa i rettilinei alimentati. Il coast viene analizzato separatamente:
zero tensione con uscita in COAST non equivale al modello di frenata motorizzata.
Con il peso si stima una forza resistente **effettiva**, che comprende trasmissione
e ruote; non e una misura isolata del coefficiente di attrito del pavimento.
Il coast in rotazione determina decelerazione e decadimento, ma non separa da
solo attrito e momento d'inerzia assoluto. Per il controller sono gia utili la
risposta misurata e i coefficienti equivalenti.

I parametri laterali sono sperimentali e non vengono trasformati automaticamente
in un nuovo modello del controller. Dopo la raccolta si confrontano i risultati
con i dati grezzi e con una prova indipendente, poi si aggiornano modello e
limiti di velocita/accelerazione. Anche le scale di pod e IMU devono essere
gia verificate: la caratterizzazione non puo scoprire ogni errore comune di scala.

## Dati registrati e verifiche

Il flusso `NXCHAR` versione 1 registra circa 50 campioni/s, piu le transizioni
di fase: identificatore prova, fase/esito, comandi, tensioni motore misurate,
batteria, pod e IMU grezzi, accelerometro, posa e velocita stimate, configurazione
geometrica, validita, sequenza e campioni persi. Tempi di controllo, sensori e
letture motore sono separati: l'API non fornisce una misura hardware atomica.
La coda e limitata e la scrittura USB avviene in un task separato dai comandi.

Le velocita motore usano soltanto gli encoder dei quattro 600 RPM selezionati
dalle maschere fisiche, anche quando sono esclusi dall'odometria. I due centrali
200 RPM con rapporto diverso non vengono mediati con essi. La stessa correzione
e applicata al logger SD esistente.

Il metodo di identificazione `V = kS sign(v) + kV v + kA a` e descritto nella
[documentazione WPILib](https://docs.wpilib.org/en/2020/docs/software/wpilib-tools/robot-characterization/introduction.html).
La gestione USB usa il [terminale PROS e il suo framing](https://pros.cs.purdue.edu/v5/pros-4/filesystem.html).
I test host verificano sequenze e guasti, coast/stop dell'adapter e recupero dei
parametri da dati sintetici. La prima sessione sul campo resta da eseguire.
