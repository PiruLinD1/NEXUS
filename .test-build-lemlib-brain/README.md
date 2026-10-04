# Prova separata LemLib sul Brain

Programma **LEMLIBTEST**, slot predefinito **2**. È stato preparato e compilato: **non è stato caricato**. Lo slot va controllato sul Brain prima di usarlo; il progetto NEXUS resta separato.

1. Posiziona il robot sul riferimento e segna anche la direzione del muso.
2. Dopo il caricamento nello slot verificato, avvia **LEMLIBTEST**. Attendi fermo durante la calibrazione IMU.
3. Controlla il titolo **LEMLIB 0.5.6 | 3388145** e annota X, Y e heading iniziali.
4. Ripeti a mano la stessa S di andata e ritorno. Questo programma non aziona motori o meccanismi.
5. Riporta il robot sul riferimento con lo stesso orientamento, attendi **almeno 3 secondi fermo** e annota X, Y e heading finali.
6. Per una nuova prova, chiudi e riavvia l’app: non ci sono pulsanti di azzeramento. Per tornare al robot normale, avvia NEXUS nel suo slot.

Il codice odometrico e i getter `TrackingWheel` sono quelli originali [LemLib v0.5.6, commit 3388145e0d90ee3c7c17d7267a30ef64367df4cb](https://github.com/LemLib/LemLib/tree/3388145e0d90ee3c7c17d7267a30ef64367df4cb). Tutti i 48 file importati LemLib/fmt sono identici all’archivio ufficiale e verificabili con `UPSTREAM-MANIFEST.json`. Licenza MIT in `LICENSE-LemLib`; i componenti SDK conservano le proprie licenze nei file. `src/main.cpp` contiene soltanto l’avvio, la configurazione dei sensori e la visualizzazione. Non include NEXUS e non istanzia `lemlib::Chassis` o controllori di movimento.

Configurazione: IMU porta 1; pod avanti porta 2 normale; pod orizzontale porta 3 normale; diametri **2.0 pollici esatti**; offset avanti **+20 mm**, laterale **−70 mm**. La porta laterale è normale perché il sorgente LemLib proietta il contatore orizzontale positivo verso sinistra, mentre NEXUS usa la porta −3 per avere il positivo a destra. Gli offset mantengono il proprio segno. Le letture mostrate come `L destra` e `lateral_right_mm` hanno il segno fisico di NEXUS; questa conversione riguarda solo la visualizzazione, mentre LemLib legge direttamente il sensore normale.

L’updater upstream richiede un secondo tracking wheel verticale non nullo anche quando l’heading viene dall’IMU. Il sostituto originale legge il gruppo destro `{-7,9,-10}` (ruota 3.25 in, 360 rpm alla ruota, mezzeria +143.5 mm). Il suo costruttore imposta le unità encoder a rotazioni; **non comanda movimento, frenatura o azzeramento dei motori**. Con pod avanti e IMU presenti, i valori del sostituto non entrano nella posa.

La calibrazione IMU è quella nativa PROS. Il task originale esegue `lemlib::update()` seguito da `pros::delay(10)`; i dispositivi sono impostati a 5 ms come nel progetto NEXUS. Nessun bias locale, filtro aggiunto, resampling o getter simulato. LCD e USB sono aggiornati da un altro task a circa 10 Hz. Le righe USB `LL06` contengono posa LemLib e letture diagnostiche successive: **non sono campioni sincronizzati dell’updater** e non vanno interpretate come una traccia odometrica a 100 Hz. Per la prova A/B si confrontano le posizioni ferme iniziale/finale; il getter della posa resta quello upstream.

Compilazione dalla cartella di questo progetto con il toolchain PROS locale:

```powershell
$toolchain = 'C:/Users/rugge/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-windows/usr/bin'
$env:Path = "$toolchain;" + $env:Path
& "$toolchain/make.exe" -j4
```

Il progetto contiene il kernel PROS **4.2.1** e liblvgl **9.2.0** copiati dal progetto corrente. Il worker di avvio ha priorità inferiore al callback `initialize`, così la calibrazione bloccante non allunga il callback PROS. Il controllo hardware della prova e l’upload restano da eseguire.
