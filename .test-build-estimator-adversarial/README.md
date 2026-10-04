# Analisi esplorativa dei tempi dei sensori

`variable_sensor_age.py` usa il tratto 130–166 s di
`docs/odometry-usb-01.csv`. Le letture registrate definiscono tre curve lineari
fra campioni (pod F, pod L, angolo IMU); il probe applica **ulteriori** ritardi
causali, con tempi di acquisizione non decrescenti per ciascun canale. I valori
agli estremi vengono mantenuti e tutti i canali raggiungono gli stessi valori
finali prima di confrontare gli endpoint.

Questa costruzione non ricostruisce i tempi di acquisizione del robot: né le
età fisiche dei campioni né il movimento fra due letture sono misurati. I clock
dei pacchetti nel log indicano ricezione, non necessariamente acquisizione.

## Risultato dei programmi predefiniti

Il massimo cambiamento dell'endpoint nelle simulazioni con età aggiunta
0–10 ms è **1,832475 mm** (componenti +1,785397 e −0,412700 mm), rispetto
al replay senza ritardi aggiunti. Il massimo dei 128 programmi casuali
continui è 0,633295 mm; quello dei 128 programmi su griglia di acquisizione
5 ms è 0,518877 mm. I casi con mantenimento fino a 20 ms sono stress separati,
fuori dall'ipotesi 0–10 ms.

Non è una ricerca della correzione che chiude il percorso. Il massimo
osservato non è il massimo matematico su tutte le sequenze di ritardo e non è
un limite all'errore fisico. Questi risultati **non escludono la sincronizzazione
come causa** della prova reale e non ne dimostrano la responsabilità.

## Limite discreto condizionato

Il risultato indicato come `TIGHTER CONDITIONAL DISCRETE BOUND` è
**27,605518 mm** per età aggiunte 0–10 ms: 13,445566 mm per i contatori e
14,159952 mm per la proiezione angolare. È un limite del solo modello di
ricampionamento sopra definito, non del percorso fisico che ha prodotto il CSV.

La derivazione usa tre passaggi:

1. Scrivendo il contatore ritardato come `F'_i = F_i - d_i`, la sommazione per
   parti trasforma il cambiamento a coefficienti di proiezione originali
   `A_i` in `sum d_i (A_(i+1) - A_i)`. Gli scarti ai due estremi sono nulli;
   lo scarto interno è limitato dall'escursione della curva sorgente nei 10 ms
   precedenti. Lo stesso vale per L.
2. Il coefficiente della corda è la media dei versori fra gli angoli ai due
   estremi. Il suo cambiamento in norma è al più la media degli scarti assoluti
   dei due angoli, perché la rotazione unitaria è Lipschitz con costante 1.
3. Con acquisizioni monotone gli intervalli di ciascun contatore ripartiscono
   la curva sorgente: la variazione totale è contata una sola volta. Un elemento
   sorgente al tempo `u` può contribuire soltanto a un intervallo host che termina
   fra `u` e `u + ritardoMassimo + intervalloHostMassimo`. Il codice prende il
   massimo del peso angolare su questo insieme. I contributi degli offset
   telescopici coincidono perché gli angoli iniziali e finali coincidono.

Il limite torna zero a ritardo zero. La sua validità richiede proprio le curve
interpolate, i ritardi aggiunti limitati e monotoni, gli estremi riallineati e
la stessa integrazione a corda. Non copre traiettorie intermedie ignote,
filtraggi interni dei dispositivi, una ricostruzione inversa dei timestamp né
ritardi fisici non misurati. Le stime grossolane stampate prima di questo
limite sono conservate come esplorazione, non usate per una conclusione sul
percorso reale.

Il confronto dinamico tra derivata dell'angolo e giroscopio grezzo dà per Z
correlazione −0,998068 al ritardo discreto zero. Il segno grezzo è opposto a
quello dell'angolo; il robot non usa il rate grezzo nella posa. La correlazione
non stabilisce l'età fisica di nessuno dei due segnali o dei pod.

Output completo: `variable_sensor_age_results.txt`. Nessuna modifica al firmware,
nessuna compensazione applicata e nessun caricamento sul Brain.
