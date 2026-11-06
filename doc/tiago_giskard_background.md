# Giskard persistente e invio goal separato

## Avvio unico consigliato

Nel container, dopo aver configurato la Ethernet del laptop sulla sottorete
del robot, fermare le istanze di bringup/client avviate manualmente e usare:

```bash
cd /workspace/cram
bash scripts/launch_tiago_giskard.sh
```

Il launcher carica l'ambiente ROS e imposta dominio 72, CycloneDDS con il file
locale e un thread BLAS/OMP. Avvia due processi persistenti, senza inviare goal.
Mostra una vista essenziale dei log server/client e attende `STACK READY`: i processi sono collegati,
non necessariamente autorizzati al movimento. Presa collegata e teleop attiva
non impediscono il bringup; il client mostra `GOALS BLOCKED` con i motivi e
rimane attivo. Al rilascio degli interlock e con feedback valido mostra
`GOALS ENABLED`. Da un altro terminale
dello stesso container, con condizioni operative confermate:

```bash
./scripts/tiago-goal 0.30
```

La distanza è in metri: ad esempio `./scripts/tiago-goal 0.5` richiede 50 cm.
Il comando non avvia Python/ROS e non modifica il client persistente. La conferma
di consegna non è conferma di accettazione: il comando resta in primo piano fino
a `SUCCESS`, `CANCELED` o rifiuto/fallimento. Ctrl+C in questo terminale richiede
la cancellazione del solo goal inviato da questo comando e attende la conferma.
Il client e il server persistenti restano attivi. Ctrl+C nel terminale del
launcher mantiene invece il significato di arrestare entrambi i processi.
Riavviare il launcher dopo l'aggiornamento per caricare il nuovo protocollo.
Per il vecchio invio senza attesa usare `./scripts/tiago-goal 0.3 --no-wait`;
in questa modalità il comando termina subito e non gestisce Ctrl+C sul goal.
Con una pipe diversa usare `./scripts/tiago-goal 0.3 --fifo /percorso/pipe`.
Per usare il nome breve anche fuori dalla directory del repository, nel terminale:

```bash
export PATH="/workspace/cram/scripts:$PATH"
tiago-goal 0.3
```

Ctrl+C nel launcher invia SIGTERM ai soli processi avviati dal launcher e
attende fino a 15 s; se non terminano usa SIGKILL come recupero finale.
I log restano nella directory stampata all'avvio. L'uscita di uno
dei figli provoca l'arresto dell'altro. Il launcher non adotta né termina
istanze manuali preesistenti. Non avvia controller hardware.

Opzioni: `--fifo PATH`, `--log-dir PATH`, `--startup-timeout SECONDS` (default
90 s per processo). La directory di log esplicitamente scelta viene riutilizzata
troncando i due file server/client. `--help` non avvia alcun processo figlio.

Nel terminale vengono filtrati i messaggi URDF non riconosciuti, gli INFO ROS/Python
di routine e i dettagli dei traceback. Gli esiti, le latenze, gli errori e gli avvisi
restano visibili; il warning noto sul database opzionale viene filtrato. Il flusso
integrale di ciascun processo resta nei file `server.log` e `client.log`.
Il filtro viene eseguito in modalità interattiva per evitare il buffering
dell'input di `mawk` sulle pipe: anche poche righe devono apparire durante la
prova, senza attendere il riempimento del buffer o la fine dei processi.
Per visualizzarlo anche in terminale usare:

```bash
bash scripts/launch_tiago_giskard.sh --verbose
```

Lo stato idle del client è riassunto su una riga, per esempio
`GOALS BLOCKED: teleop attiva; ricarica attiva`, e viene ripetuto solo al cambiamento
dei motivi effettivi, anziché per ogni variazione dei dettagli del mux.

Anche senza `--verbose`, `CLIENT RECEIVED` riporta il timestamp Unix della
ricezione della richiesta nel client, prima dei controlli del singolo goal.
`SUCCESS` riporta il timestamp al rilevamento del risultato positivo e il tempo
monotono trascorso dalla richiesta. È un evento lato client, non il timestamp
esatto della transizione sul server. `FINAL POSE` viene campionato dopo un secondo
di assestamento e non determina il timestamp di successo.

Eseguire nel container `aicor-cram-demo`, repository in `/workspace/cram`.
In ciascun terminale del container caricare prima:

```bash
source /opt/ros/overlay_ws/install/setup.bash
cd /workspace/cram
export ROS_DOMAIN_ID=72
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/cram/doc/tiago_cyclonedds.xml
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
```

Configurare prima la Ethernet sulla sottorete del robot. Usare un solo
server Giskard. Non eseguire contemporaneamente il vecchio `trial.py`, che
avvia anch'esso un server. Questi script non avviano controller hardware.

## Terminale 1: bringup

```bash
/opt/ros/cram-env/bin/python -u scripts/tiago_giskard_bringup.py
```

Resta in attesa su `/giskard/command`, senza inviare goal. Per eseguirlo
in background nello stesso terminale:

```bash
/opt/ros/cram-env/bin/python -u scripts/tiago_giskard_bringup.py > /tmp/tiago-giskard.log 2>&1 &
tiago_giskard_pid=$!
tail -f /tmp/tiago-giskard.log
```

Attendere `READY` prima del client. I feedback continuano ad aggiornarsi
anche da fermo; il client verifica che siano freschi prima di inviare il goal.
Interrompere `tail` con Ctrl+C lascia attivo il server. Per arrestare il
server usare `kill -TERM "$tiago_giskard_pid"` nella shell che conserva il PID,
oppure Ctrl+C se avviato in primo piano. Non usare SIGKILL come arresto normale.

## Terminale 2: un goal reale

Le tolleranze sono definite in `MotionTolerances` dentro
`scripts/tiago_giskard_goal.py`: `position` è in metri (attualmente 0.02),
`orientation_degrees` in gradi (attualmente 5). I task e il messaggio `SENDING`
usano questa configurazione, senza testo numerico duplicato nel log.
Riavviare il client, oppure il launcher unico, dopo aver modificato i valori.

Per verificare soltanto gli ingressi del client, senza inviare goal:

```bash
/opt/ros/cram-env/bin/python -u scripts/tiago_giskard_goal.py --check
```

Un rifiuto elenca gli stati mancanti, eventuali `joy_priority`/emergenza,
i lock attivi del mux e gli scarti temporali oltre ±2 s, coerenti con il guard
del server. Il controllo fallisce prima di contattare il server del goal:
è normale che in questo caso il log Giskard non mostri goal ricevuti.

Operatore presente, area libera, teleop disattivata ed emergenza raggiungibile.

```bash
/opt/ros/cram-env/bin/python -u scripts/tiago_giskard_goal.py --distance 0.30 --execute
```

La distanza è in metri, positiva e finita; default 0.30. Il client termina
dopo questo goal, mentre il server rimane in attesa. Ogni richiesta acquisisce
nuovamente la posa iniziale reale. Riferimento solo `odom`, orientamento iniziale
fisso, posizione lungo una retta, tolleranze configurate in `MotionTolerances`, limiti server 0.2 m/s e
0.1 rad/s. Anticollisione del modello attiva: non sostituisce la supervisione
dell'operatore e non rappresenta necessariamente tutti gli ostacoli dell'ambiente.

Il client richiede emergenza e priorità joy rilasciate, mux libero e feedback
odometria/giunti fresco e finito. Durante il goal richiede cancellazione per
interlock, feedback non valido o timeout di 60 s. Ctrl+C richiede cancellazione
del proprio goal. Non vengono scritti file in `/trial`.

## Client persistente e comando leggero da un altro terminale

Con bringup già attivo, caricare l'ambiente ROS/DDS anche nel terminale client.
Avviare una sola istanza del client:

```bash
/opt/ros/cram-env/bin/python -u scripts/tiago_giskard_goal.py --listen /tmp/tiago-giskard-goals.fifo
```

Il caricamento iniziale include import, recupero del mondo remoto,
sincronizzatore, action client, heartbeat e worker. Non viene inviato alcun goal
all'avvio. Attendere `READY CLIENT`, quindi in un altro terminale dello stesso
container inviare soltanto la distanza in metri:

```bash
test -p /tmp/tiago-giskard-goals.fifo && printf '0.30\n' > /tmp/tiago-giskard-goals.fifo
```

Questo comando avvia realmente un goal di 30 cm se i controlli passano. Il solo
`printf` non importa Python o ROS: utilizza le connessioni già aperte. Il terminale
del client mostra `WHEEL ONSET` e l'esito. Attendere il successivo `READY CLIENT`
prima di una nuova richiesta. Le richieste ricevute mentre un goal viene eseguito
vengono scartate, non accodate per movimenti automatici successivi.

Per cancellare il solo goal attivo mantenendo entrambi i processi collegati,
premere Ctrl+C nel terminale di `tiago-goal DISTANZA`. In alternativa, da un
altro terminale dello stesso container:

```bash
./scripts/tiago-goal --cancel
```

Il client legge il comando anche durante il movimento, richiede la cancellazione
tramite l'action client e stampa `CANCEL REQUESTED` e poi `CANCELED` alla risposta
del server. Questo esito conferma la cancellazione dell'azione, non misura
l'istante fisico di arresto delle ruote. In assenza di un goal attivo stampa
`CANCEL: no active goal`; non conserva la richiesta per un goal futuro.
Altre distanze ricevute mentre il client è occupato vengono rifiutate.

Il comando foreground usa un ID univoco e un piccolo file di risposta privato
in `/tmp/tiago-goal.XXXXXX`, rimosso all'uscita del comando. Le cancellazioni da
Ctrl+C sono associate a quell'ID: una richiesta vecchia non cancella un goal
successivo. Non vengono avviati nuovi processi Python o ROS a ogni richiesta.

Per chiudere il client senza fermare il server Giskard:

```bash
printf 'quit\n' > /tmp/tiago-giskard-goals.fifo
```

Oppure usare Ctrl+C nel terminale client. I blocchi di ricarica/joystick, feedback
mancanti o non validi e ruote già in movimento rifiutano il singolo goal senza
terminare il client. Le richieste bloccate non vengono conservate: dopo il rilascio
occorre inviarle nuovamente. Un goal fallito viene riportato e il client resta
collegato. Se un interlock si attiva durante il movimento rimane attiva la
cancellazione del goal. Errori di infrastruttura o di inizializzazione possono
ancora terminare il processo. La modalità one-shot `--execute` e `--check`
mantengono il preflight iniziale con attesa massima di 10 s.
Un'interruzione forzata potrebbe lasciare la FIFO;
non inviare comandi se il client non è attivo e pronto. È disponibile anche
`--interactive`: dopo il caricamento si digitano direttamente `0.30` o `quit`
nello stesso terminale, senza un secondo comando Python.

La misura usa il tempo monotono locale: ricezione della riga di comando → primo
callback di `/mobile_base_controller/odom` con velocità lineare almeno 0.005 m/s
o angolare almeno 0.01 rad/s. Conserva un solo timestamp per richiesta, senza
registrare flussi o modificare il loop Giskard. Richiede ruote ferme e feedback
recente prima del goal. Include il ritardo di pubblicazione e trasporto degli
encoder; non misura l'istante fisico esatto e non è un limite garantito di latenza.

La modalità persistente elimina l'avvio del client da ogni nuova richiesta;
parsing e compilazione del goal Giskard restano. Non è ancora stata validata
una latenza inferiore a 15 s sul robot con questa modalità.

## Sequenza automatica: movimento, cancellazione, nuovo goal

Riavviare il launcher per caricare il protocollo aggiornato del client. Con server
e client già pronti, eseguire manualmente nel container:

```bash
cd /workspace/cram
/opt/ros/cram-env/bin/python scripts/tiago_goal_sequence.py --execute
```

La sequenza conserva i valori delle ultime prove: primo goal di 0,5 m,
secondo di 1,2 m, ritardo di cancellazione 0,4 s. Le distanze si cambiano con
`--first-distance` e `--second-distance`, senza modificare il codice.
Dopo il primo feedback di movimento attende il ritardo configurato e invia
SIGINT al comando `tiago-goal`, usando lo stesso percorso
di cancellazione di Ctrl+C. Attende la conferma di cancellazione, feedback recente
di ruote ferme per almeno 0,2 s con almeno tre campioni e il rilascio degli
interlock. Poi invia la seconda distanza in avanti rispetto alla posa raggiunta,
mantenendone l'orientamento iniziale. Il primo goal viene interrotto: il percorso
totale non è la somma delle due distanze richieste.

Il ritardo è modificabile con `--cancel-after 0.5`; una FIFO diversa si indica
con `--fifo PATH`. La verifica di arresto riusa i feedback del client persistente,
senza registratori né ulteriori nodi ROS. Se la cancellazione non è confermata,
il primo goal termina altrimenti o le condizioni non tornano pronte, il secondo
goal non viene inviato. Ctrl+C durante la sequenza cancella il comando posseduto
e interrompe la sequenza, lasciando server e client attivi.

## Cambio di direzione dopo cancellazione

Dopo aver riavviato il launcher con il client aggiornato, questa variante
richiede un primo goal di 0,5 m, lo cancella dopo 0,4 s dal feedback di movimento,
attende l'arresto, ruota di 30° a sinistra e avanza di 0,4 m:

```bash
/opt/ros/cram-env/bin/python scripts/tiago_goal_sequence.py --execute \
  --first-distance 0.5 --cancel-after 0.4 \
  --turn-degrees 30 --second-distance 0.4
```

La rotazione è un'azione Giskard separata: `CartesianPosition` mantiene il punto
iniziale e `CartesianOrientation` richiede l'orientamento iniziale più 30°.
Usa le stesse `MotionTolerances` degli avanzamenti, senza una retta di lunghezza
zero. Rimangono i limiti server di 0,2 m/s e 0,1 rad/s e tutti gli interlock.
La sequenza richiede successo della rotazione e una nuova conferma di ruote
ferme prima di avanzare. Un rifiuto, fallimento o cancellazione della rotazione
impedisce l'avanzamento successivo.

I 40 cm sono calcolati dalla posa misurata dopo la rotazione, lungo la direzione
effettivamente raggiunta. Con tolleranza di 5° la rotazione non è necessariamente
esattamente 30°. Per confrontare i risultati, distinguere questo errore angolare
dall'errore lungo il segmento finale.

È disponibile anche la sola rotazione, sempre con risultato in primo piano e
cancellazione tramite Ctrl+C:

```bash
./scripts/tiago-goal --turn 30
```

Angoli positivi ruotano a sinistra; negativi a destra. Sono accettati angoli
finiti, non nulli, entro ±180°. La rotazione non supporta `--no-wait`.
`--turn-degrees 0` (default) lascia la sequenza precedente senza svolta.

## Confronto con DifferentialDriveBaseGoal: 40 cm

Riavviare il launcher completo, server e client, per caricare anche il parametro
angolare aggiunto al goal nativo. Poi, da un secondo terminale del container:

```bash
cd /workspace/cram
./scripts/tiago-goal 0.4 --differential-drive
```

Invia un solo goal, senza cancellazione automatica, 40 cm davanti alla posa
misurata al momento della richiesta. L'orientamento finale richiesto coincide
con quello iniziale. Il target viene espresso in `odom`; non usa mappa o AMCL.
`SENDING` deve indicare `DifferentialDriveBaseGoal` e le tolleranze effettive
`0.02 m / 5 deg`, lette dalla stessa `MotionTolerances` dei task cartesiani.
Rimangono il limite di 0,2 m/s, il timeout di 60 s e la cancellazione con Ctrl+C.

Il goal nativo mantiene le sue tre fasi: orientamento verso la destinazione,
avvicinamento e orientamento finale. Non impone un orientamento costante lungo
l'intero percorso. La soglia `threshold` viene usata per la posizione, mentre
il nuovo parametro opzionale `orientation_threshold` applica i 5° in radianti
a tutte e tre le fasi. Se quest'ultimo è omesso, rimane il comportamento
precedente, con un solo valore numerico per entrambe le grandezze.

Per confrontare un goal cartesiano con la stessa distanza e configurazione:

```bash
./scripts/tiago-goal 0.4
```

Eseguire le prove separatamente, con ruote ferme e condizioni operative
confermate. Nessuno dei due comandi avvia nuovi server o controller hardware.

## Esperimento di rotazione con velocità longitudinale zero

La variante è opzionale e non modifica i task originali di Giskard. Il task
aggiunto vive in `experiments/src/experiments/tiago_pure_turn.py`; gli script
di prova lo inviano solo quando è specificato `--pure-turn`.
Riavviare il launcher per caricare il protocollo aggiornato del client:

```bash
cd /workspace/cram
bash scripts/launch_tiago_giskard.sh
```

Dopo `STACK READY`, eseguire personalmente da un altro terminale del container,
con le condizioni operative confermate:

```bash
cd /workspace/cram
/opt/ros/cram-env/bin/python scripts/tiago_goal_sequence.py --execute \
  --first-distance 0.5 --cancel-after 0.4 \
  --turn-degrees 30 --second-distance 0.4 --pure-turn
```

La sequenza conserva cancellazione, attese delle ruote ferme e avanzamenti
cartesiani. Durante la sola rotazione sostituisce il mantenimento della posizione
con `CartesianOrientation` e un vincolo `v_forward = 0` nel QP, applicato a tutti
i passi dell'orizzonte di controllo, con entrambi i limiti di slack pari a zero.
La normalizzazione resta positiva; non viene impostata a zero e non viene
modificato o filtrato il comando dopo il solver.

Il log deve indicare `PURE TURN; solver forward velocity=0; no position recovery`.
La rotazione mantiene la tolleranza angolare configurata (attualmente 5°) e
il limite server di 0,1 rad/s. In caso di fallimento o cancellazione non parte
l'avanzamento successivo. Il vincolo appartiene alla singola azione di rotazione:
il nuovo avanzamento usa nuovamente il goal rettilineo senza blocco longitudinale.

Per provare solo la rotazione:

```bash
./scripts/tiago-goal --turn 30 --pure-turn
```

Senza `--pure-turn` resta disponibile la rotazione precedente con recupero di
posizione. Entrambe le varianti conservano Ctrl+C e gli interlock. La variante
impone velocità comandata zero entro la precisione numerica del solver, ma non
garantisce assenza di spostamenti fisici dovuti a slittamento o asimmetrie della
base. Non effettua recuperi automatici della posizione durante la rotazione.

## Errori rispetto al target nei log del client

Dopo aver riavviato il client o il launcher, i comandi esistenti producono anche
righe `GOAL ERROR`, visibili nella vista essenziale del terminale. Il client usa
soltanto l'ultimo messaggio di `/direct_laser_odometry/odom` già ricevuto: non
apre ulteriori sottoscrizioni e non registra una storia dei campioni.

Durante l'azione stampa al massimo una riga al secondo (`phase=tracking`).
All'arrivo dell'esito stampa sempre un campione (`phase=result`), anche in caso
di cancellazione o errore, prima del messaggio di esito. Dopo un successo stampa
anche un campione dopo il secondo di attesa già previsto (`phase=settled`);
questa etichetta non costituisce una nuova verifica dell'arresto fisico.

I valori indicano il target meno la posa misurata:

- `longitudinal`, metri nell'asse avanti iniziale: positivo se manca avanzamento,
  negativo se il robot ha superato il target lungo quell'asse;
- `lateral`, metri nell'asse sinistra iniziale: positivo se la correzione verso il
  target sarebbe a sinistra, negativo se sarebbe a destra;
- `angular`, gradi della correzione di yaw più breve: positivo verso sinistra;
- `planar`, norma dei due errori di posizione, non una soglia distinta per asse;
- `elapsed`, secondi monotoni dalla ricezione della richiesta nel client;
- `odom_stamp`, timestamp ROS del campione utilizzato, utile per vedere se il
  feedback sta avanzando.

Ogni goal acquisisce nuovi assi iniziali: il secondo avanzamento viene quindi
valutato lungo la direzione misurata dopo la rotazione. Per la rotazione pura,
gli errori di posizione sono diagnostici e non attivano un recupero della posa.
Questi sono errori planari calcolati dal client; non sono una lettura dello stato
interno del monitor Giskard, che può usare una posa sincronizzata diversamente
e controlla anche componenti tridimensionali. Nessuna tolleranza o condizione
di completamento viene modificata da questa scrittura dei log.
