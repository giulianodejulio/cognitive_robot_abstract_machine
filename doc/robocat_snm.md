# SNM senza HPM: prima prova supervisionata

Per avviare insieme preparazione controller, bridge, server e terminale SNM
dall'host vedere [robocat_byobu.md](robocat_byobu.md).

Questa prima versione esegue una richiesta da CLI: Nav2 raggiunge lo staging,
SNM verifica arresto e condizioni di ingresso, quindi invia un goal cartesiano
rettilineo a Giskard. Il piano Giskard viene costruito con le API CRAM native.
Il coordinatore è `EntryMission`; la logica di ingresso è `EntryPolicy`.
Non sono ancora implementati selezione di posti alternativi, HPM, TTS,
cambio piano o interfaccia action CRAM–SNM. Non c'è ripartenza automatica dopo
un'interruzione. La futura action può riusare la logica della richiesta.

Il codice è stato verificato senza robot: i test di compilazione costruiscono
i vincoli sul modello reale acquisito ma non eseguono cicli di controllo.
Il successo dei test software non certifica una prova fisica riuscita.

Verifica software del 12 novembre 2026: 71 test mirati superati, inclusi policy,
geometria, compilazione del task nativo, trasporto locale a pipe, cancellazione,
diagnostica e regressioni dei launcher/routing Robocat. Eseguiti nel container,
con `--orm-build never`, un thread BLAS e limite di memoria virtuale di 2,5 GiB.

## Sequenza

1. Controllo del server Giskard: feedback wheel e comando `/snm_vel`.
2. Controllo nativo in sola lettura dei moltiplicatori ruota `2.0 / 2.0 / 1.22`
   e degli ingressi mux `navigation=cmd_vel/10`, `snm=snm_vel/20`.
3. Invio del solo goal Nav2 posseduto dalla richiesta; goal altrui attivi
   impediscono l'avvio e non vengono cancellati.
4. Attesa di `SUCCEEDED`, assenza di altri goal Nav2, mux libero e ruote ferme.
5. Verifica della mappa e della posa effettivamente raggiunta; cattura della
   trasformazione map–wheel. Una correzione AMCL durante l'ingresso non sposta
   il target già catturato; un cambio di mappa invalida la richiesta.
6. Attesa di almeno tre scan distinte e un secondo di apertura/passaggio liberi.
7. Un solo goal `CartesianPositionStraight` + `CartesianOrientation`, con
   tolleranze 2 cm / 5°, riusando `RobocatStraightMotion`.
8. Successo solo dopo risultato Giskard, posa odometrica entro tolleranza,
   parte posteriore del robot oltre soglia + margine e arresto misurato.

I timeout di attesa e movimento sono separati. `Ctrl+C` richiede la cancellazione
del goal posseduto, anche se l'accettazione è in ritardo. La supervisione nativa
annulla il proprio goal Nav2 se perde gli heartbeat del laptop. Il rilevamento
di un nuovo goal Nav2 durante l'ingresso richiede la cancellazione di Giskard;
non cancella il goal dell'altro client.

## Geometria da verificare prima di eseguire

`--staging` legge un file nella forma di `doc/robocat_elevator_staging.yaml`.
Le coordinate sono quelle salvate; non vengono cercate nuovamente in Eulero.
Verificare in RViz che corrispondano alla mappa attiva e che lo yaw punti
verso la cabina. Il waypoint non viene modificato dal comando.

`--doorway-distance D` è la distanza **dallo staging al centro della soglia
lungo l'asse X dello staging**, in metri. Deve essere misurata/calibrata nella
mappa corrente: non viene recuperata dalla vecchia calibrazione in odom.

`--distance 1.5` colloca il target a 1,5 m lungo lo stesso asse **dallo staging**,
non 1,5 m dalla posa residua dopo l'arrivo Nav2. L'ingresso è ammesso solo se
la posa raggiunta è entro 2 cm e 5° dallo staging. Non viene aggiunta una
rotazione o una manovra di raccordo davanti alla porta.

Ingombri iniziali: 16 cm davanti e 30 cm dietro `base_footprint`, corridoio largo
60 cm, margine longitudinale 10 cm. Sono i limiti della prima configurazione:
confrontarli con il robot fisico e gli accessori prima della prova. Il target
deve lasciare la parte posteriore completamente oltre la soglia. Sono accettate
distanze fino a 2 m. La mappa deve lasciare libero anche il corridoio con
clearance statica di 30 cm dal centro; il comando non modifica né ritaglia
la mappa per aggirare un blocco.

Il controllo lidar riusa `LidarReading.distances_to_segment` del framework,
verifica il segmento porta e sezioni del corridoio restante. Richiede copertura
di entrambi i bordi e misure utilizzabili oltre ogni sezione. Nel flusso PAL
verificato, `+Inf` indica nessun ritorno e può certificare il tratto entro 25 m
(comunque entro `range_max`). NaN, settori mascherati, dati scaduti e copertura
insufficiente restano **sconosciuti**; `-Inf` e ostacoli misurati bloccano.
È un controllo 2D: non misura il gap verticale, oggetti fuori piano o visibilità
occlusa. Non riconosce semanticamente persone o ascensori.

Per questa prova le porte devono essere **mantenute aperte fisicamente**;
`--doors-held-open` è la conferma dell'operatore, non un comando all'ascensore
e non una misura della futura richiusura. Appena lo chassis inizia a varcare
la soglia, la sola perdita di visibilità della porta dietro al robot non ferma
il movimento. Un ostacolo misurato o dati indisponibili nel corridoio restante,
guasti/interlock, posa incoerente o perdita della supervisione richiedono
comunque arresto. Non è garantito che un arresto sulla soglia sia sempre evitabile.

## Terminali

Nessuno dei comandi seguenti è stato eseguito sul robot durante lo sviluppo.
Servono il robot fermo, l'operatore presente e le consuete condizioni della prova.

### Host: connessione nativa

```bash
cd /home/gdj/workspaces/tiago_ws/cognitive_robot_abstract_machine
bash scripts/connect_robocat_planner.sh
```

Il socket esistente viene riusato per un piccolo client Nav2 eseguito in Humble
via SSH, con il sorgente trasmesso alla connessione. Non si copiano credenziali,
non si installano file sul robot e non si inoltrano azioni Nav2 attraverso DDS.
Il robot deve essere quello autenticato dall'alias `robocat`.

### Host: bridge

Fermare prima il bridge precedente, poi:

```bash
bash scripts/launch_robocat_dds_router.sh --control --command-topic /snm_vel
```

L'ingresso `snm` del mux deve essere già configurato come descritto in
`robocat_autonomous_entry.md`. Questo sviluppo non modifica il robot per
installarlo: la verifica nativa blocca la richiesta se manca.

### Container, terminale server

Fermare il precedente server/launcher Giskard. Avviare soltanto il bringup:
il client persistente `robocat-goal` non serve per questa sequenza.

```bash
cd /workspace/cram
source /opt/ros/overlay_ws/install/setup.bash
export ROS_DOMAIN_ID=103
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/cram/doc/robocat_cyclonedds.xml
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
/opt/ros/cram-env/bin/python -u scripts/robocat_giskard_bringup.py \
  --max-linear 0.3 --max-angular 0.2 \
  --ros-args -r /cmd_vel:=/snm_vel \
  -r /direct_laser_odometry/odom:=/mobile_base_controller/odom
```

I limiti sono massimi, non una velocità garantita sulla soglia. La scelta
di 0,3 m/s riprende il precedente test manuale di ingresso Robocat;
può essere abbassata per prove fuori dalla soglia.
Riavviare il server aggiornato: espone i due parametri di verifica del feedback
e del topic comando e accetta la priorità mux 20 durante il proprio goal.

### Container, terminale richiesta: prima solo controllo

```bash
cd /workspace/cram
bash scripts/robocat-entry
```

Senza `--execute` non viene inviato alcun goal Nav2/Giskard. Vengono stampati
`MAP FINGERPRINT`, posa e staging. Annotare il fingerprint della mappa verificata
e controllare il risultato `CHECK OK`. Nessun parametro cinematico viene cambiato.

### Prima prova: arrivo Nav2 e verifica handover, senza movimento Giskard

Con infrastruttura già pronta e localizzazione controllata in RViz, dalla
finestra SNM usare il fingerprint della mappa stampato dal preflight:

```bash
bash scripts/robocat-entry --handover-only --execute --map-fingerprint SHA
```

Sostituire `SHA` con il valore effettivo. Il comando muove il robot tramite
Nav2 verso lo staging salvato, quindi richiede `SUCCEEDED`, stato nativo
aggiornato e idle, routing verificato, mux libero, ruote ferme e posa raggiunta
entro 2 cm / 5 gradi dal waypoint nella stessa mappa. Ricontrolla lo stato
nativo e gli interlock dopo la verifica della posa. Termina con:

```text
HANDOVER READY: Nav2 succeeded; navigation idle; mux free; base stopped; staging pose verified; no Giskard goal sent
```

La verifica finale stampa `STAGING ERROR` con errore longitudinale, laterale,
distanza e yaw rispetto al waypoint, insieme ai limiti. Un rifiuto distingue
allineamento, posa mancante/scaduta/non sincronizzata, interlock, arresto e mappa.
Il `SUCCEEDED` nativo non implica che Nav2 usi gli stessi limiti: controllare
`general_goal_checker.xy_goal_tolerance` e
`general_goal_checker.yaw_goal_tolerance` su `/controller_server`.
Il 13 novembre 2026 le letture erano rispettivamente 0,2 m e 0,15 rad; non
sono state cambiate durante questa verifica. Un residuo misurato da fermo di
circa 2,2 cm è quindi compatibile con successo Nav2 e rifiuto dell'handover a
2 cm. Questi parametri vanno concordati prima di una prova di arrivo più precisa.

Non costruisce né invia il task d'ingresso. Non certifica porta aperta o
corridoio libero dentro la cabina: questi controlli appartengono alla successiva
richiesta di ingresso. Non richiede perciò `--doorway-distance` o
`--doors-held-open`. `--entry-only` è incompatibile con questa modalità.

Senza `--execute` resta un preflight senza movimento, non un handover fisico.
`--confirm-navigation-idle` resta vietato con `--execute`: la sola conferma
manuale del preflight non autorizza una navigazione quando manca lo stato
nativo. Senza l'autorizzazione di bootstrap descritta sotto, non viene inviato
alcun goal e l'attesa resta bloccante.
Non occorre riavviare Giskard, DDS o il controller. Il test deve essere eseguito
dall'operatore con percorso libero e arresto d'emergenza accessibile; `Ctrl+C`
richiede la cancellazione del goal Nav2 posseduto. Verificare l'arresto fisico.

### Primo goal Nav2 senza uno stato iniziale pubblicato

Il bootstrap è un'autorizzazione esplicita all'avvio controllato, non una
certificazione automatica dell'assenza di altri goal. Prima di usarlo,
l'operatore verifica che non siano attive altre richieste native. I server
action ROS pubblicano gli stati dei goal accettati; il loro protocollo standard
non offre un servizio di elenco dei goal attivi
([ROS 2 action design](https://design.ros2.org/articles/actions.html)).

```bash
bash scripts/robocat-entry --handover-only --execute --bootstrap-navigation \
  --map-fingerprint SHA
```

Il flag è distinto da `--confirm-navigation-idle`, richiede `--execute` e non
è consentito con `--entry-only`. Il supervisore autorizza l'invio soltanto
con interlock validi, mux libero e ruote ferme. Il permesso viene trasmesso
con gli heartbeat, revocato quando quelle condizioni non valgono e scade sul
client nativo dopo 0,5 secondi senza rinnovo. Server action disponibile,
routing verificato e watchdog continuano a essere obbligatori. Un goal attivo
osservato impedisce l'invio; non vengono cancellati o sostituiti deliberatamente
goal altrui.

La mancanza di stato non permette però di escludere via API un goal non
osservato: un'eventuale richiesta concorrente resta un limite di questa modalità,
anche con base ferma e mux libero. Il log lo dichiara con `BOOTSTRAP AUTHORIZED`
e `native idle status is not certified`.

Questa eccezione autorizza soltanto il primo goal Nav2. Non pubblica uno stato
idle fittizio e non cambia le condizioni del passaggio a Giskard: servono
risultato Nav2 effettivo, stato nativo aggiornato e idle, base ferma, mux libero
e posa allo staging entro 2 cm / 5 gradi. Se lo stato non arriva nemmeno dopo
l'avvio, l'handover non viene certificato. Senza `--bootstrap-navigation`
resta il comportamento rigoroso precedente.

### Successivamente: solo ingresso dallo staging già raggiunto

Sostituire `D` con la misura reale e `SHA` con il fingerprint stampato.
Verificare anche che il target a 1,5 m sia dentro la cabina e lasci spazio
al robot; porta mantenuta aperta, zona libera e teleop disattivata.

```bash
bash scripts/robocat-entry --entry-only --execute \
  --doorway-distance D --distance 1.5 \
  --map-fingerprint SHA --doors-held-open
```

### Poi: sequenza completa da una posa vicina in mappa

```bash
bash scripts/robocat-entry --execute \
  --doorway-distance D --distance 1.5 \
  --map-fingerprint SHA --doors-held-open
```

La richiesta avvia Nav2 verso lo staging, poi attende la porta aperta senza
una seconda conferma. `Ctrl+C` interrompe la richiesta; dopo cancellazione o
errore non viene inviata una ripartenza. I limiti configurabili sono
`--navigation-timeout` (180 s), `--wait-timeout` (60 s), `--motion-timeout` (60 s).
Non tenere altri client che possano inviare goal Nav2/Giskard durante la prova.

Gli eventi con timestamp sono stampati e pubblicati su `/snm/entry_state` nel
dominio locale 103. Nessun rosbag viene avviato automaticamente; per conservare
l'output si può aggiungere `2>&1 | tee /tmp/robocat-entry.log` al comando.

## Lettura dei blocchi

- Configurazione Giskard mancante: riavviare il bringup aggiornato.
- Feedback/topic errato: il server deve usare wheel e `/snm_vel`.
- Controllo nativo fallito: leggere stderr; verificare mux separato e
  moltiplicatori `2.0 / 2.0 / 1.22` già applicati al controller.
- Attesa di native status: il supervisore non considera sicura un'azione
  Nav2 di cui non riceve lo stato. Il bootstrap esplicito permette soltanto
  il primo invio alle condizioni sopra; non inventa uno stato idle per l'handover.
- Staging/TF non coerenti: verificare localizzazione e waypoint in RViz.
- Porta/passaggio sconosciuti: verificare timestamp, visibilità e geometria;
  non bypassare il blocco interpretando dati mancanti come apertura.
- Mancato arresto confermato: verificare fisicamente il robot prima di una
  nuova richiesta. Il software non rilascia i lock né resetta i driver.

### Handover-only trial with PAL's original arrival tolerance

For an operator-run diagnostic trial, keep Nav2 `general_goal_checker` at
`xy_goal_tolerance=0.20` m and `yaw_goal_tolerance=0.15` rad. After infrastructure
startup, localization verification and a successful read-only check, run from
the SNM/container terminal:

```bash
bash scripts/robocat-entry \
  --handover-only --execute --bootstrap-navigation \
  --handover-position-tolerance 0.25 \
  --handover-orientation-tolerance-deg 10 \
  --map-fingerprint 3ba2bccaa15358b35cd0539ba59ddc991c5a8f416e6279e5df5973440b35f0ae
```

The explicit 25 cm / 10 degree bounds apply only to the SNM staging acceptance
check in `--handover-only`. They provide margin around the native 20 cm / 0.15 rad
arrival bounds for this diagnostic comparison. They do not modify Nav2 parameters
or Giskard motion tolerances. Without these options, staging acceptance remains
2 cm / 5 degrees. The CLI rejects their use for elevator-entry execution.

Expected success: `HANDOVER READY`, with native navigation succeeded and idle,
free mux, stopped wheels and all original interlocks satisfied. No Giskard motion
goal or elevator crossing is submitted. This verifies the handover conditions,
not entry precision or a resolution of the recurring driver fault.

### Configurable staging acceptance before actual entry

The actual entry modes (`--entry-only`, or Nav2 followed by entry) accept:

```text
--staging-position-tolerance METRES
--staging-orientation-tolerance-deg DEGREES
```

For the 2026-11-13 operator trial, the proposed explicit acceptance is 0.10 m /
10 degrees. The current measured staging residual is approximately 5.6–5.8 cm;
a 5 cm bound would still reject it. These options affect only the initial staging
check. The Giskard inside goal and independent completion check retain 2 cm / 5
degrees. Defaults remain the original values unless the options are supplied.
They do not disable map checks, passage visibility, interlocks or cancellation.
The older `--handover-...` options remain restricted to `--handover-only` trials.

Closed/open scans acquired with synchronized map TF on 2026-11-13 place the closed
panels approximately 0.82–0.86 m along staging X; the open scan sees the back wall
at approximately 2.45 m. Raw measurements are saved under
`/home/gdj/workspaces/robocat_experiments/elevator_entry_20261113/`.

The nominal 0.60 m corridor touches jamb returns near staging Y=-0.30 m. Do not
send an entry request solely from this provisional distance estimate. Confirm
corridor centering/clearance and the inside target against the measured opening
before the operator-run entry. No corridor clearance was reduced automatically.

### Recentered staging and explicit isolated-ray mode (2026-11-13)

`labora_demo/labora_floor/elevator_staging` and `doc/robocat_elevator_staging.yaml`
were moved 0.10 m along positive local Y, retaining the original orientation.
New map coordinates: X=21.69610789414912, Y=-31.945390716277654.
Original pose backups exist in the robot mapping directory and the host
`robocat_experiments/elevator_entry_20261113/` folder. No base motion accompanied
this waypoint update.

The operator-authorized `--allow-isolated-scan-gaps` mode permits only an interior
single invalid ray bounded on both sides by finite clear measurements, with
bounding intersections no more than 0.01 m apart on the checked section. A
measured obstacle still vetoes the section. Consecutive invalid rays, missing
edge measurements or larger gaps remain unknown; all valid measurements and
coverage checks still apply. The mode is off by default. This is a lidar
sampling assumption, not a guarantee against sub-centimetre obstacles.

A first *operator-run* entry request from the currently reached pose can use:

```bash
bash scripts/robocat-entry --entry-only --execute \
  --doorway-distance 0.86 --distance 1.30 \
  --staging-position-tolerance 0.10 \
  --staging-orientation-tolerance-deg 10 \
  --doors-held-open \
  --map-fingerprint 14cae3e62b38495028d71f384a7bea50e41025b26b866dfbcf6107edb8961fda
```

The 1.30 m target is measured from the new staging reference, not the reached
pose. With the configured 0.30 m rear extent it places the rear nominally 0.14 m
beyond the 0.86 m threshold estimate, retaining the configured 0.10 m margin.
Confirm actual chassis/accessories and physical door hold before the test.
The final goal remains 2 cm / 5 degrees. The command may wait and time out without
moving if current scans contain larger/consecutive gaps or lack edge coverage.
Recorded samples show clear doorway evidence after recentering, but near-robot
sections still include unknown coverage. Do not bypass those remaining blockers
or assume this trial is guaranteed to initiate motion. Ctrl+C requests owned-goal
cancellation; no automatic restart is sent.

### Transient map/wheel synchronization after Nav2 completion

The staging check now retries a missing or unsynchronized map pose for up to the
existing 0.5 s feedback interval, while the base remains settled and interlocks
valid. It still requires the original 2 s maximum TF age and 0.25 s wheel/TF
skew. A persistent failure reports the measured TF age and wheel/TF skew instead
of only the generic pose-unavailable message. No stale pose is substituted and
no goal is submitted during the wait. This handles a transient missed sample;
it does not repair persistent TF outages.

A passive sample after the operator's failed entry measured 1,331 lookups, with
1,043 within the 0.25 s bound. TF age ranged from about 0.01 to 0.99 s and
wheel/TF skew from about 0.08 to 1.08 s. These are post-failure observations,
not proof of the exact values at the failed handover. The clock offset between
robot wheel stamps and local receipt was also approximately 0.06–0.09 s.

### Correct PAL no-return interpretation (2026-11-13)

The normal Robocat entry observer now interprets positive infinity as a no-return
measurement up to min(25 m, scan.range_max), matching the verified active source
and range filter. No lidar or PAL filter parameters were modified. Finite
out-of-range angular-mask values (31 m), NaN and insufficient coverage remain
unknown; negative infinity and measured obstacles block. The optional
`--allow-isolated-scan-gaps` flag is unnecessary for this correction and should
be omitted for the next baseline test.

A paired raw/filtered bag showed that all observed +Inf outputs already existed
in the raw driver stream, while masked angular sectors were marked 31 m. New
real scans, preserving distinct special values, passed the actual doorway and
passage tests in 9/9 open-door samples. All 10 closed-door samples blocked.
These replace the earlier recorded-sample conclusion that near-robot no-return
rays alone made the corridor unknown. Invalid measurements remain blocking.
Source evidence and results:
`/home/gdj/workspaces/robocat_experiments/lidar_perception_20261113/validation.md`.

Only a fresh SNM client is required; neither Giskard nor the lidar needs a restart.
This validates fixed-robot perception on these samples, not an executed entry.
The latest estimated position was approximately 0.29 m from recentered staging,
so staging alignment must be checked independently before an entry-only trial.

### Scan and wheel timestamp association (2026-11-16)

SNM retains at most 200 wheel samples and associates each captured scan with the
closest measured wheel timestamp, within the existing 0.25 s matching bound.
It no longer rejects a fresh scan merely because the latest wheel sample is
more recent. No pose extrapolation is performed. The current model pose and the
matched acquisition pose compensate the scan geometry for base motion; one
captured scan is used for both timestamp validation and clearance evaluation.

Scan age and current wheel age are checked independently against 0.5 s during
entry. Missing timestamp matches, stale data, motor faults and operator
interlocks still cancel the request. Logs distinguish stale scans, stale wheel
feedback and missing historical matches. Goal tolerances and motion timeouts
are unchanged. Restart the SNM request to load the change; the Giskard server
requires no restart.

A stationary passive check received 170 scans and found a historical match for
all 170: median absolute timestamp difference 5.6 ms, maximum 86.4 ms (including
subscription startup). This verifies association on live streams, not an elevator
entry under load; a new operator-run motion trial remains necessary.

### Native PassDoorAction comparison

Add `--pass-door-action` to select the original CRAM `PassDoorAction` plan.
Without the flag, the existing single straight-motion entry remains the baseline.
Nav2 still reaches staging and SNM still requires stable open-door and clear-path
observations before submitting the native action.

The native plan first refines the staging pose, then waits on its original
`PassageClear` monitor and crosses to the configured inside target. This extra
staging correction is part of the comparison: the native action does not skip its
approach phase just because Nav2 already returned success. Both motion stages use
the existing Robocat Cartesian straight-position/orientation mapping with the
same 2 cm / 5 degree final thresholds. Wheel feedback, speed limits and the
configured inside target remain unchanged.

`CalibratedDoorPassage` supplies the calibrated doorway plane to the native plan
and expresses outgoing goals/monitor points in the server's existing world frame.
Its client-side door/entry-way annotations do not add collision geometry or model
unknown door thickness. It does not infer the gap, cabin dimensions or a new inside
target. The original framework action and monitor implementations are unchanged.

Once submitted, the native action owns the latched doorway monitor; SNM no longer
cancels that action solely because its redundant door-plane observation becomes
false or unknown during crossing. Continuous remaining-path clearance, sensor
freshness, model agreement, navigation ownership, operator locks and driver-fault
checks remain active. This supervised mode still requires doors held open; it is
not a door-closing detection or recovery capability. Final `ENTRY FINAL` geometric
checks remain unchanged and can still reject an otherwise successful motion.

From the SNM container terminal, with the same operator-confirmed conditions:

```bash
bash scripts/robocat-entry --execute --bootstrap-navigation \
  --pass-door-action \
  --doorway-distance 0.86 --distance 1.30 \
  --staging-position-tolerance 0.10 \
  --staging-orientation-tolerance-deg 10 \
  --doors-held-open \
  --map-fingerprint 14cae3e62b38495028d71f384a7bea50e41025b26b866dfbcf6107edb8961fda
```

Restart only the SNM request to load this mode; infrastructure restart is not
required. Compare staging corrections, entry duration, visible trajectory and
`ENTRY FINAL` results. No new hardware trial was executed during implementation.

### Comparison step 1: retain the verified opening only

Use `--retain-door-clearance` to isolate the retained-opening behavior described
by native `PassDoorAction`. This step adopts that behavior in the current SNM
policy; it does not run `PassDoorAction` or replace `SegmentVisibility` with
`PassageClear` yet. These are separate later comparisons.

The initial stable-opening requirement (three distinct scans and one second),
perception, single Cartesian straight motion, target, speed limits and final
acceptance are unchanged. After stable authorization, a false or unknown
observation of the door plane alone no longer cancels entry. Remaining-path
clearance and all feedback/operator/driver protections stay live. It remains a
supervised doors-held-open trial, not a detector of physical door closing.

No flag selects the original baseline. `--pass-door-action` selects the broader
native action experiment including its additional approach correction. The two
comparison flags cannot be combined.

For step 1, add only `--retain-door-clearance` to the validated baseline command:

```bash
bash scripts/robocat-entry --execute --bootstrap-navigation \
  --retain-door-clearance \
  --doorway-distance 0.86 --distance 1.30 \
  --staging-position-tolerance 0.10 \
  --staging-orientation-tolerance-deg 10 \
  --doors-held-open \
  --map-fingerprint 14cae3e62b38495028d71f384a7bea50e41025b26b866dfbcf6107edb8961fda
```

The initial `ENTRY MODE` line identifies retained-opening mode. Compare premature
cancellation, motion onset delay, trajectory and `ENTRY FINAL` residuals under
the same initial conditions. Restart only the SNM request to load this change.

### Laser versus wheel feedback comparison

Both persistent infrastructure and the SNM request accept `--feedback wheel|laser`.
The default remains `wheel`. The selector is shared with the waypoint client.
SNM checks the actual Giskard feedback and command parameters and refuses a
request if they do not match the selected source and `/snm_vel` routing.

With laser selected, map alignment, inside target, frozen doorway, acquisition-time
scan association, model agreement and final assessment all use
`/direct_laser_odometry/odom`. Actual `/mobile_base_controller/odom` remains
subscribed independently for stationary-wheel confirmation and safety checks;
its topic is not remapped to the laser. Separate histories are kept for both
pose sources. Entry remains bounded by the existing freshness and geometric
checks; no tolerances or speed limits are relaxed.

With the robot stopped and no active goals, close the temporary test session and
stop infrastructure using the owned shutdown procedure. If cleanup is not
confirmed, resolve that before proceeding; do not delete readiness markers.
Restart on the host with:

```bash
bash scripts/robocat-infra start --feedback laser --max-linear 0.3 --max-angular 0.2
byobu-tmux attach-session -t robocat-infra
```

Complete preparation, check the Giskard READY line reports
`/direct_laser_odometry/odom`, detach with F6 and run `bash scripts/robocat-byobu`
on the host. The automatic SNM read-only check inherits the selected source.
From the container SNM terminal, first run `bash scripts/robocat-entry --feedback laser`
without `--execute`. For the operator-run comparison, keep all baseline settings:

```bash
bash scripts/robocat-entry --execute --bootstrap-navigation \
  --feedback laser --retain-door-clearance \
  --doorway-distance 0.86 --distance 1.30 \
  --staging-position-tolerance 0.10 \
  --staging-orientation-tolerance-deg 10 \
  --doors-held-open \
  --map-fingerprint 14cae3e62b38495028d71f384a7bea50e41025b26b866dfbcf6107edb8961fda
```

The client prints `FEEDBACK: laser (/direct_laser_odometry/odom)` before running.
For the wheel comparison, restart infrastructure with `--feedback wheel` and use
`--feedback wheel` in the same request. Nav2 remains native and is not reconfigured
by this selector. No hardware trial was executed while implementing this option.

### Recording a model/feedback disagreement

In a second CRAM container terminal, start `bash scripts/robocat-record-entry`
before launching the same entry request. The helper sources the overlay, selects
domain 103/CycloneDDS and records a fixed topic list in SQLite format under
`/workspace/cram/tmp/robocat-entry-bags/entry-YYYYMMDD-HHMMSS`. An optional first
argument chooses another output directory. It records no cameras and does not
record all topics. It does not send motion goals.

The new `/snm/pose_comparison` topic is a `diagnostic_msgs/DiagnosticArray` sampled
in the existing SNM observation loop, including observations that cause a stop.
Its header is the comparison time in the common model/feedback frame. Key/value
fields include `model.x/y/yaw`, `feedback.x/y/yaw`, `geometry_target.x/y/yaw`,
`submitted_target.x/y/yaw`, source, original feedback and scan timestamps, ages,
position and yaw errors and the actual disagreement limits. Model/feedback
limits remain 0.03 m / 0.05 rad. Serialization/publication is skipped when nobody
subscribes; no timer, sensor subscription or control loop is added.

The native world synchronization protocol does not carry an original sensor
acquisition timestamp for the model pose. Therefore `model_observed_at` and the
header describe when SNM sampled the model, not when Giskard originally acquired
its odometry; `model_source_stamp_available=False` makes this explicit.

Start the recorder first, launch the operator-authorized trial from the SNM
terminal, then leave recording active for about five seconds after the robot
stops and stop only the recorder with Ctrl+C. Wait for rosbag to finalize its
metadata before replaying or copying the directory. Topics appearing only when
SNM starts are discovered automatically.

The bag includes map, TF/static TF, description and joints for RViz, both
odometries, scan, `/snm_vel`, SNM state/comparisons, action status, ROS log and
forwarded safety/mux diagnostics. The current DDS bridge does not forward the
native mux output `/mobile_base_controller/cmd_vel`; it is not falsely included
as an available local stream. A separate native recording would be needed to
study that output specifically. For this discrepancy investigation, the poses
actually compared and the supervisor's decision are the primary evidence.

On the host, a default recording is under
`/home/gdj/workspaces/tiago_ws/cognitive_robot_abstract_machine/tmp/robocat-entry-bags/`.
No infrastructure or Giskard restart is needed; the next SNM invocation loads the
new diagnostic.

### Comparing native Giskard completion with SNM acceptance

Restart the Giskard server once after installing this diagnostic update, before
starting a new test. Its startup prints
`DIAGNOSTICS: /giskard/goal_diagnostics (native decision snapshots)`.
The same `bash scripts/robocat-record-entry` command now includes that topic.
No new CLI flag is needed; the diagnostic remains enabled with `--no-tracking`.

`/giskard/goal_diagnostics` is a reliable, transient-local
`diagnostic_msgs/DiagnosticArray`, retaining the latest 100 events. A native
state-history observer captures every changed state synchronously, after task
observations and lifecycle updates and before the executor integrates its next
command. It is not a timer sampling a later predicted world pose. Events contain
server ROS decision time, integer goal ID matching the logs, control cycle,
root/tip and goal-reference frames, task class, observation/lifecycle states,
compiled error and threshold, numeric bound target and current pose. Position
errors are in metres; orientation errors and yaw are in radians. The server's
applied odometry topic (after remapping), frame, child frame, acquisition stamp,
position and quaternion are included independently of the diagnostic header
timestamp. `feedback_sample_is_applied=True` identifies the exact input sample
written into the server model, rather than a later received sample.

Use events with `stage=state_change_before_qp` and `end_motion=True` to identify
the native completion decision. `compiled_initial_state` marks the chart before
its first real control cycle; `command_stop` is a later phase after the native
stop call, and must not be confused with the pre-command decision. Each task
has its own status row; the orientation task carries target yaw, the position
task carries target XYZ. The event is published even with no current subscribers;
the topic can therefore be inspected after completion as well as recorded live.
The snapshot adds no goals, timers or changes to task constraints/tolerances.

The SNM pose-comparison diagnostic now also carries
`assessment.position_error`, `assessment.yaw_error`,
`assessment.rear_clearance`, `assessment.position_tolerance` and
`assessment.orientation_tolerance`. Rear clearance is the residual *after* the
required margin; a negative value fails the full-entry condition. These are
computed from the same selected odometry sample used by SNM. Only healthy
observations feed the final acceptance decision.

Start the bag before the entry request; leave it recording until about five
seconds after the robot stops, then Ctrl+C the recorder. Preview markers are
also recorded when available. Check that the completed bag lists
`/giskard/goal_diagnostics` and `/snm/pose_comparison`; a recorder cannot obtain
server diagnostics from an older Giskard process that has not been restarted.
The bag supports comparisons of the software's poses and decisions; it does not
provide independent physical ground truth for odometry accuracy.
