# Collegamento TIAGo — 27 ottobre 2026

## Ambiente

- Robot: `tiago-72c`, singolo braccio; Ubuntu 22.04, ROS 2 Humble.
- SSH via Ethernet: `pal@10.68.0.1`.
- Framework: container `aicor-cram-demo`, ROS 2 Jazzy, rete Docker `host`.
- Repository montato nel container in `/workspace/cram`.
- ROS 1 Noetic del PC non utilizzato.
- Password escluse da questo documento.

## Accesso SSH

La chiave salvata per `10.68.0.1` era diversa da quella ricevuta.
Eseguiti sul PC:

```bash
cp ~/.ssh/known_hosts ~/.ssh/known_hosts.backup
ssh-keygen -R 10.68.0.1
ssh -o StrictHostKeyChecking=ask pal@10.68.0.1
```

Accettata la chiave ECDSA:
`SHA256:Ol6rPMHj7h8yFHFJ6x11T6Bkp3kHIvmaaKUoD7Woo9o`.
Non verificata tramite un canale indipendente. Non è noto se la vecchia
chiave appartenesse a un altro robot o a una precedente installazione.
Il comando `ssh-keygen -lf /etc/ssh/ssh_host_ecdsa_key.pub`, inizialmente
eseguito su `bolbait`, mostrava la chiave del PC, non del TIAGo.

## Comunicazione ROS 2

Il robot usa dominio `72` e Cyclone DDS; multicast disabilitato,
peer configurato sul robot: `localhost`.
Dal container, Fast DDS non rilevava i topic nemmeno con dominio `72`
e peer esplicito.

Installato nel container, con autorizzazione:

```bash
apt-get update
apt-get install -y --no-install-recommends ros-jazzy-rmw-cyclonedds-cpp
```

Creata la configurazione [tiago_cyclonedds.xml](tiago_cyclonedds.xml),
inizialmente copiata in `/tmp/tiago-cyclonedds.xml` nel container.
Usa l'interfaccia Ethernet `enx00e04c0902b0`, peer `10.68.0.1`,
multicast disabilitato e ricerca degli indici partecipante fino a `500`.
Il limite e la dimensione messaggi `1400B` riprendono la configurazione del robot.

Per una shell del container:

```bash
source /opt/ros/overlay_ws/install/setup.bash
export ROS_DOMAIN_ID=72
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/cram/doc/tiago_cyclonedds.xml
```

Le variabili valgono per la shell e i suoi processi figli.
La dipendenza è installata nel container corrente; non ancora nella sua immagine.
La configurazione XML è nel repository montato, quindi non dipende da `/tmp`.

## Verifiche in sola lettura

| Dato | Topic / frame | Esito |
|---|---|---|
| Lidar | `/scan`, `base_laser_link` | Misure ricevute; circa 15 Hz sul robot |
| Odometria ruote | `/mobile_base_controller/odom` | Circa 50 Hz sul robot; velocità praticamente nulla |
| Odometria laser | `/direct_laser_odometry/odom` | Posa coerente con TF `odom` → `base_footprint` |
| Giunti | `/joint_states` | 14 posizioni numeriche; circa 50 Hz sul robot |
| TF | `map` → `base_footprint` | Disponibile |
| TF lidar | `base_footprint` → `base_laser_link` | Disponibile; traslazione circa `[0.202, 0, 0.095]` m |

Ricezione dei campioni verificata anche direttamente dal container.
Il braccio espone `arm_1_joint` … `arm_7_joint`.
Alcuni valori di velocità e sforzo dei giunti sono `NaN`.
L'odometria delle ruote differisce da quella laser;
`/mobile_base_controller` ha `enable_odom_tf=False`.

## Internet e sincronizzazione

Chrony era attivo ma non sincronizzato: nessuna sorgente utilizzabile,
nessuna route Internet e nessun DNS sulla connessione Ethernet.
Osservato uno scarto di circa 54 secondi rispetto al container.
La data 2081 del messaggio di benvenuto SSH non corrispondeva all'orologio corrente.

Collegato il robot al Wi-Fi `LabORA`, inserendo la password al prompt:

```bash
sudo nmcli --ask device wifi connect LabORA ifname wlan0
```

- Wi-Fi assegnato: `192.168.68.63/22` (DHCP); gateway `192.168.68.1`.
- Profilo salvato con riconnessione automatica.
- Ethernet rimasta su `10.68.0.1/24` per PC–robot.
- DNS e HTTPS verificati.
- Chrony si è sincronizzato automaticamente: `Leap status: Normal`,
  `NTPSynchronized=yes`; nessuna modifica manuale all'orologio.
- Lidar, giunti e TF ricontrollati dal container con timestamp coerenti.

## Stato dell'adattamento

Il modello esistente `semantic_digital_twin.robots.tiago.Tiago` descrive
un TIAGo++ a due bracci e resta invariato.
La nuova variante `semantic_digital_twin.robots.tiago_single_arm.TiagoSingleArm`
descrive il robot acquisito: base differenziale, torso, un braccio,
pinza PAL a due dita e testa.
Acquisiti via SSH i parametri `/robot_state_publisher:robot_description`
e `/move_group:robot_description_semantic`, esportati con
`ros2 param get --hide-type` e copiati nel container:

- `/tmp/tiago-72c.urdf`: XML valido, 54 link e 53 giunti.
- `/tmp/tiago-72c.srdf`: XML valido, 1278 esclusioni di collisione;
  gruppi `arm`, `arm_torso`, `gripper`.

Copie conservate nel repository in
`semantic_digital_twin/resources/robots/tiago_single_arm/`:

- `robot.urdf`: descrizione acquisita, invariata.
- `original.srdf`: descrizione semantica acquisita, invariata.
- `collision.srdf`: copia per il framework, con la pulizia autorizzata descritta sotto.

La lettura diretta dei due topic dal container è scaduta senza ricevere dati;
l'esportazione dei parametri via SSH è riuscita.
Le mesh richiedono `pal_gripper_description`, `pal_urdf_utils`,
`pmb2_description`, `tiago_description`.
Installati da sorgenti ufficiali PAL Robotics, con autorizzazione:

| Pacchetto | Provenienza | Versione | Commit |
|---|---|---|---|
| `pal_urdf_utils` | [pal-robotics/pal_urdf_utils](https://github.com/pal-robotics/pal_urdf_utils), ramo `humble-devel` | `2.11.4` | `e8600a339a3a9b7ffe67d1030190b97b67fb577b` |
| `tiago_description` | [pal-robotics/tiago_robot](https://github.com/pal-robotics/tiago_robot/tree/4.19.4/tiago_description), tag `4.19.4` | `4.19.4` | `5255eba8a74b7d9d6e5dab6d6e2b4c2fca886c97` |

Sorgenti in `/opt/ros/overlay_ws/src/pal_urdf_utils` e
`/opt/ros/overlay_ws/src/tiago_robot` nel container.
Build limitata ai due pacchetti:

```bash
source /opt/ros/overlay_ws/install/setup.bash
cd /opt/ros/overlay_ws
colcon build --merge-install --executor sequential \
  --packages-select pal_urdf_utils tiago_description \
  --cmake-args -DBUILD_TESTING=OFF
```

`tiago_description` è allineato alla versione `4.19.4` letta sul robot:
la versione pubblica più recente, inizialmente provata, non conteneva
sei mesh di collisione richieste dal suo URDF.
Non sono state sostituite con geometrie diverse né copiate mesh dal robot.
Verifica finale: entrambi i pacchetti risolti da ament e tutte le
34 mesh uniche referenziate dall'URDF presenti.
Non avviati launch, controllori o simulatori.
I pacchetti sono installati nel container corrente, non ancora nella sua immagine.

## Modello offline

- Il modello conserva i limiti dei giunti presenti nell'URDF.
- Tutte le interfacce hardware restano disabilitate; nessuna postura predefinita aggiunta.
- Sorgenti della base e del lidar inizialmente simulate.
- Testa e lidar riusano le classi TIAGo esistenti; i loro parametri
  ottici e di scansione non sono ancora calibrati sul robot.
- Abilitate nel modello le regole di anticollisione esterna e autocollisione,
  con i valori predefiniti del framework; non ancora validate sul reale.
- L'SRDF originale riferisce due frame assenti nell'URDF:
  `head_front_camera_rgb_frame` e `head_front_camera_rgb_optical_frame`.
  Nella copia `collision.srdf` sono eliminate soltanto le 103 esclusioni
  che li coinvolgono; restano le altre 1175. Nessuna rinomina dei frame.
- Nessuna modifica all'URDF, all'SRDF o ai controllori sul robot.

Installato nel Python del container `docformatter 1.7.8`, con autorizzazione,
per eseguire `scripts/format_docstrings.py` sui nuovi sorgenti e test.
Corretta con autorizzazione un'asserzione del nuovo test: il framework
mantiene proprie regole di esclusione oltre alla matrice SRDF.

Verifica eseguita nel container, dopo controllo della memoria libera,
con limite di memoria virtuale a 5 GiB:

```bash
source /opt/ros/overlay_ws/install/setup.bash
ulimit -v 5242880
/opt/ros/cram-env/bin/python -m pytest \
  test/semantic_digital_twin_test/test_robots/test_tiago_single_arm.py \
  test/semantic_digital_twin_test/test_robots/test_robot_part_mixin_validation.py \
  --orm-build never -q
```

Esito: 24 test passati, inclusi 6 nuovi test del modello.
Il parser segnala tag PAL/Gazebo non riconosciuti nell'URDF acquisito;
il caricamento della geometria e delle parti robotiche è riuscito.
I test del modello completo richiedono i pacchetti descrittivi PAL installati;
sono saltati negli ambienti che non li hanno. Nessun test usa il robot live.

La sincronizzazione continua è descritta sotto. Restano la calibrazione
fisica dei sensori, la verifica dei limiti operativi e l'interfaccia di comando della base.
La base espone `/cmd_vel` (`Twist`) e
`/mobile_base_controller/cmd_vel` (`TwistStamped`); nessuno dei due è stato comandato.

Nessun comando di movimento inviato. Nessuna prova fisica eseguita.

## Prova di sincronizzazione in sola lettura

Eseguita nel container con i componenti esistenti:
`PendingJointStateSynchronizer`, `SubscribedBasePoseSource`,
`SubscribedLidarSource` e `WorldStateInputs`.

- Aggiunto al mondo temporaneo il frame `odom` e un collegamento differenziale
  alla base, senza abilitarne l'interfaccia hardware.
- Applicate le posizioni dei 14 giunti: errore massimo rispetto al messaggio
  ricevuto `0.0`.
- Applicata l'odometria di `/direct_laser_odometry/odom`.
- Ricevuti 569 raggi lidar. Il pattern ricevuto accompagna la lettura;
  non dipende dal pattern simulato ereditato dal vecchio modello.
- Interfacce hardware abilitate dopo la sincronizzazione: zero.

Parametri pubblicati dai sensori:

| Parametro | Valore |
|---|---|
| Lidar: angolo minimo / massimo | `-1.6523163318634033` / `1.6518452167510986` rad |
| Lidar: incremento angolare | `0.00581718236207962` rad |
| Lidar: intervallo distanza | circa `0.05`–`10.0` m |
| Camera: topic informazioni | `/head_front_camera/rgb/camera_info` |
| Camera: frame effettivo | `head_front_camera_color_optical_frame` |
| Camera: risoluzione | `640 × 480` |
| Camera: `fx`, `fy` | `570.1754289822842` px |
| Camera: `cx`, `cy` | `319.5`, `239.5` px |
| Camera: campo orizzontale / verticale ricavato dagli intrinseci | circa `1.022850` / `0.796825` rad |

Il campo visivo pubblicato differisce dai valori ereditati
(`0.99483` / `0.75049` rad). I parametri ricevuti sono stati applicati
all'istanza temporanea della camera e del lidar; nessuna nuova calibrazione
fisica e nessuna modifica dei valori predefiniti della classe.

## Visualizzazione

Visualizzatore del modello disponibile, mentre il processo è attivo,
su `http://127.0.0.1:8765/`, servito dal container solo sul loopback.
Il file è `/tmp/tiago-preview/index.html`, generato con le mesh visuali
del mondo del framework tramite trimesh.

La prima vista usava la posa iniziale dell'URDF. È stata poi aggiornata
con una posa campionata dai giunti e dall'odometria reali.
Aggiornare la pagina per vederla. È un'istantanea, non un aggiornamento continuo.
La connessione ROS dell'osservatore è stata chiusa dopo l'acquisizione.

## Monitor continuo in sola lettura

Implementato `experiments.tiago_monitor`, senza nuove dipendenze.
Avviato nel container su **http://127.0.0.1:8766/**.
La porta `8765` resta la precedente istantanea.

- Riceve giunti, odometria laser, lidar, intrinseci camera e TF.
- Aggiorna 36 mesh ogni `0.5` s, mantenendo la vista scelta nel browser.
- Applica all'istanza osservata frame e intrinseci camera e pattern lidar ricevuti.
- Mostra età e stato dei dati: `waiting`, `current`, `stale`, `clock_skew`.
  Soglia configurabile, predefinita `2` s; gli scarti futuri oltre soglia
  sono distinti dai campioni vecchi.
- Segnala l'interruzione del monitor e conserva l'ultima posa visualizzata.
- Confronta la posizione del lidar nel modello e nella TF più recente.
  Il confronto usa gli ultimi campioni disponibili, non tempi esattamente allineati.
- Calcola i contatti solo con giunti e odometria aggiornati, inizializzando
  la matrice di collisione tramite il framework.

Verifica iniziale sul robot fermo: tutti i flussi `current`, scarto di traslazione
lidar circa `3.1e-17` m. Il primo conteggio di zero contatti non era significativo:
una regola standard escludeva tutte le autocollisioni del robot passivo.
La correzione e la verifica effettiva sono descritte sotto.
Il mondo osservato non contiene ambiente o ostacoli reali.

Il nodo ha soltanto i publisher ROS standard `/rosout` e `/parameter_events`,
verificati dal nodo stesso. Nessun publisher di movimento, action client
di movimento o interfaccia hardware abilitata.

Per avviarlo nuovamente, nel container con l'ambiente ROS/DDS configurato
nel paragrafo «Comunicazione ROS 2» e nessuna copia del monitor già attiva:

```bash
/opt/ros/cram-env/bin/python -m experiments.tiago_monitor
```

Opzioni: `--port`, `--directory`, `--maximum-age`, `--period`.
In primo piano si arresta con `Ctrl+C`. Per fermare l'istanza avviata
in background durante questa sessione, dal PC:

```bash
docker exec aicor-cram-demo pkill -f '^/opt/ros/cram-env/bin/python -m experiments.tiago_monitor$'
```

File generati: `/tmp/tiago-monitor/`; log dell'istanza corrente:
`/tmp/tiago-monitor.log`. Nessuna modifica persistente al robot.

Verifica automatica: 12 test passati nel container, usando messaggi simulati,
senza robot live e con memoria limitata a 5 GiB:

```bash
/opt/ros/cram-env/bin/python -m pytest \
  test/experiments_test/test_tiago_monitor.py \
  test/experiments_test/test_tiago_monitor_inputs.py \
  --orm-build never -q
```

Verificati anche il JSON servito, la corrispondenza dei nomi di tutte le
mesh esportate e la sintassi JavaScript. Nessun browser automatico disponibile
nel container per una verifica visiva automatizzata.

## Verifica funzionale del framework

Eseguita nel container, senza viewer e senza comandi al robot.

| Controllo | Esito |
|---|---|
| Creazione robot tramite `RobotSpecification` | Base differenziale simulata; giunti interni disabilitati |
| Solver Giskard sul modello TIAGo reale | Raggiunti due obiettivi simulati: avanzamento e spostamento laterale con orientamento finale |
| Selezione della mappatura Coraplex | Corretta per `TiagoSingleArm` |
| Esecuzione Coraplex → Giskard con anticollisione | Obiettivo simulato completato |
| Ostacolo intersecante la base | Collisione rilevata |
| Autocollisioni nel modello passivo | Coppie previste dall'SRDF effettivamente controllate |

Correzioni necessarie:

- Aggiunta `coraplex.alternative_motion_mappings.tiago_single_arm.SingleArmBaseMotion`.
  Prima Coraplex usava `SetOdometry` per il nuovo robot, impostando direttamente
  la posa in simulazione. Ora usa `DifferentialDriveBaseGoal`.
  La mappatura supporta ora simulazione cinematica e costruzione dei goal reali;
  nessun goal reale è stato eseguito.
- Aggiunta `ObservedRobotCollisionExclusions`: per questo robot le esclusioni
  dipendono dall'SRDF, non dall'assenza di interfacce di comando.
  La regola standard considerava altrimenti tutte le parti come rigidamente
  collegate ed eliminava ogni autocollisione.
  Il comportamento per corpi estranei al robot resta invariato.

Nel modello parsato ci sono 27 corpi con geometria di collisione e
**119 coppie di autocollisione effettivamente controllate**.
Le interfacce hardware restano disabilitate durante l'osservazione.

Verifica aggiornata sulla postura ricevuta dal TIAGo:

- Flussi aggiornati e TF lidar coerente con il modello.
- Contatto fra `gripper_link` e `torso_fixed_column_link`;
  compenetrazione *nel modello* circa `2.2` mm.
- L'utente conferma un contatto braccio/base sul robot reale,
  coerente con questa segnalazione. Non è una misura fisica della profondità.
- Nessuna ulteriore esclusione SRDF aggiunta; nessun movimento inviato.

Verifiche automatiche finali, in invocazioni separate:
8 test del modello, 12 del monitor e 6 di integrazione del framework.
Tutti usano dati locali o messaggi simulati; nessun test comanda il robot.

```bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
ulimit -v 5242880
/opt/ros/cram-env/bin/python -m pytest test/semantic_digital_twin_test/test_robots/test_tiago_single_arm.py --orm-build never -q
/opt/ros/cram-env/bin/python -m pytest test/experiments_test/test_tiago_monitor.py test/experiments_test/test_tiago_monitor_inputs.py --orm-build never -q
/opt/ros/cram-env/bin/python -m pytest test/experiments_test/test_tiago_framework.py --orm-build never -q
```

Una prima esecuzione cumulativa ha avuto un crash nativo durante la creazione
di un nuovo mondo. Non riprodotto nelle esecuzioni finali con thread numerici
limitati; la causa non è stata identificata.

L'interfaccia di comando è ora implementata come descritto sotto;
restano il collaudo fisico di limiti e arresti e la gestione del contatto iniziale.
La demo della porta usa ancora il vecchio `Tiago` e richiede una postura `PARK`
del braccio, non ancora definita per questa variante: non è pronta per il reale.

## Interfaccia di comando della base

Implementata in
`giskardpy.middleware.ros2.scripts.iai_robots.tiago.single_arm`.
Non avviata in modalità hardware e nessuna velocità inviata al robot.

Percorso verificato dal grafo ROS sul TIAGo:

```text
framework → /cmd_vel (Twist) → twist_mux
          → /mobile_base_controller/cmd_vel (TwistStamped)
```

Parametri letti dai servizi dei nodi attivi, senza modificarli:

| Parametro | Valore |
|---|---|
| Timeout controllore base | `0.25` s |
| Timeout ingresso navigazione del mux | `0.5` s |
| Priorità navigazione nel mux | `10` |
| Limiti lineari del controllore | `-0.2` … `1.0` m/s |
| Limite angolare del controllore | `1.05` rad/s |
| Accelerazione lineare / angolare massima | `0.5` m/s² / `1.05` rad/s² |

L'interfaccia conserva il mux e impone limiti iniziali più restrittivi:
**±0.05 m/s** e **±0.1 rad/s**, configurabili con `BaseVelocityLimits`.
Gli stessi limiti sono applicati al solver e alla saturazione dei comandi.

- Comanda soltanto la base; braccio, testa, pinza e torso restano disabilitati.
- Legge giunti, odometria laser, TF e lidar.
- Se giunti o odometria sono assenti o hanno timestamp oltre `2` s dal clock
  locale, invia velocità zero.
- Su output numerico non finito invia zero e solleva `NonFiniteBaseCommand`.
- Riusa lo stop del publisher e del ciclo di controllo Giskard.

I timeout sono configurazioni verificate, non tempi di arresto fisico misurati.
Il mux può selezionare sorgenti a priorità maggiore; il comando zero del
framework non sostituisce l'arresto d'emergenza.

Correzioni aggiuntive necessarie per il cablaggio:

- Riacquisito il robot dal mondo risultante dopo il merge.
- Corretto il controllo iniziale Giskard: una base controllata è sufficiente,
  anche senza giunti interni controllati.
- Corretto il factory predefinito QP di Giskard usando la configurazione
  esistente valida, a `20` Hz.
- Estesa la mappatura Coraplex al goal differenziale anche in modalità reale;
  verificata soltanto la costruzione del piano.

Avvio nel container con l'ambiente ROS/DDS già configurato:

```bash
/opt/ros/cram-env/bin/python -m giskardpy.middleware.ros2.scripts.iai_robots.tiago.single_arm
```

Questo avvia la modalità standalone, senza publisher di comando della base.
La modalità hardware richiede esplicitamente `--hardware`; non è stata avviata.
Il nodo si chiama `giskard`, coerentemente con il client Coraplex predefinito.

Verifiche nel container, con memoria limitata e thread numerici limitati:

- 15 test specifici passati: limiti, stop, feedback assente/vecchio, output
  non finito, cablaggio closed-loop con trasporto fittizio e avvio standalone
  effettivo del server.
- 7 test di integrazione Coraplex–Giskard passati; nessuna esecuzione reale.
- 6 test esistenti del publisher base passati. Durante il teardown sono
  comparsi traceback del thread spinner ROS (`executor` già azzerato);
  problema di chiusura non corretto in questo intervento.

Test specifici:

```bash
/opt/ros/cram-env/bin/python -m pytest test/giskardpy_test/test_ros2_tools/test_tiago_base_interface.py --orm-build never -q
```

Il contatto iniziale pinza–torso resta segnalato e non è stato escluso.
Il prossimo collaudo fisico deve ancora verificare invio, limiti e arresto
della base con operatore presente.

## Priorità joystick verificata sul reale

Osservate in sola lettura le alternanze effettuate dall'operatore:

| `/joy_priority` | Diagnostica mux | Effetto |
|---|---|---|
| `true` | joystick `locked`, priorità corrente `100` | Navigazione `/cmd_vel` bloccata; precedenza alla teleoperazione |
| `false` | joystick `free`, priorità corrente `0` senza comandi | Navigazione consentita; nessuna esecuzione autonoma avviata automaticamente |

Le alternanze sono state rilevate ripetutamente. Ultimo valore osservato:
`false`. Il blocco joystick ha timeout `0`: l'assenza di nuovi messaggi
non dimostra da sola che il blocco sia libero; consultare anche la diagnostica.

La prova fisica della base precedente è stata annullata per feedback non
aggiornato prima di creare il publisher di comando. Nessuna velocità inviata.

## Collegamento dopo cambio porta Ethernet

Rilevata la connessione PC–robot su `enp1s0`, IP PC `10.68.0.136`;
l'adattatore precedente `enx00e04c0902b0` risulta scollegato.
Aggiornato `tiago_cyclonedds.xml` alla porta attuale.
SSH funziona e i giunti sul robot hanno timestamp aggiornati, ma nuove
sottoscrizioni nel container non ricevono i campioni entro il timeout.
La causa residua della mancata comunicazione DDS non è ancora individuata.
La prova richiesta a `0.02` m/s per `0.5` s resta sospesa: nessun movimento
inviato in assenza del feedback necessario.

Feedback successivamente ripristinato con `enp1s0` e riavvio del monitor,
necessario per riaprire DDS sull'interfaccia aggiornata. Verificati pacchetti
DDS di scoperta e dati sulla Ethernet e tutti i flussi del monitor `current`:
giunti, odometria laser, lidar, camera e TF; età osservata circa `0.04`–`0.09` s.
Il sistema vede Internet su Wi-Fi `LabORA`; l'adattatore USB-LAN risulta
`unavailable`, anche se collegato fisicamente. Nessun movimento eseguito.

Nuovo tentativo della singola prova breve: blocchi mux liberi e priorità `0`
verificati prima dell'avvio. Procedura limitata a `25` s, quindi sotto il limite
utente di tre minuti. Annullata dal controllo preliminare dopo l'attesa dei
segnali richiesti, prima di creare il publisher di comando. Il messaggio
aggregato non distingue quale ingresso mancasse: non dimostra che tutti i
flussi del monitor siano nuovamente persi. Nessun comando inviato e nessun
ulteriore tentativo automatico.

## Diagnosi di `missing current feedback`

Il messaggio aggregava due problemi distinti:

- Dopo un ulteriore cambio cavi, il robot è nuovamente raggiunto tramite
  USB-LAN `enx00e04c0902b0` (`10.68.0.135`), mentre `enp1s0` porta Internet.
  DDS era ancora vincolato a `enp1s0`. La configurazione ora seleziona la
  sottorete `10.68.0.0`, indipendentemente dal nome della porta; i processi
  già avviati richiedono riavvio per applicarla.
- Ripristinata la rete, mancava soltanto `/power/is_emergency`: il publisher
  usa durabilità `TRANSIENT_LOCAL`, il test richiedeva `VOLATILE` e perdeva
  lo stato già pubblicato. Aggiunta a `LatestMessageSubscriber` la possibilità
  di scegliere il profilo QoS, mantenendo invariato il valore predefinito.
  La diagnostica separata `/tmp/tiago_feedback_diagnosis.py` nel container
  richiede lo stato persistente. Lo script di movimento precedente non è
  stato modificato: prima di riutilizzarlo occorre applicargli lo stesso QoS.

Verifica reale in sola lettura: controllo `BaseFeedback` valido, giunti con
età `0.020` s, odometria laser `0.087` s, odometria ruote `0.009` s;
`/power/is_emergency = false`. Nessun publisher di movimento creato.
Il dato emergenza è uno stato persistente: la ricezione non ne dimostra una
pubblicazione periodica.

Fonte della selezione per sottorete: [documentazione CycloneDDS](https://cyclonedds.io/docs/cyclonedds/latest/config/network_interfaces.html).
QoS e valori dei segnali verificati direttamente sui topic del robot.

## Prova reale in avanti a velocità ridotta

Eseguita su richiesta dell'operatore tramite `/tmp/tiago_forward_2s.py`
nel container, con QoS emergenza `TRANSIENT_LOCAL`:
comando longitudinale `+0.02` m/s, rotazione nulla, durata `2.025` s,
poi invio ripetuto dello stop. Controlli feedback ed emergenza attivi.

- Spostamento odometria ruote: `0.03775` m.
- Spostamento odometria laser: `0.02723` m.
- Picco velocità longitudinale misurata: `0.02670` m/s.
- Velocità finale: `0.000163` m/s; rotazione finale `-0.00000208` rad/s.
- Arresto rilevato sotto soglia `0.002` m/s dopo circa `0.302` s.

Le due odometrie rilevano uno spostamento; le distanze differiscono di circa
un centimetro. Il picco misurato supera il comando nominale pur rimanendo
sotto la soglia di interruzione della prova (`0.04` m/s).

## Prova reale a 0.1 m/s per due secondi

Eseguita su richiesta dell'operatore con `/tmp/tiago_forward_01_2s.py`.
Limite longitudinale della sola istanza di prova impostato a `0.1` m/s;
default del framework invariato (`0.05` m/s). Comando avanti `+0.1` m/s,
rotazione nulla, durata `2.027` s, quindi stop ripetuto. Controlli feedback
ed emergenza attivi; soglia di interruzione velocità misurata `0.15` m/s.

- Spostamento laser: `0.15715` m; ruote: `0.20514` m.
- Picco velocità longitudinale misurata: `0.11116` m/s.
- Velocità finali longitudinale e angolare praticamente nulle.
- Arresto sotto soglia rilevato dopo circa `0.402` s.

La discrepanza laser–ruote è circa `4.8` cm; queste misure non stabiliscono
la causa della differenza. Il picco misurato supera il comando nominale.

## Ripetizione con riferimento a terra

Operatore pronto, riferimento iniziale segnato e condizioni confermate.
Ripetuta una sola prova `+0.1` m/s per `2.025` s, poi stop:
laser `0.18813` m, ruote `0.20276` m, differenza circa `1.46` cm.
Picco misurato `0.11655` m/s; arresto rilevato dopo `0.402` s,
velocità finali praticamente nulle. Misura fisica dell'operatore ancora
attesa: accuratezza delle due odometrie non ancora determinata.

## Esclusione pinza–torso e primo goal Coraplex reale

L'operatore chiarisce che nella postura attuale pinza e torso non si toccano
sul robot vero e richiede di ignorare questa coppia. Aggiunta soltanto
`gripper_link`–`torso_fixed_column_link` in `operator_exclusions.srdf`,
caricato insieme alle esclusioni PAL. `original.srdf` e `collision.srdf`
restano invariati; gli altri controlli restano attivi. Verifica test-driven:
nuovo test fallisce prima della modifica, poi tutti i 9 test del modello passano.
La precisazione dell'operatore sostituisce la precedente conferma di contatto
per la configurazione attuale.

Preparato target fisso 20 cm davanti alla posa corrente, trasformato nel frame
del mondo. Mapping Coraplex `MoveMotion` → `DifferentialDriveBaseGoal` verificato;
simulazione con anticollisione completata. Aggiunto peer DDS `127.0.0.1`
per la scoperta reciproca tra client e server nel container: senza questo
peer il client non trovava `fetch_world`; dopo la modifica mondo sincronizzato.

Avviato server Giskard closed-loop temporaneo con limite base `0.05` m/s;
goal reale Coraplex accettato, ma abortito dal monitor di progresso:
`DifferentialDriveBaseGoal/step3/position` senza progresso per 5 s.
Il contatto escluso non è la causa segnalata. Goal non completato;
server arrestato al timeout, stop eseguito dal ciclo di gestione del goal.
Non effettuati ulteriori tentativi di movimento.

Nel client compare anche `MissingWorldError` durante la deserializzazione
dell'errore restituito dal server: problema distinto dal mancato progresso,
ancora da correggere. Log server: `/tmp/tiago-position-server.log` nel container;
client temporaneo: `/tmp/tiago_position_client.py`.
