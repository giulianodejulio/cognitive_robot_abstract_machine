# Robocat: DifferentialDriveBaseGoal aggiornato con DLO

Integrato nella repository originale il comportamento del commit milestone
`bbdbf48a92a84ad04a4a35a22f603c8f5dae888b`, verificato nel clone separato
a `0744d77776f9235839e5828d9830aa9f8dd2251c`. Nessun clone sostituito,
nessuna dipendenza o configurazione hardware PAL modificata.

Il task ora limita la traslazione cartesiana durante l'allineamento iniziale
a 0,001 m/s, con priorità massima; allinea entro `heading_threshold=0,02 rad`
prima della traslazione; vicino alla posizione desiderata usa l'orientamento
finale e mantiene l'altezza corrente della base. È un limite quasi nullo,
non la stessa imposizione di velocità esattamente zero di `--pure-turn`.

Preservata l'estensione locale `orientation_threshold`: i goal Robocat
mantengono 2 cm e 5° nelle due fasi di posa. L'allineamento iniziale usa
il minimo tra la tolleranza angolare e 0,02 rad (circa 1,15°). Senza override
angolare rimane il fallback alla singola `threshold` nelle fasi di posa.
Il goal selezionato nella CLI resta relativo alla posa iniziale misurata,
50 cm avanti con orientamento finale uguale a quello iniziale.

Verifica: regressioni prima della modifica (6 fallimenti attesi, 3 passati),
poi 22 test passati includendo i 13 test esistenti del task con solver.
Esecuzione solo in container senza rete, limite RAM 896 MiB,
`--orm-build never`. Nessun test di movimento reale eseguito per questa
integrazione. Il launcher precedente è stato fermato, a client inattivo,
per liberare memoria e richiedere il caricamento del codice aggiornato.

## Esecuzione manuale nel container

Terminale 1, bridge DDS già attivo con `--control`:

```bash
cd /workspace/cram
bash scripts/launch_robocat_giskard.sh \
  --feedback laser --max-linear 0.05 --max-angular 0.1
```

Aspettare `STACK READY`. Il READY del server deve riportare feedback
`/direct_laser_odometry/odom`. Non usare `wheel` per questo confronto.

Terminale 2, registrazione prima di inviare il goal:

```bash
source /opt/ros/overlay_ws/install/setup.bash
export ROS_DOMAIN_ID=103
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/cram/doc/robocat_cyclonedds.xml
bag_path=/tmp/robocat-ddrive-dlo-$(date +%Y%m%d-%H%M%S)
echo "$bag_path"
ros2 bag record -o "$bag_path" --storage mcap \
  --max-cache-size 4194304 --disable-keyboard-controls \
  --qos-profile-overrides-path /workspace/cram/doc/robocat_bag_qos.yaml \
  --include-hidden-topics --topics \
  /scan /tf /tf_static /robot_description /joint_states \
  /direct_laser_odometry/odom /mobile_base_controller/odom /cmd_vel \
  /diagnostics /power/is_emergency /joy_priority \
  /giskard/command/_action/status /giskard/command/_action/feedback
```

Aspettare `Recording` e le sottoscrizioni ai topic richiesti.
Terminale 3, soltanto con condizioni operative attuali confermate:

```bash
cd /workspace/cram
./scripts/robocat-goal 0.5 --differential-drive
```

Ctrl+C nel terminale del goal ne richiede la cancellazione mantenendo attivi
server e client. Dopo l'esito e l'arresto delle ruote, Ctrl+C nel terminale
del recorder chiude il bag. Conservarlo sul laptop prima di ricreare il
container; da un terminale host, sostituendo il nome effettivo:

```bash
docker cp aicor-cram-demo:/tmp/robocat-ddrive-dlo-TIMESTAMP \
  /home/gdj/workspaces/robocat_experiments/odometry_checks_20261106/
```

Il bag contiene comandi: riprodurlo soltanto su un dominio isolato. La TF
grezza può contenere sia DLO sia i frame interni Giskard; una ricostruzione
visuale deve scegliere coerentemente la catena della posa, non mescolarle.

## Cosa confrontare

Misurare lo stesso punto del telaio tra inizio e fine; osservare oscillazioni
dello yaw lungo il rettilineo, eventuali inversioni e aggiustamenti finali.
Confrontare con il task cartesiano sugli stessi 50 cm e sugli stessi limiti.
Il fix agisce principalmente sull'allineamento e sui goal vicini: non
corregge latenza, rumore o tracking della DLO, né l'errore della catena ruote.
Il task differential drive può correggere orientamento durante il tratto;
non impone un orientamento costante lungo tutta la traiettoria.
