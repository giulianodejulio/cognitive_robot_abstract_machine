# Robocat: DDS e Giskard persistenti

Verifica del 4 novembre 2026 sul robot reale, senza goal di movimento.
Risorse e launcher TIAGo conservati. Il modello Robocat e le mesh sono in
`semantic_digital_twin/resources/robots/robocat`; non serve Gazebo.

## Architettura

- Robot Humble: dominio 102, stack PAL e mux nativi.
- CRAM/Giskard Jazzy: dominio 103, DDS su loopback.
- Container `robocat-dds-router`: feedback 102 → 103; con `--control`,
  anche `/cmd_vel` 103 → 102 verso l'ingresso navigazione del mux.
- Nessun inoltro del grafo ROS, dei servizi o delle azioni tra i domini.
  Il client e l'action server Giskard comunicano localmente sul 103.

Non collegare nodi Jazzy direttamente al dominio 102: il confronto controllato
ha riprodotto il crash del module manager PAL con quella configurazione.

## Terminale sul laptop: bridge

La Ethernet deve essere configurata con `10.68.0.206/24`.

```bash
cd /percorso/cognitive_robot_abstract_machine
bash scripts/launch_robocat_dds_router.sh --control
```

Avviare una sola istanza. Se il container è già attivo, non rilanciarlo.
Senza `--control` vengono inoltrati soltanto i feedback. Ctrl+C arresta
il bridge; da un altro terminale: `docker stop robocat-dds-router`.

## Terminale nel container: server e client

```bash
cd /workspace/cram
bash scripts/launch_robocat_giskard.sh --max-linear 0.1 --max-angular 0.2
```

La posa di feedback è selezionabile all'avvio. Per usare le ruote:

```bash
bash scripts/launch_robocat_giskard.sh --feedback wheel
```

Per usare la DLO:

```bash
bash scripts/launch_robocat_giskard.sh --feedback laser
```

`laser` resta il valore predefinito. `wheel` applica a server e client il
remapping ROS `/direct_laser_odometry/odom:=/mobile_base_controller/odom`.
Non cambia la configurazione PAL, i limiti o le tolleranze. La DLO continua
a funzionare sul robot e il lidar rimane disponibile. Il log `READY` del
server indica il topic effettivo. Fermare il launcher corrente con Ctrl+C
prima di cambiare sorgente; i goal successivi useranno la nuova posa.
Il comando `robocat-goal` e la sequenza con FIFO Robocat restano gli stessi.
La precisione fisica dell'odometria ruote, specialmente con slittamenti sulla
soglia dell'ascensore, resta da validare.

Il bringup Python Robocat nasconde per impostazione predefinita i messaggi noti
del parser sui campi URDF ignorati e l'avviso del database opzionale assente.
Gli altri messaggi e gli errori operativi restano visibili. Per rivedere questi
avvisi avviando direttamente il server, aggiungere `--verbose` a
`scripts/robocat_giskard_bringup.py`. Il filtro vale solo durante `server.setup()`
e non modifica il modello né i controlli del robot.

Il launcher imposta l'ambiente ROS dedicato e avvia entrambi i processi,
senza inviare goal né attivare controller hardware. Attendere `STACK READY`
e `GOALS ENABLED`. Gli interblocchi possono impedire i goal senza terminare
il bringup. Ctrl+C nel launcher arresta i due processi di sua proprietà.
Log completi: directory `/tmp/robocat-giskard-launch.*` stampata all'avvio;
`--verbose` mostra anche i messaggi normalmente filtrati.

## Altro terminale nel container: stato e goal

```bash
cd /workspace/cram
./scripts/robocat-goal --ready
```

Questa richiesta verifica soltanto lo stato. Per una successiva prova reale,
con operatore presente, area libera, teleop disattivata, presa scollegata
ed emergenza raggiungibile:

```bash
./scripts/robocat-goal 0.30
```

Il goal cartesiano è relativo alla posa iniziale raggiunta: avanzamento
rettilineo mantenendo l'orientamento iniziale. Il client riusa il task e le
tolleranze attuali del client TIAGo: 2 cm e 5°, non una configurazione
Robocat indipendente. Ctrl+C nel comando goal richiede la cancellazione;
server e client restano attivi.

## Sequenza Robocat: rettilineo, cancellazione, rettilineo

Con bridge e launcher già attivi, da un altro terminale del container:

```bash
cd /workspace/cram
/opt/ros/cram-env/bin/python -m scripts.robocat_goal_sequence --execute \
  --first-distance 0.4 --cancel-after 0.4 --second-distance 0.4
```

Usa il sender e la FIFO Robocat senza richiedere `--fifo`. Riusa la logica
della sequenza TIAGo: cancella dopo il tempo indicato dall'inizio del moto
rilevato, attende cancellazione confermata e ruote ferme, poi calcola il
secondo goal dalla posa raggiunta. La selezione wheel/laser resta quella
del launcher. I valori predefiniti di distanza e tempo sono ereditati
dalla sequenza esistente; passarli esplicitamente rende la prova ripetibile.
Sono disponibili anche `--turn-degrees` e `--pure-turn` per la rotazione
intermedia. Il comando esegue movimenti reali e richiede le medesime
condizioni operative della prova singola.

## Sicurezza e verifiche

Il preflight richiede giunti e odometrie freschi e finiti, emergenza libera,
teleop disattivata e mux libero. Durante il movimento il publisher richiede
ancora feedback e interblocchi validi. Le odometrie devono usare
`odom` / `base_footprint`. Il comando DDS è volatile, con cronologia 1 e
lifespan 250 ms; non viene mantenuto per una successiva riconnessione.
Questi controlli non sostituiscono l'arresto fisico né provano una copertura
completa degli ostacoli circostanti.

Il 4 novembre: preflight passato, server e client persistenti pronti,
`robocat-goal --ready` riuscito; PAL rimasto attivo con PID 1400.
24 test software passati in container senza rete, con limite RAM 896 MiB,
inclusi modello/mesh, QoS, route dei comandi, interblocchi e protocollo di
cancellazione con nuovo goal. Nessun goal reale inviato da questi controlli:
tracking e cancellazione sul Robocat fisico restano da validare.

## Controller base inattivo dopo l'avvio

Il 4 novembre il log PAL ha mostrato un'attivazione iniziale riuscita,
seguita dalla disattivazione di `mobile_base_controller` per posizione ruota
non valida. Con posizioni tornate numeriche, l'operatore lo ha riattivato
e ha confermato il ripristino della teleop. Sul robot, dopo aver caricato
`/opt/pal/alum/setup.bash` e l'ambiente nativo 102:

```bash
ros2 control set_controller_state mobile_base_controller active
ros2 control list_controllers
```

Non eseguire automaticamente questo recupero: riabilita la ricezione dei
comandi alla base. Se torna inattivo, leggere il log `controller_manager`
e il feedback ruote; non ripetere alla cieca né aggirare l'emergenza.
