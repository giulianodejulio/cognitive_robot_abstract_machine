# Robocat: inventario iniziale

## Stato aggiornato al 4 novembre 2026

Bridge DDS installato e verificato; modello e mesh Robocat integrati.
Server e client persistenti pronti sul dominio Jazzy 103, preflight e
richiesta di stato riusciti. L'operatore ha riattivato il controller base
dopo un errore di posizione ruota all'avvio e confermato il ripristino
della teleop. Nessun goal Giskard reale inviato nella verifica odierna.

Per comandi e stato attuale vedere [Giskard Robocat](robocat_giskard_background.md).
Le sezioni seguenti conservano la cronologia delle verifiche del 3 novembre;
le indicazioni di risorse non ancora installate sono storiche e superate.

Verifiche del 3 novembre 2026, esclusivamente in sola lettura via SSH.
Nessun controller attivato, server Giskard avviato o goal inviato.
Configurazioni TIAGo conservate senza modifiche.

## Collegamento e ROS

- Alias SSH del laptop: `robocat`, utente `pal`, IP `10.68.0.1`.
- Accesso con chiave riuscito; hostname `robocat-2c`.
- Ethernet laptop: `enx00e04c0902b0`, `10.68.0.206/24`, route diretta.
- Ubuntu 22.04.5, ROS 2 Humble, overlay PAL `/opt/pal/alum`.
- Ambiente letto dal processo robot_state_publisher: `ROS_DOMAIN_ID=102`,
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
- DDS sul robot: `/opt/pal/alum/share/cyclone_robot_cfg/config/cyclone_config_control.xml`.
- NTP robot sincronizzato; scarto effettivo rispetto al laptop non misurato.

Non usare automaticamente il dominio 72 di TIAGo. La ricezione dal container
del laptop non è ancora stata verificata per Robocat.

## Base e topic

La configurazione installata usa `diff_drive_controller/DiffDriveController`,
ruote `wheel_left_joint` e `wheel_right_joint`, raggio 0.1 m e separazione
0.25 m. Sono parametri del file installato, non una misura fisica né una
verifica dei parametri runtime.

- Mux: ingresso navigazione `/cmd_vel`, priorità 10, timeout 0.5 s.
- Uscite rimappate: `/mobile_base_controller/cmd_vel` e
  `/mobile_base_controller/cmd_vel_unstamped`; tipo effettivo da verificare.
- Odometrie ruote e DLO: campioni ricevuti su `/mobile_base_controller/odom`
  e `/direct_laser_odometry/odom`, entrambe con header frame `odom`.
- `/joint_states`, `/scan`, `/tf`, `/tf_static`, `/robot_description` e
  `/diagnostics` scoperti; contenuto e frequenza ancora da validare.
- `/humans/voices/...` presenti: la discovery non dimostra disponibilità
  di tracking delle posizioni delle persone.

## Stati campionati

- `/power/is_emergency`: `false`, ricevuto con best-effort/volatile.
  La richiesta transient-local produce incompatibilità DURABILITY:
  non copiare il requisito QoS TIAGo per questo topic.
- `/joy_priority`: `true`, ricevuto con reliable/transient-local.
- `/power/is_charging`: `false`, ricevuto con best-effort/volatile.
- Mux: configurati blocchi joystick 100, assisted teleop 200, charging 210.
  Diagnostica effettiva del mux ancora da leggere.

Questi sono campioni storici, non autorizzazioni o condizioni attuali
per muovere il robot.

## Prossimi passi

Verificare DDS dal container con dominio 102, feedback, TF e diagnostica;
acquisire il modello Robocat e identificare mesh e interfacce effettive.
Creare risorse dedicate Robocat senza sovrascrivere gli script TIAGo.
Non avviare gli script TIAGo su Robocat. Prima di movimenti occorrono
richiesta attuale e conferma delle condizioni operative.

## Verifica successiva: blocco runtime

Il container è in rete host con il repository montato correttamente.
Sul dominio 102 la discovery dei topic funziona con la configurazione DDS
Ethernet esistente, usata senza modificarla. Tuttavia un campionamento di
10 secondi non ha ricevuto giunti, lidar, odometrie, stati o descrizione;
nessuna trasformazione odom/base disponibile. Anche una successiva attesa
di 20 secondi per DLO non ha ricevuto messaggi.

Il servizio robot `pal_startup.service` è fallito il 3 novembre alle
11:17:56 UTC (12:17:56 ora italiana). Nel traceback del module manager:
`Failed to get node names: empty node name returned by the RMW layer`.
Controllo processi successivo: assenti ros2_control_node,
robot_state_publisher, direct_laser_odometry e twist_mux. Non sono stati
riavviati. Durante letture ROS locali sono comparsi anche errori
`string data is not null-terminated` e `invalid data size` in serdata.cpp.
La causa del guasto del grafo DDS non è ancora determinata; non è esclusa
un'interazione fra partecipanti Humble e Jazzy. Non attribuire già il
guasto a Ethernet o al container senza una verifica discriminante.

Prima del fallimento, gli endpoint confermavano:

- `/cmd_vel`: Twist, sottoscritto da twist_mux; otto publisher di Nav2
  presenti, non prova di comandi effettivamente trasmessi.
- `/mobile_base_controller/cmd_vel`: TwistStamped, pubblicato dal mux e
  sottoscritto dal controller, reliable/volatile.

La diagnostica runtime del mux e lo stato attuale dei controller restano
non verificabili finché lo stack è fermo. ListControllers è scaduto.

Modello espanso recuperato dai parametri del precedente lancio, analizzato
offline nel container: robot `robocat`, 17 link, 16 giunti; mobili
wheel_right_joint, wheel_left_joint e head_joint. Frame base_footprint,
base_link e laser_link presenti nella descrizione. Dieci URI di mesh,
in robocat_description, pal_urdf_utils e realsense2_description. La risorsa
RealSense contiene un percorso assoluto Humble: va risolto per Jazzy.
URDF acquisito nel container in `/tmp/robocat-acquired.urdf`; log e parametri
grezzi sul laptop in `/tmp/robocat-startup-model.txt`.
Nessun modello Robocat ancora caricato in Giskard, mesh non ancora importate.

## Diagnosi DDS e configurazione candidata

Verificata una differenza incompatibile nel messaggio interno del grafo:
`rmw_dds_common/msg/Gid` contiene `char[24]` in Humble (anche sul robot)
e `char[16]` in Jazzy. Sorgenti ufficiali:
https://github.com/ros2/rmw_dds_common/blob/humble/rmw_dds_common/msg/Gid.msg
e https://github.com/ros2/rmw_dds_common/blob/jazzy/rmw_dds_common/msg/Gid.msg.
È un candidato concreto per il crash di discovery, non una prova sperimentale
di causalità: nessun nuovo crash provocato per riprodurlo.

Versioni rilevate: CycloneDDS robot PAL 0.10.5, rmw_cyclonedds_cpp 1.3.4;
container CycloneDDS 0.10.5, rmw_cyclonedds_cpp 2.2.4.
I vecchi processi Giskard TIAGo sono ancora presenti sul dominio 72;
non sono stati fermati né riconfigurati.

Preparati file dedicati, senza modificare configurazioni robot o TIAGo:

- `robocat_cyclonedds.xml`: Jazzy sul dominio 103, solo loopback.
- `robocat_dds_router.yaml` e `robocat_dds_router_profiles.xml`: candidato
  bridge DDS Router fra robot 102 e Jazzy 103, allowlist dei soli feedback.
  Non inoltra ros_discovery_info, cmd_vel, servizi o azioni.
  Ethernet laptop vincolata all'indirizzo verificato 10.68.0.206:
  aggiornare questo file dedicato se cambia l'indirizzo del laptop.

XML/YAML validi sintatticamente; DDS Router non installato, quindi parsing
specifico Fast DDS e inoltro/QoS non ancora verificati. La proposta non è
una riparazione completata. Richiede autorizzazione alla nuova dipendenza
come da AGENTS.md; il recupero del servizio PAL richiede inoltre un riavvio
esplicito, che può riattivare controller e componenti di navigazione.
Non avviare nodi Jazzy direttamente sul dominio 102 per continuare le prove.

Riferimento per configurazione XML e separazione domini del router:
https://eprosima-dds-router.readthedocs.io/en/latest/rst/examples/xml_configuration.html.

## Confronto controllato locale / singolo subscriber Jazzy

L'operatore ha riavviato manualmente PAL alle 11:30:13 UTC. Senza nodi
Jazzy sul dominio 102, servizio attivo (PID 119104), giunti e DLO ricevuti
localmente, trasformazione odom/base_footprint presente e aggiornata.
Emergenza inizialmente false; dopo richiesta all'operatore di premere il
pulsante, lettura locale confermata true alle 11:33:52 UTC.

Eseguito quindi un solo subscriber Jazzy a /direct_laser_odometry/odom,
best-effort, nel container sul dominio 102, durata 10 secondi; rosout e
servizi parametri disabilitati, nessun comando di movimento. Ricevuti zero
campioni. Alle 11:34:04 UTC il module manager ha avviato l'arresto dei
moduli; controllo successivo: pal_startup failed, MainPID 0, exit-code.
Il subscriber è terminato e non è stato rilanciato. Questo confronto
supporta concretamente un guasto innescato dalla partecipazione Jazzy al
dominio Humble; non identifica ancora da solo la singola libreria da
correggere. Nessun riavvio automatico eseguito dopo il crash riprodotto.

L'operatore ha successivamente riavviato PAL: processo 131220, servizio
attivo e stabile nelle verifiche locali. Emergenza true, joint states
ricevuti, joint_state_broadcaster e head_controller attivi; base inattiva
ed emergency_stop_controller attivo. DLO non ricevuta in quella finestra.
Non sono stati attivati controller né rilasciata l'emergenza da remoto.

## Risorse hardware rilevate

Intel Core i7-1360P, 12 core / 16 thread, RAM 31875 MiB. Durante la lettura:
21957 MiB disponibili e swap 8191 MiB inutilizzata. Il margine suggerisce
che un futuro container CRAM sul robot sia plausibile; prestazioni e
convivenza con PAL restano da misurare. Spostare Jazzy sullo stesso host
non elimina l'incompatibilità del grafo con Humble.

Installazione del router in container dedicato autorizzata dall'operatore.
Build e istruzioni dedicate in `robocat_dds_bridge.md`; nessuna installazione
di DDS Router sul sistema del robot.

## Aggiornamento: bridge installato e feedback ricevuti

DDS Router 3.5.4 compilato e attivo nel container dedicato del laptop.
Prova isolata di inoltro e blocco inverso riuscita. Collegamento reale
robot 102 -> Jazzy 103: lidar, diagnostica mux, emergenza, ricarica,
joystick, URDF e TF statici ricevuti; PAL stabile con lo stesso PID.
Dettagli, comandi e consumi misurati in `robocat_dds_bridge.md`.

I giunti e le odometrie mancavano anche localmente sul robot nell'ultima
verifica: base, testa e joint_state_broadcaster inattivi sotto emergenza,
emergency_stop_controller attivo. Nessuna attivazione remota effettuata.
I vecchi file TIAGo non sono stati modificati dall'installazione del bridge.
