# Bridge DDS Robocat

Il robot usa Humble sul dominio 102. CRAM/Giskard deve usare Jazzy sul
dominio 103 e loopback: non avviare nodi Jazzy direttamente sul dominio 102.
Il test controllato ha riprodotto il crash PAL con un solo subscriber Jazzy
collegato direttamente. Vedere `robocat_setup.md` per le osservazioni.

## Build sul laptop

```bash
cd /percorso/cognitive_robot_abstract_machine
DOCKER_BUILDKIT=0 docker build \
  --memory 1536m --memory-swap 1536m --cpuset-cpus 0,1 \
  --tag robocat-dds-router:3.5.4 \
  .github/docker/robocat-dds-router
```

La build compila sequenzialmente le versioni fissate in `sources.tsv`.
L'immagine finale contiene solo eseguibile e librerie runtime, senza ROS.
Non sostituisce il container `aicor-cram-demo`.

## Avvio sul laptop

La Ethernet deve avere `10.68.0.206/24`, l'indirizzo attualmente verificato.
Se cambia, aggiornare `robocat_dds_router_profiles.xml` e il controllo nel
launcher, prima di avviare il bridge.

```bash
bash scripts/launch_robocat_dds_router.sh
```

Esecuzione in primo piano, nome container `robocat-dds-router`, memoria
massima 256 MB, un core CPU, filesystem in sola lettura. Ctrl+C lo arresta;
da un altro terminale: `docker stop robocat-dds-router`.

Per impostazione predefinita il bridge inoltra solo i feedback nella
allowlist da Robocat al laptop. Con `--control` abilita anche il solo
`/cmd_vel` dal laptop al mux del robot, volatile, cronologia 1, lifespan
250 ms. I comandi nativi del robot non vengono riflessi sul laptop.
Servizi, azioni, ros_discovery_info e immagini non vengono inoltrati.
Il bridge non avvia lo stack PAL.

L'integrazione server/client e lo stato aggiornato del 4 novembre sono in
[Giskard Robocat](robocat_giskard_background.md). Le osservazioni del
3 novembre sotto riportate descrivono la precedente verifica sotto emergenza.

## Verifica del 3 novembre 2026

- Immagine `robocat-dds-router:3.5.4` compilata: 94 824 705 byte, circa 95 MB.
- Quattro test pytest offline passati. Prova runtime su rete Docker isolata,
  domini 200/201: 607 campioni inoltrati, zero nel verso inverso e un campione
  persistente consegnato al late joiner.
- Primo parser runtime: rimossa la lista multicast XML vuota non supportata.
- QoS reliable/transient-local esplicito per joy_priority, robot_description
  e tf_static; cronologia rispettivamente 1, 1 e 20.
- Bridge reale lasciato attivo; consumo osservato circa 35 MiB su limite
  256 MiB. È una misura del carico attuale, non un massimo garantito.
- Da CRAM sul dominio 103, in 20 s: 190 scan, 587 messaggi diagnostici,
  26 stati emergenza, 13 stati ricarica, joystick e descrizione persistenti.
  TF statici base_footprint/base_link e base_link/laser_link disponibili.
- Emergenza true, ricarica false, joystick false; mux ok, priorità 0,
  blocchi joystick, ricarica e assisted teleop liberi.
- PAL è rimasto attivo con lo stesso PID 131220 durante la connessione.
  Nessun nuovo crash osservato.
- Giunti, odometrie e TF odom/base non ricevuti: confermato che mancano
  anche localmente sul robot. ListControllers riporta base, testa e
  joint_state_broadcaster inattivi, emergency_stop_controller attivo.
  Non sono stati riattivati e l'emergenza non è stata rilasciata.

Il trasferimento dei soli feedback funziona. Il controllo della base,
il bringup Giskard Robocat e la ricezione odometrica dopo il ripristino dei
controller restano da validare. Nessun movimento eseguito.

## Ambiente nel container CRAM

```bash
source /opt/ros/overlay_ws/install/setup.bash
export ROS_DOMAIN_ID=103
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/cram/doc/robocat_cyclonedds.xml
```

Questi export valgono solo per i processi successivi del terminale corrente.
Non usare il launcher TIAGo, che imposta il dominio 72 e il modello TIAGo.

Esempio di ricezione passiva:

```bash
ros2 topic echo /joint_states sensor_msgs/msg/JointState \
  --once --qos-reliability best_effort
```

L'esclusione di ros_discovery_info impedisce di vedere il grafo ROS remoto:
`ros2 node list` non deve essere usato per stabilire se il robot è connesso.
La verifica è la ricezione di campioni freschi. Servizi e azioni Giskard
restano locali al dominio 103. Il comando della base richiede l'opzione
esplicita `--control` e interblocchi validi nel client/server Robocat.

## Riferimenti

- https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/configuration.html
- https://eprosima-dds-router.readthedocs.io/en/latest/rst/examples/xml_configuration.html
- https://github.com/eProsima/DDS-Router/blob/v3.5.4/ddsrouter.repos
