# TIAGo: passaggio al nuovo laptop

## Obiettivo

Riprendere gli esperimenti sul TIAGo reale dopo migrazione del workspace.
Leggere prima AGENTS.md e doc/tiago_setup.md. La documentazione descrive
prove successive: le sezioni finali aggiornano le affermazioni storiche.

## Ambiente ricostruibile

- Robot tiago-72c, singolo braccio, Ubuntu 22.04, ROS 2 Humble.
- SSH atteso: pal@10.68.0.1. Nessuna password salvata qui.
- Framework: ROS 2 Jazzy, Python /opt/ros/cram-env/bin/python.
- Container aicor-cram-demo, rete host, repository /workspace/cram.
- Ricostruzione: bash scripts/setup_tiago_laptop.sh /percorso/repository.
- Il clone remoto da solo non contiene le modifiche locali non pubblicate.
- Setup Python tramite uv.lock; ORM rigenerati con scripts/regenerate_all_orm.py.
- Docformatter già autorizzato e dichiarato tra le dipendenze dev.
- DDS: dominio 72, rmw_cyclonedds_cpp, doc/tiago_cyclonedds.xml.
  Selezione Ethernet 10.68.0.0; peer robot e loopback. Esportare
  CYCLONEDDS_URI=file:///workspace/cram/doc/tiago_cyclonedds.xml soltanto
  dopo aver configurato l'interfaccia Ethernet del nuovo PC.
- Il setup non configura Ethernet o SSH, non avvia controller e non richiede GPU.
- Nuova macchina: verificare host key del robot, route e sincronizzazione clock.
  Non disabilitare il controllo host key per aggirare un errore SSH.

## Stato verificato

- Giunti, lidar, TF, odometria laser ricevuti e applicati al modello.
- /direct_laser_odometry/odom è coerente con TF odom → base_footprint.
- URDF originale acquisito, mesh PAL complete, modello TiagoSingleArm.
- collision.srdf rimuove solo i 103 riferimenti a frame camera assenti.
- operator_exclusions.srdf esclude solo gripper_link–torso_fixed_column_link,
  su richiesta esplicita dell'operatore, che non osserva contatto reale.
- Coraplex SingleArmBaseMotion seleziona DifferentialDriveBaseGoal.
- Comando base: /cmd_vel Twist → twist_mux → controller TwistStamped.
- Interfaccia in giskardpy.middleware.ros2.scripts.iai_robots.tiago.single_arm:
  default standalone; --hardware abilita closed-loop. Solo base comandabile.
- Limiti default ±0.05 m/s e ±0.1 rad/s; stop con giunti/odometria non aggiornati.
- Prove reali riuscite a +0.02 m/s e +0.1 m/s, con stop automatico.
- /power/is_emergency ha QoS TRANSIENT_LOCAL: richiederlo per ricevere
  lo stato persistente. LatestMessageSubscriber ammette override qos_profile.
- Ultime verifiche: 9 test del modello passati; altre verifiche in tiago_setup.md.

## Problemi aperti

1. Primo goal reale Coraplex di 20 cm: accettato, poi NoProgressError
   DifferentialDriveBaseGoal/step3/position. Causa non individuata.
2. Client: MissingWorldError deserializzando l'errore del server.
3. Joypad: testa risponde, base no; Start non commuta la luce da bianca a
   gialla secondo l'operatore. Diagnostica robot ancora da effettuare.
   In precedenza /joy_priority true bloccava navigazione, false la consentiva.
4. Discrepanza odometria ruote/laser; nessuna misura a terra completata.
5. Nessun ambiente reale completo nel mondo del monitor: non assumere che
   la sola anticollisione del modello protegga da tutti gli ostacoli reali.

## Ripresa

Iniziare in sola lettura: route Ethernet, SSH, clock, ROS/DDS, giunti,
odometrie, lidar, TF, emergenza e diagnostica mux. Non cambiare modalità
né inviare movimenti senza una richiesta attuale dell'operatore e condizioni
operative aggiornate. Le vecchie conferme riguardavano prove già terminate.
Non riavviare automaticamente vecchi script di movimento in /tmp: temporanei,
non versionati e non inclusi nella ricostruzione.

Usare soltanto Python e test nel container. Per pytest: controllare memoria,
limitare il consumo, sottogruppi separati, --orm-build never. Non modificare
un test fallito senza autorizzazione. Non leggere/editare ORM generati.
Non installare nuove dipendenze senza autorizzazione. Non pubblicare su upstream.

## Verifica migrazione — 29 ottobre 2026

- Ethernet laptop: `enx00e04c0902b0`, `10.68.0.135/24`; robot raggiungibile.
- Accesso SSH con chiave del laptop verificato. Voce host aggiornata dopo
  confronto con l'impronta documentata; backup locale conservato.
- Laptop e robot sincronizzati tramite NTP; Chrony sul robot riporta circa
  1 ms di scarto rispetto alla propria sorgente.
- Container esistente con rete host e mount del repository corretto;
  import principali riusciti, tutte le 34 mesh presenti.
- Corretto il conflitto colcon/setuptools: `setuptools 79.0.1` nel container,
  vincolo dev `>=68,<80` e lockfile locale aggiornati. `uv lock --check --offline`
  riuscito. Il lockfile è ignorato da Git: conservarlo nella copia del workspace.
  Gli avvisi pip su docopt e pathlib derivano dagli override uv intenzionali:
  docopt-ng e pathlib della libreria standard sono disponibili.
- Ripristinato DDS aggiungendo `General/DontRoute=true` alla configurazione
  locale: il robot pubblicizza anche `192.168.68.62`, fuori sottorete, e la
  traccia mostrava invii verso tale indirizzo. Dopo la modifica giunti, lidar,
  TF, emergenza, joystick e diagnostica arrivano nel container.
  Fonte: https://cyclonedds.io/docs/cyclonedds/latest/config/network_interfaces.html
- Emergenza `false`, letta con QoS reliable/transient-local.
  Anche `joy_priority` richiede transient-local per lo stato persistente:
  valore `true`, mux priorità 100, blocco joystick attivo; ricarica e
  assisted teleop liberi. Nessuna commutazione richiesta o effettuata.
- Odometrie ruote e laser non ricevute né nel container né nei tentativi
  locali sul robot. TF odom/base assente nella finestra osservata.
- `mobile_base_controller` è inattivo. Nei log dell'avvio corrente:
  `Either the left or right wheel position is invalid for index [0]`,
  seguito dalla disattivazione automatica del controller per errore di update.
  Questo spiega l'assenza di comando base e odometria ruote attuali;
  il problema specifico della commutazione Start resta non verificato.
- `arm_4_joint` ha posizione NaN; diagnostica attuatore: disconnesso,
  `BoardOverTemp`, temperatura riportata 64, stato scheda Error.
  TF del braccio contiene valori non validi. Entrambi gli attuatori della
  testa segnalano `Input Voltage Error`. Nav2 risulta inattivo.
- Nessun controller attivato, reset hardware o comando di movimento inviato.
  Gli errori robot e le odometrie mancanti restano da risolvere prima
  di qualunque nuova prova. Nessuna nuova prova di NoProgressError eseguita.

## Aggiornamento dopo riavvio e inventario CRAM–SNM

Le verifiche successive al riavvio dell'operatore aggiornano lo stato sopra:
base attiva, giunti finiti, odometrie ruote/laser e TF ripristinate.
Inventario HRI, arbitraggio mux, limiti della diagnosi del vecchio goal e
crash del test offline sono descritti in
[tiago_stationary_checks.md](tiago_stationary_checks.md).
Nessuna nuova prova di movimento eseguita.

## Benchmark hardware e successiva prova di 20 cm

Su nuova richiesta e con condizioni confermate eseguita una prova reale
di 20 cm a massimo 0.1 m/s, interrotta dal guard dopo il crash del recorder.
Il controllo Giskard riceveva feedback fresco. Goal non completato,
arresto verificato, nessun tentativo automatico aggiuntivo.
Misure e correzione del registratore, verificata con ROS isolato, in
[tiago_hardware_measurements.md](tiago_hardware_measurements.md).
Il laptop non risulta dimostrato insufficiente; il minimo hardware del
loop completo resta da validare dopo una prova senza errori strumentali.

## Goal 30 cm, timeout 60 s, senza recorder

Su richiesta dell'operatore, prova soltanto sul robot reale: abortita dopo
circa 31.4 s per `NoProgressError` sullo step 3/orientamento. Posizione nel
modello entro 6.2 mm, yaw residuo 27.8°. Arresto verificato.
Rilevata divergenza yaw laser/ruote fino a circa 119°; non è ancora
determinato quale stima sia corretta. Candidati e verifiche discriminanti in
[tiago_goal_failure_candidates.md](tiago_goal_failure_candidates.md).
Nessun recorder, nessuna simulazione, nessun ulteriore movimento automatico.
