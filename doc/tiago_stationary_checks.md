# TIAGo: inventario e controllo base — 29 ottobre 2026

## Metodo e stato dopo il riavvio

Verifiche tramite SSH in lettura e sottoscrizioni dal container
`aicor-cram-demo`, dominio 72, CycloneDDS con `doc/tiago_cyclonedds.xml`.
Nessun avvio di controller, nodo HRI, server hardware Giskard o mock SNM;
nessuna commutazione di modalità, richiesta di boarding o velocità inviata.

Il robot era stato riavviato dall'operatore. Stato osservato:

- `mobile_base_controller` attivo.
- 14 posizioni articolari finite; il NaN di `arm_4_joint` non presente.
- Odometria ruote: 600 campioni in circa 12 s; laser: 180 campioni.
  Età degli ultimi campioni circa 0.06 e 0.14 s; velocità praticamente nulle.
- TF `map -> odom`, `odom -> base_footprint` e TF lidar disponibili.
  La posa TF odom/base coincide con l'odometria laser nel campione letto.
  Le pose ruote e laser differiscono: non sono intercambiabili.
- Emergenza false. `joy_priority=true`, mux priorità 100, blocco joystick
  attivo; blocchi ricarica e assisted teleop liberi.
- Gli errori attuatore riportati prima del riavvio non sono ricomparsi
  nella finestra diagnostica osservata; ciò non prova la risoluzione fisica.

## Inventario HRI

Pacchetti installati sul robot:

| Pacchetto | Versione letta |
|---|---|
| hri_msgs | 2.2.1 |
| hri_person_manager | 2.2.1 |
| hri_body_detect | 3.4.1 |
| hri_face_detect | 2.2.2 |
| hri_face_body_matcher | 2.1.1 |
| hri_engagement | 2.0.2 |
| hri_emotion_recognizer | 1.0.2 |
| hri_face_identification | 2.5.3 |
| social_perception_cfg_tiago | 1.6.1 |

Presenti anche `hri`, `pyhri`, `hri_actions_msgs`, `hri_privacy_msgs`,
`hri_rviz`, `hri_visualization`, `human_description`, `rqt_human_radar`.
Gli eseguibili di detection, tracking persone, matching, engagement,
identificazione e visualizzazione sono disponibili.

Nessun nodo HRI attivo nel grafo locale Humble e nessun topic `/humans/`
scoperto né sul robot né nel container. In
`/home/pal/.pal/module_manager/enable_disable_list.yaml` sono esplicitamente
disabilitati `hri_body_detect` e `hri_person_manager`.
La configurazione body detection usa la camera RGB frontale e
`use_depth: False`: posizioni metriche e TF delle persone richiederanno
verifica effettiva prima dell'uso per navigazione sociale.

Nel container non sono presenti pacchetti HRI nell'indice ament e
`hri_msgs` non è importabile. Per consumare i messaggi ROS4HRI servirà
predisporre le interfacce Jazzy; nessuna nuova dipendenza installata.

Il simulatore `tiago_arise_gazebo_simulator` espone invece nel proprio
codice `/humans/persons/tracked`, `/humans/persons/known`,
`/humans/bodies/tracked` di tipo `hri_msgs/IdsList`, associazione `body_id`,
`anonymous`, `location_confidence` e TF persone. Questa è l'interfaccia
da confrontare con i dati reali quando la pipeline HRI sarà avviata.

## Arbitraggio CRAM–SNM

SNM è un modulo futuro, da sviluppare a partire da
`tiago_arise_humans/mock_social_navigation.py`; non è installato o avviato
come modulo operativo in questa verifica.

Percorso reale verificato tramite endpoint e parametri del mux:

```text
/cmd_vel [Twist, ingresso navigation priorità 10, timeout 0.5 s]
  -> /twist_mux
  -> /mobile_base_controller/cmd_vel [TwistStamped]
  -> /mobile_base_controller
```

Sull'uscita c'è un solo publisher, `twist_mux`; sottoscrittori:
`mobile_base_controller` e `twist_marker`. Non osservati publisher diretti
di CRAM o SNM sul controller.

Su `/cmd_vel` sono registrati nove publisher: uno `controller_server`,
due `global_behavior_server`, sei `behavior_server`. Sono endpoint
registrati, non prova di pubblicazione simultanea.
CRAM configura lo stesso topic in `single_arm.py`; il mock SNM ha
`cmd_vel_topic=/cmd_vel` come default e quindi è configurabile.

**L'esclusività CRAM–SNM non è implementata né validata.** Il mux distingue
ingressi, non publisher diversi sullo stesso ingresso. I messaggi JSON
`boarding_request` e `boarding_result` del mock non acquisiscono un lock
del mux, non cancellano un goal Giskard e non bloccano altri publisher.
Non basta inviare uno zero alla sorgente che si vuole sospendere:
continuerebbe a competere sul medesimo ingresso.

Per il modulo reale occorrono ingressi distinti per CRAM e SNM e un
arbitro con stato esplicito: arresto/cancellazione della sorgente uscente,
conferma della sospensione, abilitazione della sorgente entrante e gestione
di timeout, crash, feedback vecchio e richieste duplicate. Va mantenuta
la precedenza dei blocchi operatore. Una semplice priorità SNM superiore
non basta: alla sua scadenza potrebbe riemergere un comando CRAM ancora
attivo. La prova dell'arbitro va inizialmente svolta con uscita scollegata
dall'hardware e dominio DDS separato.

## Goal Coraplex–Giskard senza progresso

Il log `/tmp/tiago-position-server.log` e il client temporaneo del vecchio
laptop non sono presenti nel container migrato. Non è quindi possibile
attribuire retroattivamente la causa all'attuale blocco joystick, ai vecchi
errori hardware o al solver.

Conferme dal codice:

- `SingleArmBaseMotion` costruisce `DifferentialDriveBaseGoal` per REAL.
- La sequenza ha orientamento iniziale, avvicinamento e posa finale.
  Il terzo passo è un `CartesianPose`, quindi controlla ancora la posizione,
  oltre all'orientamento. `step3/position` non significa solo rotazione.
- Soglia del goal 0.01; `StillProgressing` ha timeout predefinito 5 s e
  osserva la diminuzione dell'errore. L'aborto non identifica chi ha impedito
  il progresso e non prova, da solo, che la base non si sia mai mossa.
- L'interfaccia reale legge l'odometria laser e TF map/odom; manda zero con
  giunti o odometria assenti/vecchi. Tale stop non registra qui una ragione
  dettagliata da correlare al monitor di progresso.
- `MissingWorldError` durante deserializzazione resta un problema distinto.

Tentati i due test esistenti di raggiungimento posizione in
`test/experiments_test/test_tiago_framework.py`, con `--orm-build never`,
thread numerici limitati, memoria virtuale massima 1280 MiB, dominio 173 e
sola interfaccia loopback. Il processo è abortito nativamente in
`trimesh.convex_decomposition` durante costruzione del modello, prima del
solver; nessun risultato di raggiungimento disponibile. Il limite di memoria
può essere rilevante, ma la causa del crash non è stata stabilita.
Nessun test modificato e nessun ORM letto, modificato o rigenerato.

Prima di un nuovo goal reale, predisporre una registrazione sincronizzata
di target e frame, errore di posizione/orientamento per passo, velocità
solver e uscita CRAM, uscita mux e sorgente selezionata, motivi di stop,
odometrie, TF ed emergenza. Servono una richiesta attuale e condizioni
operative confermate per qualsiasi prova che abiliti il comando hardware.

## Limite di introspezione Jazzy–Humble

Dal container sono visibili topic e dati, ma i nomi dei nodi remoti e
degli endpoint risultano sconosciuti. I nomi dei publisher sopra riportati
sono stati quindi verificati tramite CLI Humble sul robot, con scoperta
senza daemon. Il daemon locale del robot ha anche restituito errori
intermittenti di introspezione; non è stato riavviato.

## Singola prova successiva autorizzata — 29 ottobre 2026

Operatore presente, area libera e arresto disponibile confermati; teleop
disattivato dall'operatore. Goal Coraplex–Giskard 10 cm davanti alla posa
corrente, orientamento finale invariato, limite della sola istanza 0.1 m/s
e 0.1 rad/s, timeout goal 15 s. Default del framework invariati.

Prima della prova sono passati tre test del monitor di progresso e i due
test di raggiungimento sul modello TIAGo. Questi ultimi sono stati ripetuti
senza rete, con cache nuova, affinità a una CPU, limite RAM reale 1152 MiB
e memoria virtuale 5 GiB; nessun test o ORM modificato. Il precedente crash
non si è riprodotto in questa configurazione.

L'unico goal reale è stato accettato e ha comandato la base per 12 cicli,
circa 0.56 s tra primo e ultimo comando registrato. Picco dell'uscita solver
0.09625 m/s. Il mux ha inoltrato un comando osservato di 0.07891 m/s;
la registrazione ROS dei comandi è incompleta per il problema sotto.

Il guard della prova ha abortito per `stale /direct_laser_odometry/odom`
con soglia 1 s. Non è un nuovo `NoProgressError`; la causa del vecchio
aborto resta da determinare. Il goal di 10 cm non è stato completato.

Durante lo stop è emerso un secondo errore nel registratore temporaneo:
in questa versione rclpy, MessageInfo è un dizionario, mentre lo script
leggeva `info.publisher_gid` come attributo. Il thread spinner è terminato.
Il processo è stato quindi fermato e sono stati inviati ripetutamente
comandi zero da un processo indipendente; arresto confermato dalle ruote.

Spostamenti finali rispetto ai campioni immediatamente precedenti al goal:

- Ruote: 0.04874 m.
- Laser: 0.03930 m.
- Velocità finali ruote longitudinale e angolare praticamente nulle.

La prima misura intermedia ruote di 1.6 cm non includeva tutto lo spostamento
fino all'arresto. L'operatore ha osservato un breve movimento.

Artefatti locali: `/tmp/tiago-trial-20261029/` sul laptop, con `trial.py`,
`run.log`, `transport.jsonl` e `analysis.json`. Il container temporaneo
della prova è stato rimosso; `aicor-cram-demo` rimane attivo e invariato.
Il registratore va corretto e verificato offline prima di riutilizzarlo.
La configurazione su una CPU ha anche portato l'executor ROS a un solo
worker: possibile contributo al ritardo del feedback, non causa dimostrata.
Nessun ulteriore goal o tentativo automatico effettuato.

### Correzione del registratore e verifica passiva

Corretto lo script temporaneo: `info["publisher_gid"]` al posto
dell'accesso ad attributo. Tre regressioni offline riproducono il difetto
prima della correzione e passano dopo: sorgente propria, sorgente estranea,
persistenza del blocco dopo interferenza. Separati inoltre i callback del
registratore da quelli Giskard con un nodo e un executor dedicati a due
worker. L'ingresso hardware dello script richiede ora `--execute`.
Quest'ultima configurazione completa non è stata provata con un goal reale.

Verifica passiva di 8 s con due worker, registrazione degli stessi topic e
nessun publisher di movimento:

- Laser: 118 campioni, età mediana 0.132 s, massima 0.142 s;
  intervallo massimo tra callback 0.075 s.
- Ruote: 393 campioni, età massima 0.096 s.
- Giunti: 391 campioni, età massima 0.092 s.
- Nessun errore diagnostico ERROR e nessun comando non nullo osservato.

Nel vecchio log, al primo interlock l'età laser era 1.023 s e l'ultimo
callback risaliva a 0.910 s prima. Questo documenta un ritardo di ricezione
nel processo della prova; non dimostra che la causa fosse soltanto il
numero di worker. Rimane da verificare il comportamento sotto carico solver.

Un secondo controllo passivo di 4 s ha ricevuto 60 campioni laser con età
massima 0.138 s. Alla chiusura dei harness passivi sono comparsi warning
di teardown ROS (file già chiuso nel primo harness, handle in distruzione
nel secondo): distinti dal difetto publisher_gid, non osservati durante
la raccolta. Nessun nuovo goal inviato.
