> **Setup Robocat attuale (16 novembre 2026):** [deployment/ROBOCAT_CURRENT.md](deployment/ROBOCAT_CURRENT.md). Le sezioni successive descrivono il salvataggio storico del 6 novembre.

# CRAM/Giskard su TIAGo e Robocat reali

Questa branch conserva le modifiche locali degli esperimenti sul robot reale
al 6 novembre 2026. Il punto di partenza è il commit
`5d20cd79aa6bab471822e8b76bf57c062a91d721` del repository AICOR.
Non è una sostituzione con il clone milestone aggiornato.

La directory originale e il mount del container attivo non sono stati cambiati.
La copia è in `cram_real_robot_fork`, branch
`real-robots/tiago-robocat-20261106`. Il remoto di pubblicazione è il fork
`giulianodejulio/cognitive_robot_abstract_machine`; il push su upstream
è disabilitato nella configurazione Git locale di questa copia.

## Cosa è conservato

- Modelli TIAGo singolo braccio e Robocat, URDF/SRDF e mesh necessarie,
  comprese le mesh `.obj` precedentemente escluse dal `.gitignore`.
- Server/client persistenti, launcher, sender CLI, cancellazione con Ctrl+C,
  sequenze di movimento, rotazioni pure e selezione feedback wheel/laser.
- Modifiche locali al framework, inclusa l'integrazione della correzione
  DifferentialDriveBaseGoal dal commit upstream
  `bbdbf48a92a84ad04a4a35a22f603c8f5dae888b`, conservando tolleranze distinte.
- DDS Router dedicato, Dockerfile e revisioni sorgenti del bridge, domini
  separati Robocat Humble 102 e CRAM Jazzy 103, profili CycloneDDS.
- Test delle aggiunte, documentazione operativa e configurazioni RViz.
- `uv.lock`, inventari Python/apt e revisioni dei pacchetti ROS del container.
- Mappa dell'ascensore, grafo SLAM serializzato, frame staging/doorway,
  provenienza della calibrazione e dati delle prove odometriche.
- Parametri osservati sul robot e configurazioni desiderate per ripristinarli.

Il PDF di onboarding, credenziali SSH/Wi-Fi, impostazioni personali,
cache, ambienti virtuali e ORM generati non sono aggiunti alla branch.
I rosbag e gli altri dati voluminosi sono conservati nell'archivio locale
descritto in `deployment/ARTIFACTS.md`, non nel repository Git.

## Ripresa e ricostruzione

Leggere `AGENTS.md`, `doc/tiago_handover.md`, `doc/robocat_setup.md` e
`doc/robocat_giskard_background.md`. I documenti includono una cronologia:
le sezioni più recenti e i dati in `deployment/` distinguono lo stato attuale
dai tentativi precedenti.

La ricostruzione disponibile è `scripts/setup_tiago_laptop.sh`: costruisce
l'immagine base e l'estensione PAL dai Dockerfile, monta questo checkout,
esegue `uv sync --locked --extra dev --active` e rigenera gli ORM.
Richiede il nome `aicor-cram-demo` libero: non rimuovere il container attuale
per provarla. Non è stata eseguita una nuova build durante questo salvataggio.

Le dipendenze Python vivono nel container: ricrearlo richiede nuovamente
l'installazione. `deployment/container/` registra lo stato reale e le
differenze apt rispetto a `tiago-cram-real:local`, incluso PlotJuggler.
Non è un export dell'immagine Docker né una garanzia di ricostruzione
bit per bit: le sorgenti e i repository apt esterni possono cambiare.

I modelli e le mesh conservano la loro provenienza PAL; i diritti dei
singoli asset non sono sostituiti dalla licenza del codice applicativo.

## Stato Robocat al salvataggio

- Ethernet laptop statica `10.68.0.206/24`; robot `10.68.0.1`, alias `robocat`.
- Jazzy resta su dominio 103, mai collegato direttamente al dominio 102.
- Feedback wheel selezionabile nel launcher; `wheel_radius` resta 0,1 m.
- Compensazione sperimentale: raggi sinistro/destro 2,0; interasse 1,22.
  Sono valori runtime verificati, non ancora una calibrazione fisica definitiva.
- Goal rettilinei 0,25 / 0,5 / 1 m validati dall'operatore con queste correzioni.
- Sequenza manuale Nav2 fino a staging e Giskard avanti 1,6 m riuscita,
  con limiti Giskard 0,3 m/s e 0,2 rad/s.
- Routing separato `/snm_vel` preparato e testato offline; al salvataggio
  il mux nativo osservato non contiene ancora il nuovo ingresso.
- Ingresso autonomo con sorveglianza continua della porta non implementato.
  Non avviare automaticamente il rettilineo solo perché Nav2 ha successo.
- AMCL/mappa attivi e frame vanno verificati sul robot prima di riprendere.

Le configurazioni desiderate in `deployment/robot/desired/` non vengono
applicate automaticamente. Nessun riavvio di controller o movimento è
stato eseguito per creare questo snapshot.

## Verifiche

L'ultima modifica al routing ha superato 12 test offline mirati e i controlli
di sintassi shell. Il salvataggio verifica l'identità dei file copiati,
la presenza delle risorse ignorate necessarie e l'integrità Git. Non equivale
all'esecuzione dell'intera suite o a una nuova validazione sul robot.
