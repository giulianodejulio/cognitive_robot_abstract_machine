# Rettifilo Robocat: oscillazioni yaw, 4 novembre 2026

**Riferimento fisico acquisito il 6 novembre:** un goal ruote di 0,50 m,
massimo comandato 0,05 m/s, ha prodotto 1,00 m misurato dall'operatore
sullo stesso punto del telaio. Ruote: 0,494868 m; DLO: 0,981978 m.
Questo confronto sostiene una sottostima della catena ruote, non un
fattore due spurio della DLO. Incertezza della misura fisica non quantificata;
non identifica ancora il parametro errato e non valida la rotazione.
Dati in `/home/gdj/workspaces/robocat_experiments/odometry_checks_20261106/goal_0p5_wheel_005_trial01/`.

**Aggiornamento diagnostico del 6 novembre:** il confronto scan diretto nel
successivo test ascensore indica circa 1,138 m contro 0,566 m delle ruote.
La variazione di distanza frontale laser conferma circa 20 cm/s contro
10 cm/s ruote. Joint states e odometria ruote sono coerenti con raggio
configurato 0,10 m e interasse 0,25 m. Il precedente rapporto DLO/ruote
circa due non identifica quindi, da solo, un errore della DLO: va verificata
prioritariamente la calibrazione fisica/encoder della catena ruote, oltre
alla scala laser con un riferimento misurato. Le conclusioni storiche
seguenti non dimostrano la correttezza metrica delle ruote. Nessun fattore
empirico applicato. Risultati e limiti nel file
`/home/gdj/workspaces/robocat_experiments/elevator_20261106_staging_01/diagnostic-findings.md`.

Una prova reale autorizzata: goal cartesiano di 1 m, limiti server 0,1 m/s
e 0,2 rad/s. Acquisiti soltanto `/cmd_vel`, odometria ruote e DLO, senza
visualizzazione o compressione. Goal cancellato dopo circa 32 s per
oscillazioni persistenti; cancellazione confermata e richiesta `--ready`
riuscita dopo l'arresto. Nessun secondo goal inviato.

## Misure

- Comando: circa 20 Hz; velocità angolare raggiunge entrambi i limiti
  ±0,2 rad/s. Nell'intervallo attivo di 31,4 s si osservano 32 cambi di
  segno tra campioni con modulo superiore a 0,02 rad/s. Anche la velocità
  lineare alterna avanti/indietro vicino al goal, fino a ±0,1 m/s.
- Odometria ruote: 50 Hz, massimo intervallo di ricezione 37 ms.
- DLO: circa 9,1 Hz, massimo intervallo 806 ms; età del timestamp alla
  ricezione mediana 155 ms, massima 202 ms. I clock non sono stati
  rimisurati in questa prova: le età includono un eventuale offset.
- Correlazione tra comando e twist ruote su tempi di ricezione, cercando
  ritardi 0–600 ms a passi di 10 ms: massimo circa 0,992 per la velocità
  angolare a 180 ms e 0,995 per la lineare a 200 ms. Sono stime aggregate
  comprensive di trasporto, campionamento e dinamica; non latenze isolate
  del solo controller né prova di causalità.
- Nel tratto iniziale quasi uniforme (circa 0,5–4,5 s dal primo comando),
  le ruote riportano 0,399 m di spostamento e 0,398 m integrando il proprio
  twist. DLO riporta 0,799 m di spostamento, ma soltanto 0,398 m integrando
  il proprio twist. Rapporto posa/integrale DLO circa 2,008.
- Al momento del risultato di cancellazione il client riporta errore
  planare DLO 15,35 cm e orientamento −6,143°; non è un errore misurato
  rispetto a ground truth esterna.

## Interpretazione e seguito

Il comando Giskard oscilla: la situazione non è quella di comando
angolare nullo con base che devia da sola. Le ruote seguono strettamente
il comando, con ritardo. Resta possibile che un feedback errato o ritardato
inneschi e amplifichi le correzioni; questa prova non isola il solver
come causa iniziale.

La discrepanza interna tra posa e twist DLO è il candidato prioritario
da indagare prima di modificare tolleranze o guadagni. Un offset costante
dei frame non spiega il rapporto sulle distanze relative. Verificare
configurazione e origine della posa DLO, frequenze e trasformazioni TF,
ed eventuali publisher duplicati sul dominio nativo. Senza ground truth
non si conclude quale stima descriva esattamente il movimento fisico.

Dati nel container `aicor-cram-demo`:
`/tmp/robocat-yaw-probe-20261104/`, bag SQLite e `signals.csv` con tempi
di ricezione, timestamp sorgente, twist e posa. Sono file temporanei:
copiarli prima di ricreare il container.

## Verifica successiva delle sorgenti, senza inviare movimenti

- Sul dominio nativo 102 un solo publisher di `/direct_laser_odometry/odom`:
  `/direct_laser_odometry`. Ingressi: `/scan`, odometria ruote, TF e TF statici.
  `/scan` ha un solo publisher, `/pal_laser_filters`.
- DLO pubblica la TF odom/base; `enable_odom_tf=false` nel controller ruote.
  Non emerge una doppia pubblicazione da questi due componenti. Nei 96
  campioni nativi DLO con TF allo stesso timestamp, le traslazioni coincidono
  esattamente. Non usare la TF come stima indipendente dalla DLO.
- Configurazione runtime: `dlo.op_mode=0`, filtro moto attivo, solver con
  un thread e 50 iterazioni massime, griglia 2 cm. Nessun parametro di scala
  del moto identificato tra quelli esposti. Non interpretare `op_mode=0`
  senza documentazione dell'implementazione. Pacchetti PAL installati:
  `dlo_ros` 0.4.0, `dlo_core` 0.1.0; sorgenti del calcolo non disponibili
  nelle directory installate ispezionate. Nessun parametro modificato.
- Laser rispetto a base_footprint: traslazione circa (0,113; 0; 0,298) m,
  rotazione nulla. Laser a circa 10 Hz, 2000 raggi, scansione circa 99 ms.
  I filtri esposti sono di range, esclusione angolare e footprint; nessuna
  compensazione dinamica esplicita identificata in questi parametri.
- Confronto di catture native e bridged sovrapposte: identici 15 campioni
  DLO, 81 campioni ruote, 16 trasformazioni odom/base e 15 scan. Per gli
  scan il confronto considera uguali i NaN nelle stesse posizioni.
  Il bridge non altera i contenuti dei campioni confrontati.
- Nella cattura nativa: ruote circa 50 Hz, DLO/TF/scan circa 10 Hz.
  Età mediana scan 151,65 ms e DLO 154,34 ms già sul robot: il ritardo
  rispetto al timestamp non è introdotto principalmente dal bridge.
  Non attribuire tutta questa età a calcolo: comprende la convenzione
  del timestamp della scansione, acquisizione e pipeline.
- Clock robot avanti di circa 2,4 ms rispetto al laptop, stimato su una
  connessione SSH persistente, migliori RTT 3,4–4,1 ms. Per i campioni
  comuni la ricezione sul laptop appare circa 2,5–2,9 ms prima di quella
  nel recorder nativo; l'offset spiega gran parte della differenza.
  A questa risoluzione non è giustificata una latenza di bridge negativa
  né un suo valore preciso inferiore al millisecondo.
- All'inizio della cattura bridged le odometrie indicano movimento, pur
  non avendo inviato goal in questa verifica. Quel segmento non viene
  trattato come baseline stazionaria. Nella successiva cattura nativa le
  ruote sono ferme: intervallo posa ruote circa 2 micrometri; intervallo
  posa DLO circa 2,8 mm in x e 7,9 mm in y, in circa 9,6 s.

Conclusione: nessun duplicato odometrico o errore evidente di geometria
identificato; trasporto verificato su campioni identici. Rimangono candidati
la costruzione della posa DLO a partire da scan e ruote e i ritardi del
feedback nel controllo. Il rapporto circa 2 della precedente prova non
è spiegato da queste sole letture. Il twist DLO potrebbe derivare dal
canale ruote: senza il codice non assumere che sia la derivata della
posa scan-matching. Serve una verifica mirata per isolare la causa prima
di correggere scala, guadagni o tolleranze.

Catture nel container: `/tmp/robocat-dlo-native-20261104.db3` e
`/tmp/robocat-dlo-bridge-20261104/`; misura clock in
`/tmp/robocat-clock-20261104.txt`. Nessun riavvio o cambio di configurazione
eseguito durante queste verifiche.

## Confronto con feedback ruote

Successiva prova reale autorizzata: stesso goal cartesiano relativo di 1 m,
limiti 0,1 m/s e 0,2 rad/s e tolleranze 2 cm / 5°. Server e client avviati
temporaneamente con remapping ROS globale
`/direct_laser_odometry/odom:=/mobile_base_controller/odom`, verificato
sulle sottoscrizioni effettive. Nessun file sorgente o parametro PAL
modificato. La DLO resta attiva come osservazione indipendente dal feedback
scelto. Acquisizione degli stessi tre topic del precedente test.

- Risultato `SUCCESS`, circa 11,477 s dal lato client; comando attivo
  circa 10,372 s. Nessuna cancellazione necessaria, ruote ferme confermate
  dalla richiesta di readiness al termine.
- Errore finale del goal, misurato dal feedback ruote: circa 1,71 cm
  e 0,249°. Non è una misura con ground truth esterna.
- Nell'intervallo attivo più 0,5 s: spostamento ruote circa 1,0171 m avanti,
  laterale circa 0,09 mm. Yaw ruote rispetto all'inizio tra −0,206° e
  +0,254°: forte riduzione delle oscillazioni rispetto alla prova DLO.
- Picco comando angolare circa 0,022 rad/s, rispetto a ±0,2 rad/s nella
  precedente prova; velocità lineare minima −0,00068 m/s, senza inversioni
  significative come quelle osservate prima.
- Per lo stesso movimento la DLO riporta circa 2,0030 m avanti e 3,12 cm
  laterali, con yaw relativo finale circa +1,894°. La discrepanza di scala
  rispetto alle ruote ricompare anche quando la DLO non guida il controllo.

Questo singolo confronto indica che la catena di feedback DLO è un
candidato forte per le oscillazioni precedenti, mentre la stessa catena
Giskard/base può seguire un rettilineo con feedback ruote. Non separa
l'effetto della scala da quello della latenza, della frequenza o del
rumore; non prova quale distanza sia fisicamente esatta senza riferimento
esterno. Non introdurre una correzione empirica dividendo la DLO per due.

Al termine arrestati i due processi temporanei e riavviato il launcher
Robocat standard, con feedback DLO. Bag del confronto nel container:
`/tmp/robocat-wheel-feedback-probe-20261104/`; log temporanei
`/tmp/robocat-wheel-server.log` e `/tmp/robocat-wheel-client.log`.
