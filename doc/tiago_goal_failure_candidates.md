# TIAGo: candidati per il mancato raggiungimento — 29 ottobre 2026

## Prova reale con timeout 60 s

Richiesto dall'operatore di lavorare sul robot, senza simulazione.
Eseguito un solo goal reale: 30 cm avanti, massimo 0.1 m/s e 0.1 rad/s,
loop e publisher standard, senza recorder né strumentazione per ciclo.
Il timeout dello script è stato portato da 15 a 60 s; il limite esterno
del processo include il tempo di setup. Nessun criterio di successo o
monitor di progresso del framework modificato.

Il goal è stato accettato e abortito dopo circa 31.4 s, prima del timeout:

```text
NoProgressError:
DifferentialDriveBaseGoal/step3/orientation#13
stopped approaching a goal for 0:00:05
```

L'operatore ha osservato avanzamento, rotazione parziale e oscillazioni
avanti/indietro. Stato del chart letto soltanto dopo lo stop:

- Step 1 e step 2 completati.
- Step 3 attivo; posizione raggiunta, orientamento non raggiunto.
- Errore posizione nel modello: 0.00618 m, sotto la soglia di 0.01 m.
- Yaw target: 37.922°; yaw finale nel modello: 65.735°.
  Errore residuo: 27.813°, molto superiore alla soglia 0.573°.
- Velocità finali ruote circa -0.000082 m/s e -0.000403 rad/s.
  Processo di prova terminato, nessun altro goal inviato.

Nel client è ricomparso `MissingWorldError` deserializzando l'errore:
è distinto dalla causa server, attestata dal log NoProgressError.

## Candidati ordinati per priorità di verifica

### 1. Feedback angolare incoerente fra odometria laser e ruote

Entrambe dichiarano `odom -> base_footprint`. Nei dati precedenti:

| Campione | Yaw ruote | Yaw laser | Differenza laser–ruote |
|---|---:|---:|---:|
| Prima prova breve, iniziale | 23.733° | 26.741° | 3.008° |
| Prima prova 20 cm, iniziale | 23.043° | 25.884° | 2.841° |
| Dopo la prova 60 s, a robot fermo | -28.239° circa | 90.644° circa | 118.9° circa |

Non è dimostrato quale stima sia corretta, ma la differenza non è
compatibile con il solo piccolo offset osservato inizialmente.
Il framework usa la posa laser, non quella ruote, per chiudere il loop.
Se il segno o l'entità della variazione di yaw non rappresentano il
movimento fisico, il comando correttivo può non ridurre l'errore.

Da distinguere sul reale: errore della stima laser, odometria ruote,
origini/frame non coerenti, segno o scala della rotazione comandata.
Non assegnare arbitrariamente la correttezza alle ruote o al laser.

### 2. Target in map e feedback in odom non coerenti nel tempo

Durante questa prova, lo spostamento fra pose iniziale/finale nel modello
`map` è 0.30523 m, quello laser in `odom` 0.47776 m, quello ruote 0.43816 m.
Una trasformazione rigida costante conserverebbe la distanza fra due
posizioni: questa discrepanza richiede verifica.

Possibili cause: aggiornamento `map -> odom` da localizzazione durante il
movimento oppure stato client/server non allineato nei campioni iniziali.
La posa iniziale del target è quella del mondo client; la posa finale
riportata è quella del mondo server. Non sono state registrate TF iniziali
e finali sincronizzate, quindi non si può attribuire già il difetto ad AMCL.

Un controllo passivo successivo di 4 s ha trovato `map -> odom` stabile
a robot fermo: x=0.11270 m, y=-0.71027 m, yaw=-0.43451 rad.
Ciò non verifica la sua stabilità durante il movimento.

### 3. Accoppiamento posizione/orientamento nella fase finale

`DifferentialDriveBaseGoal` non è un comando di avanzamento rettilineo.
Il suo terzo passo è `CartesianPose`: continua a correggere x/y mentre
cerca di raggiungere l'orientamento finale. Una base differenziale non
può correggere direttamente un errore laterale. Correzioni del feedback
o deriva laterale possono quindi produrre rotazioni e inversioni di
marcia, compatibili con l'osservazione dell'operatore.

Inoltre il secondo passo punta verso la posizione residua con una direzione
che dipende dalla posa corrente. Avvicinandosi al target, piccoli errori
laterali possono cambiare molto la direzione richiesta. Per esempio,
1 cm laterale su 2 cm longitudinali richiede circa 26.6° di riallineamento.
Questo è un effetto geometrico, non una misura del disturbo reale.

La sola architettura del goal non prova il bug: va distinta da un feedback
angolare errato. Lo step finale individuato rende comunque questa fase
un candidato più concreto del parser o della trasmissione iniziale.

### 4. Rotazione reale non corrispondente al comando

Le vecchie prove a velocità costante validavano la traslazione, non una
calibrazione angolare completa di segno, guadagno e velocità minima.
Possibili contributi: risposta non lineare, attrito o soglia minima, ritardo,
scala/separazione ruote errata oppure comando contrastato da altra sorgente.
Non è disponibile una traccia per ciclo che dimostri uno di questi casi.

Parametri reali letti senza modificarli: controller base con feedback di
posizione ruote, open_loop=false, publish_rate=50 Hz, update_rate=100 Hz,
cmd_vel_timeout=0.25 s, wheel_radius=0.0985 m, wheel_separation=0.4044 m;
massimo angolare controller 1.05 rad/s. La prova limita invece a 0.1 rad/s.
Il framework non applica una velocità angolare minima (default zero).
Nessuna soglia hardware di movimento è stata misurata.

## Spiegazioni che non bastano per questo esito

- Timeout esterno: escluso come causa di questa prova, durata inferiore a 60 s.
- Tolleranza appena troppo stretta: 28° residui non sono un piccolo scarto
  attorno a 0.573°; allargare la tolleranza maschererebbe il problema.
- Velocità angolare bassa da sola: può allungare la manovra, ma non spiega
  perché l'errore angolare non diminuisca per 5 s.
- Laptop insufficiente: non dimostrato; il loop ha già ricevuto e comandato
  il robot fino allo step 3 e i benchmark mostravano margine. In questa prova
  non ci sono misure complete di latenze per escludere ogni ritardo.
- Recorder: assente. Il difetto dei precedenti recorder non causa questo esito.
- MissingWorldError client: impedisce di riportare correttamente il fallimento,
  ma il server aveva già abortito per mancata convergenza angolare.

## Prossima discriminante sul reale

Prima di ripetere l'intera manovra, isolare una piccola rotazione comandata
e confrontare direzione fisica, variazione yaw ruote e variazione yaw laser.
Verificare poi il target nello stesso frame odometrico coerente, separando
la posizione dall'orientamento finale. Non modificare simultaneamente
feedback, frame, tolleranze e guadagni: non si potrebbe attribuire l'esito.
Qualunque nuova prova di movimento richiede una richiesta corrente e
condizioni operative confermate. Nessuna ulteriore prova avviata qui.

Artefatti: `/tmp/tiago-trial-30cm-60s-20261029/` e parametri letti in
`/tmp/tiago-goal-analysis-20261029/robot-parameters.txt`. Script della prova
aggiornato a 60 s; nessuna modifica al controllore robot o al solver.
# Prova relativa di 30 cm a massimo 0.2 m/s

Il 29 ottobre 2026, su richiesta attuale e con condizioni operative confermate,
eseguito un solo goal calcolato dalla posa iniziale: traslazione di 0.30 m lungo
l'asse x locale e orientamento iniziale invariato. La posa risultante è espressa
internamente in map; il target non resta solidale alla base durante il movimento.
Nessun recorder aggiuntivo; RViz chiuso. Limite angolare 0.1 rad/s.

Goal cancellato automaticamente a 60 s, senza completamento: step1 concluso,
step2 ancora attivo, step3 non iniziato. Spostamento netto stimato laser
0.2675 m, ruote 0.2802 m; errore finale di posizione nel modello 0.01043 m.
Orientamento iniziale circa -12.06°, finale circa -3.47° (differenza 8.59°).
Questo scarto non coincide necessariamente con l'errore dello step2, che punta
verso il target. Velocità finali ruote praticamente nulle. Nessun retry automatico.
Log e riepilogo: `/tmp/tiago-trial-relative-02-20261029/` sul laptop.

# Confronto con Nav2 nativo sul robot

Il 29 ottobre 2026 eseguito, su richiesta e con condizioni confermate, un solo
NavigateToPose di 30 cm lungo l'asse iniziale della base, yaw invariato.
Navigator, planner e controller erano già active. Giskard non era in esecuzione.
Client nel container; planner e controllo nativi sul robot. Nessun recorder.

Per evitare successo immediato con la tolleranza nativa di 0.30 m, modificati
temporaneamente i parametri controller: xy_goal_tolerance 0.30→0.01 m,
yaw_goal_tolerance 0.25→0.01 rad, vx_max 0.25→0.20 m/s,
wz_max 1.2→0.1 rad/s e required_movement_radius 0.5→0.05 m.
Ripristino atomico dei cinque valori originali accettato al termine.
Il confronto riguarda quindi Nav2 con questi limiti, non il tuning originale.
Restano inoltre abilitati i recovery del behavior tree nativo.

L'operatore descrive movimento nettamente più fluido rispetto a Giskard, ma
il goal non è riuscito: cancellazione a 60.1 s, status CANCELED, errore finale
stimato in map 0.1856 m e yaw -2.5442 rad (circa -145.8°).
Spostamento netto ruote 0.4150 m, laser 0.3849 m; non sono lunghezze del percorso.
Velocità ruote finali praticamente nulle.

I log del componente nativo durante la prova riportano Failed to make progress,
Optimizer fail to compute path e, alla fine, Starting point in lethal space.
Questi errori impediscono di considerare riuscita la validazione nativa e
richiedono verifica di localizzazione/costmap; non provano da soli che la
discrepanza osservata con Giskard abbia la stessa causa.
Riepilogo e client: `/tmp/tiago-native-20261029/` sul laptop.

# Verifica statica successiva al confronto nativo

Controllo breve, senza movimento, del 29 ottobre 2026:

- TF odom→base_footprint coincide con la posa laser: yaw 0.22469 rad.
  L'odometria ruote, dichiarando gli stessi frame, riporta yaw 1.82337 rad:
  differenza circa 91.6°, con posizioni distanti circa 1.21 m.
  Nav2 è configurato con odom_topic mobile_base_controller/odom ma il TF
  dinamico della base segue il laser. Il solo offset assoluto non prova
  errori nelle velocità usate da Nav2; la coerenza dei frame va chiarita.
- AMCL riporta varianze x/y/yaw circa 0.250/0.170/0.208, cioè deviazioni
  standard 0.50 m/0.41 m/26°. Campione AMCL più vecchio dei TF di circa
  un minuto: da fermo gli aggiornamenti sono condizionati dal movimento.
  La stima non è sufficientemente precisa per assumere una validazione
  centimetrica del goal in map.
- Cella sotto la base nella mappa statica libera (0), costmap globale 29,
  locale 52. Nel controllo successivo la cella locale occupata (99) più
  vicina è a circa 0.224 m dal centro base; laser minimo circa 0.801 m
  nel frame del sensore. Queste distanze hanno origini differenti e la
  costmap può contenere ostacoli persistenti/filtri: non provano un falso
  ostacolo, ma richiedono verifica visiva di layer e footprint. L'errore
  lethal space del test non è presente nella cella centrale al controllo.
- Nessuna deriva misurata nelle due odometrie durante una finestra di 4 s;
  velocità praticamente nulle. Questo non valida gli stimatori in movimento.
- Verificato via GetParameters il ripristino dei cinque valori nativi:
  0.3, 0.25, 0.25, 1.2, 0.5 nell'ordine documentato sotto.

# Prova nativa DriveOnHeading senza navigazione globale

Il 29 ottobre 2026, con nuova richiesta e conferma operativa, eseguita una sola
azione /drive_on_heading: target.x 0.30 m, speed 0.20 m/s, time_allowance 60 s.
Behavior server già attivo, global_frame odom e robot_base_frame base_link;
TF base_link/base_footprint identico sul piano. Nessun planner globale, AMCL
o goal in map; protezione anticollisione nativa mantenuta. Nessun parametro
modificato e nessun recorder. Campioni iniziali e finali dopo un secondo di
assestamento, espressi negli assi iniziali di ciascuna odometria.

L'azione riporta SUCCEEDED (status 4, error_code 0) dopo 2.20 s, ma questo
non certifica la tolleranza finale di una posa. Valutazione finale indipendente:

| Feedback | Avanzamento | Scarto laterale | Cambio yaw | Errore dal target |
| --- | --- | --- | --- | --- |
| Laser (coerente con TF) | 0.3959 m | +0.02046 m | +0.02804 rad | 0.09808 m |
| Ruote | 0.4427 m | -0.00981 m | -0.02553 rad | 0.14306 m |

Entrambe fuori dalle soglie dichiarate prima della prova: errore posizione
massimo 0.01 m, laterale 0.01 m e yaw 0.01 rad. Velocità ruote finali
praticamente nulle. Avanzamento prevalentemente diritto ma con superamento
del target, non una validazione di 30 cm precisi. Le misure sono stime
odometriche, non misura fisica a terra. Il comportamento di arresto e la
divergenza delle stime richiedono ulteriori verifiche prima di attribuire
il superamento a una specifica causa. Nessun retry automatico.
Log, riepilogo e client: `/tmp/tiago-native-heading-20261029/` sul laptop.

# Confronto CRAM–Giskard in odometria locale

Il 29 ottobre 2026, su nuova richiesta e condizioni confermate, eseguito un solo
MoveMotion relativo di 30 cm a massimo 0.20 m/s, yaw limitato a 0.10 rad/s,
timeout 60 s. Configurazione temporanea fuori repository: root del mondo odom,
corpo intermedio trial_drive_odom con localizzazione identità e sincronizzazione
map→odom disabilitata. Input giunti, lidar e odometria laser reali invariati;
publisher standard, nessun recorder. Goal derivato dalla posa iniziale locale,
orientamento richiesto invariato. Nessuna simulazione.

Cancellazione automatica a 60 s, step1 concluso, step2 ancora attivo, step3
non iniziato. Spostamento netto laser 0.24589 m, ruote 0.34202 m;
errore finale dal target nel modello locale 0.05539 m. Velocità finali ruote
praticamente nulle. Questi esiti mostrano mancata convergenza anche senza
map/AMCL, ma non misurano fluidità o inversioni durante il percorso: non è
stata aggiunta una registrazione temporale. Il confronto con DriveOnHeading
va interpretato considerando che quello invia velocità di avanzamento e
termina sulla distanza, mentre MoveMotion mantiene un controllo di posa.
Nessun ulteriore movimento automatico.
Script, log e riepilogo: `/tmp/tiago-cram-odom-20261029/` sul laptop.

# Prova CartesianPoseStraight in odom

Il 29 ottobre 2026, con richiesta e condizioni nuovamente confermate, provato
un solo MoveMotion tramite mapping temporaneo StraightBaseMotion che restituisce
CartesianPoseStraight. Target 0.30 m lungo l'asse iniziale e yaw iniziale fisso,
mondo in odom senza sincronizzazione map/AMCL, limiti 0.20 m/s e 0.10 rad/s.
Nessun recorder. L'operatore riferisce correzioni continue attorno al goal.

Abort dopo circa 47.2 s per NoProgressError sia sul task posizione sia su
orientamento, prima del timeout 60 s; MissingWorldError nel client resta
secondario alla deserializzazione. Arresto verificato dalle velocità ruote
praticamente nulle. Nel modello laser: avanzamento 0.33292 m, laterale
-0.02430 m, cambio yaw +7.5058°, errore posizione 0.04091 m.
Spostamento netto ruote 0.42144 m. La rimozione del ripuntamento dinamico del
DifferentialDriveBaseGoal non elimina il problema: non è la sola causa.
Nessun ulteriore movimento. Script/log/riepilogo in
`/tmp/tiago-cram-straight-20261029/` sul laptop.

# Stessa prova rettilinea con tolleranze 5 cm e 5°

Il 29 ottobre 2026, su nuova richiesta e conferma operativa, ripetuto il test
locale di 30 cm a massimo 0.20 m/s, yaw massimo 0.10 rad/s. Composizione
esplicita degli stessi CartesianPositionStraight e CartesianOrientation in
Parallel, con identici pesi e binding del CartesianPoseStraight precedente;
variate soltanto threshold posizione a 0.05 m e orientamento a radians(5).
Nessun recorder o input map/AMCL, nessuna modifica di produzione.

Primo avvio fermato dal preflight prima di inviare un goal per stati mancanti
o attivi; verifica separata immediata: emergency false, joy_priority false,
odom ruote ricevuta. Secondo avvio con stessi controlli: unico goal accettato
e SUCCEEDED dopo circa 2.7 s (circa 2.34 s dalla fine parsing), client senza
errori. Questo contrasta con il precedente NoProgressError dopo circa 47 s.

Dopo l'arresto: spostamento netto laser 0.35774 m, ruote 0.40816 m; errore
finale dal target laser 0.05980 m, quindi fuori dai 5 cm nonostante SUCCESS.
La soglia di completamento non garantisce la posa dopo l'arresto: la misura
finale successiva al successo richiede attenzione al transitorio di arresto.
Velocità ruote finali praticamente nulle. Non è stata misurata la fluidità
temporale o la posa esatta nell'istante del successo; nessun retry ulteriore.
Il risultato sostiene il ruolo delle tolleranze nella mancata terminazione,
senza dimostrare che siano l'unica causa o validare precisione centimetrica.
Script/log/riepilogo: `/tmp/tiago-cram-straight-wide-20261029/` sul laptop.

