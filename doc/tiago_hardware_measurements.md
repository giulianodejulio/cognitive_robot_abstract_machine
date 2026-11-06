# Misure hardware e prova di 20 cm — 29 ottobre 2026

## Obiettivo e limiti

Dimensionare il loop base TIAGo–Giskard a 20 Hz, senza HPM/SNM o persone.
Queste misure riguardano il modello, la postura e i goal descritti sotto;
non costituiscono requisiti universali per altre scene, robot o frequenze.
Il requisito minimo del loop completo resta da validare: il goal reale
è stato interrotto da un difetto del registratore, non da un limite hardware
dimostrato. Nessuna prova reale ripetuta automaticamente.

## Macchina e condizioni

- Laptop Intel Core i7-8550U: 4 core fisici, 8 thread logici; CPU 0–3
  corrispondono a quattro core distinti, CPU 4–7 ai rispettivi sibling SMT.
- RAM visibile circa 7.7 GiB; circa 1.8 GiB disponibili durante la sessione.
  Swap di 2 GiB quasi interamente occupata. Al controllo iniziale, pressione
  memoria PSI media nulla: swap occupata da sola non prova paging attivo.
- Container principale senza quota CPU o cpuset, limite configurato 8 GiB.
  Le misure sono state fatte in container temporanei limitati, riusando
  in lettura il medesimo ambiente Python e lo stesso repository.
- ROS 2 Jazzy lato framework, Humble lato robot, Ethernet diretta,
  dominio 72 e CycloneDDS configurato nel repository.
- Nessuna dipendenza aggiunta, nessun test esistente o ORM modificato.

## Benchmark solver offline

Container senza rete, RAM massima 1408 MiB, memoria virtuale massima 5 GiB,
OPENBLAS_NUM_THREADS=1 e OMP_NUM_THREADS=1. Modello TIAGo completo,
postura articolare campionata dal robot, base differenziale, goal simulato
20 cm, limiti 0.1 m/s e 0.1 rad/s, autocollisione e collisione esterna
abilitate. Nessun publisher ROS/hardware. Nessun ambiente reale completo.

Il ciclo è `Executor.tick()`: calcolo e aggiornamento simulato, esclusi
ricezione DDS, sincronizzazione tra processi, logging completo e pacing.
Tre esecuzioni sequenziali sullo stesso mondo, con stato iniziale ripristinato
e affinità del thread chiamante a 1, 2 e 4 core fisici. I pool nativi già
creati non sono stati profilati thread per thread: non è una misura di
scalabilità parallela o una simulazione di tre CPU differenti.

| Core assegnati | Compilazione | Mediana tick | P95 tick | Massimo tick | Picco RSS |
|---|---:|---:|---:|---:|---:|
| 1 | 195 ms | 0.79 ms | 0.95 ms | 1.19 ms | 349 MiB |
| 2 | 144 ms | 0.79 ms | 0.97 ms | 1.06 ms | 351 MiB |
| 4 | 148 ms | 0.81 ms | 1.26 ms | 1.67 ms | 352 MiB |

Tutti e tre i goal completati in 48 cicli simulati. Nessun tick sopra
50 ms, budget del periodo a 20 Hz. Campioni piccoli e ordine fisso:
riscaldamento delle cache e variazioni di frequenza/carico possono incidere.
Non si conclude che un core sia migliore di quattro.

## Unica prova reale di 20 cm

Richiesta corrente e condizioni operative confermate dall'operatore:
presenza, area libera, teleop disattivata, emergenza disponibile.
Goal Coraplex–Giskard 20 cm in avanti, orientamento finale invariato,
velocità massima della sola istanza 0.1 m/s, rotazione massima 0.1 rad/s,
timeout goal 15 s, soglia feedback 1 s e guard spostamento 30 cm.
Default del framework invariati. Nessun controller robot attivato o riavviato.

Container con cpuset CPU 0,1, RAM massima 1408 MiB; recorder su nodo ed
executor separati da Giskard. Goal accettato e comandi inviati per 18 cicli,
0.872 s tra primo e ultimo ciclo registrato. Picco comando solver 0.09933 m/s.

Il ciclo completo misurato include sincronizzazione degli ingressi,
solver, guard/logging, pubblicazione comandi e feedback; esclude l'attesa
del pacer e parte del lavoro asincrono degli executor.

| Misura ciclo completo | Valore |
|---|---:|
| Mediana | 20.9 ms |
| P95, ordine empirico su 18 campioni | 41.8 ms |
| Massimo | 55.2 ms |
| Cicli sopra 50 ms | 1/18 |
| Picco RSS del processo | 499.7 MiB |

Questi pochi cicli non certificano il rispetto continuativo della frequenza.
RSS e limite cgroup non sono equivalenti; non è stata cercata la più bassa
quota RAM che consenta l'avvio e l'esecuzione completa.

## Causa dell'aborto: registratore, non solver lento

Al primo comando `/cmd_vel`, il callback del registratore ha sollevato
`KeyError: publisher_gid`, terminando il suo thread spinner. I metadati
effettivi di questo rclpy contengono:

- publication_sequence_number
- reception_sequence_number
- source_timestamp
- received_timestamp

Non contengono publisher_gid. La precedente correzione da attributo a
chiave di dizionario era quindi incompleta; i test sintetici non coprivano
il formato effettivo. Il sorvegliante ha poi abortito per
`stale /direct_laser_odometry/odom` perché la sua ricezione era ferma.

Negli ultimi cicli, il ricevitore usato dal controllo Giskard aveva invece
odometria laser con età circa 0.07–0.12 s e giunti circa 0.03–0.04 s.
La ricezione sul robot o la capacità del solver non spiegano questo aborto.
Anche il ritardo osservato nella prova precedente non va interpretato
automaticamente come prova di hardware insufficiente.

Il server ha mandato stop nel percorso di errore; inviati poi ulteriori
zeri da un processo indipendente e verificato l'arresto dalle ruote.
Spostamenti finali rispetto ai campioni iniziali:

- Ruote: 0.07926 m.
- Laser: 0.06548 m.
- Velocità finali longitudinali e angolari praticamente nulle.

Goal non completato. Questo aborto non riproduce il vecchio NoProgressError.
Il salvataggio finale dello script ha inoltre tentato di leggere un risultato
action già consumato, causando MissingActionResultError; i dati erano già
registrati e lo stop era già stato effettuato.

## Correzione verificata senza robot

Registratore aggiornato per gestire anche metadati senza GID. Se l'identità
non è disponibile, confronta il comando ricevuto con quelli registrati
prima dell'invio e blocca comandi inattesi. Questa verifica rileva valori
diversi ma non identifica una sorgente che invia valori identici: non
sostituisce ingressi mux distinti o un arbitraggio esclusivo.
Il salvataggio finale usa lo stato del chart, senza leggere un risultato
action già consumato. Il controllo interno del publisher è verificato prima
di pubblicare, distinguendo il suo feedback da quello del recorder.

Due nuovi test riproducevano KeyError prima della modifica; dopo la correzione
passano tutti e cinque i test del registratore. Passa anche un test con veri
messaggi rclpy: comando previsto accettato, comando diverso rilevato.
Quest'ultimo test è eseguito in container `--network none`, dominio 173 e
solo loopback, quindi nessun comando può raggiungere il robot.
Nessun ulteriore goal hardware inviato.

## Cosa possiamo concludere sull'hardware minimo

- Per il solo solver di questa base e di questa scena, un core di questa
  CPU ha ampio margine rispetto a 50 ms; non emerge un bisogno di GPU.
- Con due core fisici il processo completo ha usato circa 500 MiB RSS.
  Non è però stato validato fino al completamento del goal, né sotto carico
  prolungato, e non è provata una quota RAM minima.
- Prima di richiedere hardware più potente serve una prova completata con
  il recorder corretto, poi misure prolungate di latenze, overrun, RAM,
  page fault/swap e pause di ricezione. Solo allora si possono abbassare
  quote CPU/RAM e individuare un minimo verificato con margine operativo.
- Il laptop attuale non è dimostrato insufficiente: il difetto osservato è
  software. I requisiti qui misurati escludono percezione, HPM/SNM, scene
  ricche di ostacoli e controllo di braccia/testa.

Artefatti sul laptop: `/tmp/tiago-trial-20cm-20261029/`, inclusi benchmark,
log reale, trasporto, analisi, script corretto e test isolati. I container
temporanei sono stati rimossi; `aicor-cram-demo` resta attivo.

## Successiva prova 30 cm senza registratore

Su nuova richiesta, con operatore e condizioni confermati, eseguito un
solo goal di 30 cm in avanti a massimo 0.1 m/s. Il precedente avvio era
stato fermato prima del goal dal blocco teleop; dopo il «pronto»
dell'operatore il goal è stato accettato.

Nessun recorder, wrapper del publisher o loop strumentato aggiunto:
usati il publisher e il loop standard del framework, con i rispettivi
controlli feedback e limiti. Lettura essenziale di emergenza/joy, timeout
15 s e stop finale; due core fisici e container limitato a 1408 MiB.

Il goal non è terminato entro 15 s ed è stato cancellato dal timeout
della prova. Non risultano aborti del registratore né NoProgressError.
Spostamento laser 0.23142 m, ruote 0.23971 m; errore di posizione nel
modello rispetto al target 0.08466 m. Velocità finali ruote circa
-0.000081 m/s e -0.000399 rad/s, quindi praticamente nulle.
Il goal di 30 cm non è validato. Nessun altro goal inviato.
Artefatti: `/tmp/tiago-trial-30cm-20261029/run.log` e `summary.json`.
