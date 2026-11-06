# TIAGo: confronto controllato DLO / ruote, 4 novembre 2026

Due prove reali autorizzate dall'operatore, che ha confermato condizioni
operative libere. Identità SSH `tiago-72c` verificata con fingerprint ECDSA
documentato. Ethernet laptop 10.68.0.206, robot 10.68.0.1, dominio ROS 72.
Controller base attivo e ruote ferme prima delle prove.

## Procedura

Due goal cartesiani di 1 m dalla rispettiva posa iniziale, orientamento
iniziale mantenuto; limiti 0,1 m/s e 0,2 rad/s, tolleranze 2 cm / 5°.
Primo goal con DLO, secondo con feedback ruote. Cambiata soltanto la posa
letta da server e client tramite remapping ROS temporaneo, senza cambiare
parametri PAL o file TIAGo nel repository. Per fissare il limite lineare
è stata usata una copia temporanea del bringup in `/tmp` nel container.
Acquisiti gli stessi tre topic: comando, odometria ruote, DLO.

Entrambi i goal riusciti, rispettivamente circa 12,041 e 11,312 secondi
dal client. Ruote ferme confermate dopo ciascuna prova. I processi temporanei
sono stati fermati al termine; non è rimasta una configurazione wheel
implicitamente attiva. Il bringup standard non è stato riavviato.

## Risultati

| Misura | Feedback DLO | Feedback ruote |
| --- | ---: | ---: |
| Yaw relativo stimato dalle ruote, minimo/massimo | −2,75° / +5,47° | −0,78° / +0,57° |
| Picco assoluto comando angolare | 0,197 rad/s | 0,086 rad/s |
| Cambi di segno del comando oltre 0,02 rad/s | 13 | 3 |
| Avanzamento finale stimato dalle ruote | 1,089 m | 1,009 m |
| Avanzamento finale stimato dalla DLO | 1,014 m | 0,989 m |
| Errore finale planare sul feedback scelto, dopo arresto | 1,55 cm | 0,94 cm |
| Errore finale yaw sul feedback scelto, dopo arresto | 1,56° | 0,13° |

L'operatore osserva oscillazione nella prima prova e oscillazione
praticamente assente con feedback ruote. I dati sono coerenti con questa
osservazione. I valori del percorso sono calcolati rispetto all'ultima
posa disponibile prima del primo comando significativo, fino a 1,5 s
dopo l'ultimo comando significativo. Nessun riferimento esterno al moto.

DLO circa 15 Hz, ruote circa 50 Hz. Non si osserva la discrepanza di scala
circa ×2 trovata su Robocat. Il confronto indica una riduzione marcata
delle correzioni usando le ruote; non separa il ruolo di frequenza,
latenza, rumore o altri aspetti del feedback DLO. Un solo tentativo per
condizione non costituisce una validazione generale del controllo.

## Clock e interpretazione dei ritardi

Età apparente mediana dei messaggi: ruote circa 1,039 s, DLO circa 1,085 s.
Misura successiva via SSH persistente: clock robot indietro di circa
1,039 s rispetto al laptop, migliori RTT 3,1–4,1 ms. Quindi queste età
non dimostrano un secondo di ritardo del trasporto: l'offset spiega quasi
tutta l'età ruote. L'eccesso della DLO è circa 45 ms. Non sono stati
modificati gli orologi durante o dopo le prove.

## Dati

Container `aicor-cram-demo`, directory `/tmp/tiago-feedback-comparison/`:
`laser-bag/`, `wheel-bag/`, log server/client delle due condizioni,
`summary.json` e `clock.txt`. Copiare i dati prima di ricreare il container.
La chiave verificata usata per SSH è in `/tmp/tiago-comparison-known-hosts`
sul laptop, senza modifiche al suo `known_hosts` permanente.
