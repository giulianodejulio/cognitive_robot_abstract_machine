# Robocat: proposta geometrica d'ingresso, 6 novembre 2026

**Verifica reale successiva non superata:** nella prova di avvicinamento
del 6 novembre l'operatore ha osservato lo chassis a cavallo della soglia,
mentre la posa ruote trasformata lo collocava con centro a x=−0,430 m.
Nessun goal di ingresso o allineamento successivo è stato inviato.
Il fondo osservato a porta aperta presenta uno scarto apparente di circa
0,560 m e 5,96° dal modello, assumendo la stessa superficie. I risultati
statici seguenti restano storici e non validano la posizione reale.
Prima di altro movimento occorre chiarire la coerenza ruote/lidar e la
calibrazione. Dati e rosbag sul laptop in
`/home/gdj/workspaces/robocat_experiments/elevator_20261106_staging_01`.

Verifica statica sui dati reali già misurati, senza simulazione dinamica,
goal o movimenti. Modello locale separato in `robocat_elevator_geometry.yaml`;
non sono state aggiunte collisioni o nuove regole al Giskard attivo.
Il 6 novembre la normale di `elevator_doorway` è stata corretta dal fondo
misurato della cabina, imponendo il parallelismo varco/fondo. Centro in
odom inizialmente invariato; yaw corretto di circa −5,75°. File precedenti conservati
con suffisso `.before_parallel_correction`.
La successiva verifica aperta degli stipiti ha portato a spostare il centro
di 9 cm verso destra, mantenendo la yaw e trasformando le pareti senza
spostare le superfici fisiche. Backup `.before_centre_correction`.

## Percorso candidato

Coordinate nel frame `elevator_doorway`, x verso la cabina, y verso sinistra.
Posa ruote letta a robot fermo nel frame corretto: x circa −0,993 m,
y circa −0,050 m, yaw +17,62°. Questo è un punto iniziale storico, non va riutilizzato
alla cieca per una successiva esecuzione.

1. Allineamento esterno verso la posa di avvicinamento.
2. Avvicinamento di circa 54,5 cm fino a x=−0,45 m, y=0.
3. Rotazione sul posto, ancora fuori dal varco, per orientarsi verso la
   posa interna, yaw circa −5,06° rispetto al frame porta.
4. Rettifilo di circa 1,054 m fino a x=+0,60 m, y=−0,093 m, mantenendo
   l'orientamento del tratto. Nessuna rotazione finale in cabina proposta.

La destinazione è sulla linea centrale stimata della cabina, distinta dal
centro della porta. La traiettoria attraversa l'apertura leggermente in
diagonale, senza cambiare orientamento durante l'attraversamento.
Questa scelta è una proposta da verificare, non un'istruzione di moto.

## Ingombro e controllo campionato

Bounding box del modello Robocat nel frame base:
x da −0,29650 a +0,15761 m, y da −0,20020 a +0,20020 m.
Controllate 189 pose lungo traslazioni e rotazioni, passo massimo 1 cm
e 1°. Screening con bounding box allargato di 10 cm in ciascuna direzione
locale: include conservativamente un buffer circolare di 10 cm attorno
all'ingombro nominale. Questo parametro non è una misura dell'errore reale
di tracking, calibrazione, slittamento o accessori fisici.

Le porzioni del rettangolo che entrano in cabina vengono confrontate
con i semipiani delle pareti misurate; le sezioni all'intersezione con
x=0 vengono confrontate con l'apertura nominale y=±0,45 m. Le ante sono
assunte aperte. Le superfici di ingresso e le loro profondità effettive
richiedono conferma prima di un'esecuzione. Il calcolo campionato non è
una certificazione continua dell'intera traiettoria.

| Superficie | Minimo calcolato sull'ingombro nominale |
| --- | ---: |
| Stipiti nel piano nominale del varco | 20,9 cm |
| Parete sinistra | 28,6 cm |
| Parete destra | 33,1 cm |
| Fondo | 84,0 cm |

Nessuna violazione del modello con ingombro allargato nelle pose
campionate. Alla posa finale la parte posteriore del bounding box è
circa 28,7 cm oltre il piano nominale della soglia.

## Lavoro necessario prima del moto

- Integrare la geometria scelta nel goal e nelle verifiche effettive,
  aggiornando il punto di partenza dalla posa ruote corrente.
- Verificare il tratto esterno e i ritorni attuali lungo il percorso:
  questa verifica comprende cabina e apertura, non gli ostacoli della lobby.
- Definire la gestione della porta durante il passaggio, delle persone,
  dei dati mancanti e della cancellazione: `PassageClear` osserva un
  segmento nel piano laser, non garantisce da solo queste funzioni.
- Confermare fisicamente condizioni della soglia, gap e dislivello,
  non misurati dal lidar orizzontale.

Nessuna esecuzione hardware autorizzata da questo documento. Il margine
calcolato dipende dall'approssimazione della posa del varco e delle pareti.

Risultati numerici in `robocat_entry_geometry_report.json`. Script di
calcolo temporaneo nel container:
`/tmp/robocat_entry_geometry_check.py`. Nessuna dipendenza installata:
il calcolo usa operazioni geometriche su rettangoli e semipiani, Python
e YAML già disponibili nel container.
