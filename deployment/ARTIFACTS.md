# Dati sperimentali fuori da Git

Archivio locale creato al salvataggio:

`/home/gdj/workspaces/tiago_ws/cram_real_robot_artifacts_20261106.tar`

Contiene l'intera directory `robocat_experiments/` del laptop: rosbag,
risultati odometrici, registrazione della mappatura, campioni della porta,
replay e dati della prima prova di avvicinamento. Non contiene l'immagine
Docker o credenziali. Il checksum è in
`deployment/provenance/artifact-archive.sha256`.

Questo archivio è una seconda copia locale, non un backup remoto.
Per conservarlo fuori dal laptop copiarlo su un supporto o servizio dati;
non è incluso nel push del fork.

Nel repository sono presenti la mappa PGM/YAML attuale, frame, calibrazioni e
risultati compatti. Il vecchio grafo SLAM e la mappa del 6 novembre sono nella
cronologia Git, rimossi dal checkout corrente perché superati. I dati originali restano anche in
`/home/gdj/workspaces/robocat_experiments/`, senza spostamenti o cancellazioni.

Per estrarre l'archivio in una directory scelta:

```bash
mkdir -p /percorso/ripristino
tar -xf /percorso/cram_real_robot_artifacts_20261106.tar -C /percorso/ripristino
```

I rosbag con comandi della base devono essere riprodotti su un dominio ROS
isolato, senza bridge verso il robot. Vedere la documentazione di replay.

## Prove e calibrazione del 18 novembre 2026

I rosbag recenti restano locali in `tmp/robocat-entry-bags/` nel workspace attivo.
La cattura del frame cabina e le analisi sono in
`/home/gdj/workspaces/robocat_experiments/cabin_frame_20261118/`.
Questi dati non sono inclusi nel vecchio archivio del 6 novembre né nel push.
La trasformazione finale e la qualità della stima sono versionate in
`config/robocat_entry/cabin_labora_floor.yaml` e `doc/robocat_cabin_frame.md`.
