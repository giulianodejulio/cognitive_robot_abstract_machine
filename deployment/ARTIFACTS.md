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

Nel repository sono già presenti mappa PGM/YAML, grafo SLAM serializzato,
frame, calibrazioni e risultati compatti. I dati originali restano anche in
`/home/gdj/workspaces/robocat_experiments/`, senza spostamenti o cancellazioni.

Per estrarre l'archivio in una directory scelta:

```bash
mkdir -p /percorso/ripristino
tar -xf /percorso/cram_real_robot_artifacts_20261106.tar -C /percorso/ripristino
```

I rosbag con comandi della base devono essere riprodotti su un dominio ROS
isolato, senza bridge verso il robot. Vedere la documentazione di replay.
