# Stato del container al salvataggio

- `container-image.log`: nome e identificatore dell'immagine realmente usata.
- `python-packages.json`: nomi/versioni dei pacchetti installati, senza URL
  autenticati o configurazione pip privata. Il lockfile ripristinabile è `/uv.lock`.
- `pip-check.log`: gli avvisi su `docopt` e `pathlib` restano presenti;
  la documentazione del workspace descrive gli override docopt-ng/stdlib.
- `dpkg-image.tsv` e `dpkg-runtime.tsv`: inventari dei pacchetti Debian.
- `dpkg-added-or-changed.tsv`: versioni presenti nel container e non nella
  sua immagine di base; include PlotJuggler e dipendenze aggiunte.
- `ros-source-commits.tsv`: commit dei checkout ROS installati nel container.

L'inventario descrive lo stato osservato, non esegue installazioni.
Non usare `pip install` indiscriminatamente al posto del lockfile uv.
Per ricostruire TIAGo seguire `scripts/setup_tiago_laptop.sh`; per il router
Robocat seguire `doc/robocat_dds_bridge.md`.

L'immagine di partenza contiene già le aggiunte TIAGo previste dal setup.
Il confronto dpkg è rispetto a quell'immagine locale, non direttamente a
`ros:jazzy` o alla Docker image originaria pubblicata da terzi.
