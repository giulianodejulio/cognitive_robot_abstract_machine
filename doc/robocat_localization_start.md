# Posa iniziale ripetibile: localization_start

Waypoint registrato in Eulero il 13 novembre 2026:
`labora_demo/labora_floor/localization_start`.
La copia locale è `doc/robocat_localization_start.yaml`; non modifica
`elevator_staging` né i pixel della occupancy grid.

La posa di `base_footprint` acquisita in `map`, con feedback sincronizzato e ruote
ferme, è x=16.683926863686825 m, y=-27.721608933207687 m,
yaw=0.1936182952543307 rad (circa 11.09 gradi).
È la localizzazione AMCL/TF osservata, non una misura esterna di ground truth.

Segnare sul pavimento almeno due riferimenti riproducibili del telaio e il verso
anteriore, così da ripristinare sia posizione sia orientamento. Usare lo stesso
piano e la stessa mappa: fingerprint
`3ba2bccaa15358b35cd0539ba59ddc991c5a8f416e6279e5df5973440b35f0ae`.

## Sessioni successive

1. Riportare fisicamente il robot sui riferimenti segnati, senza goal attivi.
2. Caricare la stessa mappa e avviare la localizzazione.
3. Nel terminale **del robot**, con ambiente Humble/PAL, dominio 102 e CycloneDDS,
   inizializzare AMCL con il file già copiato sul robot:

```bash
ros2 service call /set_initial_pose nav2_msgs/srv/SetInitialPose \
  "$(cat /home/pal/robocat_elevator_mapping_20261109/localization_start_initial_pose.yaml)"
```

4. Verificare in RViz che la scan coincida con la mappa prima di autorizzare
   navigazione o handover. Non è necessario cliccare una posa approssimativa.

Il comando modifica la stima di localizzazione, non invia un goal di movimento.
Non eseguirlo quando il robot è altrove o una navigazione è in corso.
La richiesta conserva la covarianza AMCL acquisita; la precisione del
riposizionamento manuale resta da verificare sperimentalmente.

Il file della richiesta è conservato anche in
`doc/robocat_localization_initial_pose.yaml`. La sola creazione del waypoint
non inizializza automaticamente AMCL. L'avvio Byobu/RViz non è stato modificato
per imporre questa posa a ogni apertura: si evita di applicarla quando il robot
si trova, ad esempio, allo staging dopo una prova.

Durante la creazione non sono stati inviati goal né richieste di reset AMCL.
