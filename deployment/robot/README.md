# Configurazione robot: osservata e desiderata

`observed/` contiene le letture del 6 novembre 2026, senza modifiche al robot.
I file `.log` sono output grezzi, con eventuali messaggi iniziali PAL/ROS:
non usarli direttamente come file YAML di parametri.

`desired/robocat_wheel_calibration.yaml` conserva la compensazione provata.
Il raggio e l'interasse nominali coincidono con l'URDF. I moltiplicatori
compensano le discrepanze misurate, ma la causa originale resta da diagnosticare.

Il controller installato, diff_drive_controller 4.37.0, rilegge questi
parametri in configurazione. Un `ros2 param set` riuscito non basta per
applicarli al controllo. A robot fermo, senza goal o teleop, la procedura
utilizzata è stata:

1. Disattivare solo `mobile_base_controller`.
2. Portarlo a `unconfigured` tramite cleanup.
3. Caricare i parametri desiderati e verificarli.
4. Configurare il controller portandolo a `inactive`, poi riattivarlo.
5. Verificare feedback fresco e velocità nulla prima di nuovi goal.

Non eseguire questa procedura come parte automatica dell'avvio applicativo.
Il file desiderato non è stato installato nella configurazione persistente PAL.

Per il mux, il file letto realmente dal launch è
`/opt/pal/alum/share/robocat_bringup/config/twist_mux/twist_mux_topics.yaml`.
Il frammento `doc/robocat_mux_snm_override.yaml` è ancora proposto: copiarlo
in `~/.pal/config` non lo attiva. Vedere `doc/robocat_autonomous_entry.md`.

Current Byobu/SNM setup supersedes the historical procedure above: see [ROBOCAT_CURRENT.md](../ROBOCAT_CURRENT.md).
