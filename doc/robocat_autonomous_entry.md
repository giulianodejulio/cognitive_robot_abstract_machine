# Robocat: passaggio Nav2–Giskard

## Primo passo: ingressi del mux distinti

Verifica sul robot del 6 novembre 2026: Nav2 (`controller_server`,
`behavior_server`, `global_behavior_server`) e DDS Router pubblicano tutti
`/cmd_vel`. Il mux li vede come un unico ingresso `navigation`, priorità 10.
Questa configurazione non arbitra le due sorgenti separatamente.

È preparato, ma **non installato sul robot**, il frammento
`robocat_mux_snm_override.yaml`. Aggiunge solo `topics.snm`, topic `/snm_vel`,
timeout 0,5 s, priorità 20. Non sostituisce gli ingressi nativi o i lock
joystick (100), assisted teleop (200) e ricarica (210).

La successiva ispezione del launch installato ha confermato che questo mux
non usa `get_pal_configuration`: copiare il frammento in `~/.pal/config`
non basta. Il modulo `twist_mux` avvia `robocat_bringup twist_mux.launch.py`,
che passa direttamente il file
`/opt/pal/alum/share/robocat_bringup/config/twist_mux/twist_mux_topics.yaml`.
Per la prima installazione aggiungere a quel file solo il blocco `snm`
sotto `topics`, conservando un backup. Riavviare solo `pal module restart
twist_mux`, a robot fermo e senza goal. Un aggiornamento del pacchetto può
annullare questa modifica: verificarla prima dei test.

Controllare con `ros2 param dump /twist_mux` che `navigation` rimanga su
`cmd_vel` con priorità 10, `snm` sia su `snm_vel` con priorità 20 e tutti
i lock originali siano presenti. Dopo il riavvio dei launcher, verificare
con `ros2 topic info /cmd_vel --verbose` e `ros2 topic info /snm_vel --verbose`
che il router pubblichi solo sul secondo e il mux sottoscriva entrambi.

Dopo aver applicato e verificato l'override, i comandi previsti sono:

Host, sostituendo il bridge precedente a robot fermo:

```bash
bash scripts/launch_robocat_dds_router.sh --control --command-topic /snm_vel
```

Container, sostituendo il launcher precedente a robot fermo:

```bash
bash scripts/launch_robocat_giskard.sh --feedback wheel \
  --max-linear 0.3 --max-angular 0.2 --command-topic /snm_vel
```

Le opzioni sono esplicite: il comportamento preesistente su `/cmd_vel`
rimane il default. Il bridge inoltra solo il topic di comando selezionato,
dal laptop al robot, volatile, profondità 1 e lifespan 250 ms.
Il sender `robocat-goal` e la cancellazione non cambiano.

Priorità differenti assicurano l'arbitraggio dei messaggi, ma non impediscono
che un goal Nav2 ancora attivo riprenda a comandare quando Giskard tace.
L'orchestratore deve quindi possedere e attendere l'esito dei propri goal;
nessun nuovo goal Nav2 deve essere attivo durante l'ingresso.
Verificare inoltre che Giskard inattivo non trasmetta continuamente zeri,
che altrimenti avrebbero priorità su Nav2.

## Requisito per l'ingresso automatico

Il solo `SUCCEEDED` di Nav2 non autorizza l'ingresso. La sequenza richiesta è:

1. Raggiungere `elevator_staging` con Nav2 e confermare l'arresto misurato.
2. Attendere una percezione recente e stabile di porta aperta e corridoio
   d'ingresso libero, riferita al frame calibrato dell'ascensore.
3. Inviare a Giskard il goal rettilineo relativo, inizialmente 1,6 m come
   nella prova manuale. La distanza non è una geometria valida per ogni ascensore.
4. Continuare a monitorare durante il movimento: porta in chiusura, ostacoli,
   TF mancanti o percezione scaduta richiedono cancellazione e arresto;
   non devono essere interpretati come passaggio libero.
5. Non ripartire automaticamente dopo una cancellazione: serve rivalutare
   la posa raggiunta e il percorso residuo.

Il monitor esistente `PassageClear` verifica un segmento lidar; da solo non
dimostra che l'intero corridoio sia libero né che i dati siano recenti.
Va riutilizzato con geometria coerente con la mappa e con controlli espliciti
di freschezza, disponibilità TF e ingombro. Il vecchio frame calibrato in
odom non va riusato dopo riavvii come se fosse quello della nuova mappa.

Questo primo passo prepara il routing; non implementa né avvia ancora la
sequenza autonoma o la sorveglianza continua della porta. Le prove fisiche
restano eseguite dall'operatore. Per le prove successive registrare scan,
TF/static TF, entrambe le odometrie, modello, stati di sicurezza,
comandi distinti e risultati delle azioni, per il replay isolato in RViz.
