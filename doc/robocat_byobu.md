# Avvio Robocat con Byobu sul laptop

Byobu è installato sull'host. Docker continua a ospitare ROS Jazzy, CRAM/Giskard
e SNM; il client di configurazione controller gira in Humble sul robot via SSH.
Non viene montato il socket Docker nel container e non vengono copiati segreti.

L'infrastruttura persistente `robocat-infra` è separata dalla sessione temporanea
`robocat-elevator`. Per cancellazione e chiusura dei client vedere
[robocat_test_lifecycle.md](robocat_test_lifecycle.md).

## Avvio

Sul **terminale del laptop**, con Ethernet configurata per Robocat:

```bash
cd /home/gdj/workspaces/tiago_ws/cognitive_robot_abstract_machine
bash scripts/robocat-infra start
byobu-tmux attach-session -t robocat-infra
# Dopo preparazione riuscita, staccarsi con F6 e aprire la sessione temporanea:
bash scripts/robocat-byobu
```

Per vedere prima il piano senza avviare processi:

```bash
bash scripts/robocat-byobu --plan
```

La sessione si chiama `robocat-elevator`. Ripetendo il comando si rientra
nella stessa sessione: non si ripete la preparazione e non si duplicano i server.
Non vengono inviati goal di movimento all'avvio.

## Finestre dell'infrastruttura persistente

1. **prepare**: avvia il container se il suo comando è quello inattivo previsto
   (`sleep infinity`), rifiuta un Giskard o bridge precedente, prepara il socket
   SSH e legge stato controller e moltiplicatori. Per continuare bisogna digitare
   esattamente `APPLY`, dopo aver verificato robot fermo, nessun goal attivo,
   teleop disattivata, presa scollegata, operatore ed emergenza raggiungibile.
2. **dds**: attende la preparazione riuscita e avvia il bridge sull'host con
   l'inoltro di `/snm_vel`. Non inoltra le azioni Nav2 fra Humble e Jazzy.
3. **giskard**: attende preparazione e bridge, poi avvia solo il server nel
   container, con feedback ruote e comando `/snm_vel`. Non avvia il vecchio
   client persistente `robocat-goal`.
## Finestre della sessione temporanea

1. **snm**: usa il server persistente e attende una posa valida nella mappa, esegue il controllo senza movimento e lascia
   un terminale nel container con dominio 103 e DDS già impostati. Un eventuale
   blocco del controllo viene mostrato; non viene aggirato né trasformato in goal.
2. **robot**: apre automaticamente RViz sul robot con
   inoltro X11 verso il laptop, usando `waypoints.rviz`. Avviare Byobu da un
   terminale grafico del laptop. Impostare **2D Pose Estimate** nella mappa e
   verificare che scan e muri coincidano. Alla chiusura di RViz si apre il
   terminale SSH nativo.

Se manca la posa, SNM stampa `WAITING FOR LOCALIZATION` e continua ad
attendere senza inviare goal. Quando la TF sincronizzata è disponibile stampa
`LOCALIZATION READY` e riprende i controlli. Questo verifica la disponibilità
della posa, non la sua accuratezza: il controllo visivo resta necessario.
`Ctrl+C` interrompe l'attesa. Un ingresso mux `snm` mancante resta un blocco
distinto dall'inizializzazione di AMCL.

La finestra `snm` resta selezionata nella sessione temporanea. `F3`/`F4` cambiano
finestra; `F6` scollega il terminale dalla sessione, lasciando i processi attivi.

## Applicazione dei parametri

Dopo `APPLY`, un client nativo verifica robot corretto, emergenza e ricarica,
stato teleop, wheel odometry ferma da almeno un secondo, comandi di base
non nulli assenti, mux libero con diagnostica recente e stati delle azioni base.
Se il publisher dello stato Nav2 è presente ma non ha ancora emesso messaggi,
la conferma «nessun goal attivo» fornita con `APPLY` consente la preparazione.
Questa è una dichiarazione dell'operatore, non una verifica automatica di idle.
Un goal osservato attivo continua a bloccare la procedura. Senza publisher o
senza conferma esplicita lo stato mancante resta bloccante.
Non vengono cancellati goal altrui o rilasciati lock.

La sequenza è quella già usata sul robot:

1. controller `inactive`;
2. controller `unconfigured`;
3. aggiornamento atomico dei tre moltiplicatori `2.0`, `2.0`, `1.22`;
4. controller `inactive` tramite configure;
5. controller `active`;
6. lettura dei valori effettivi, verifica stato `active` e ritorno del feedback
   ruote fermo e recente.

Questo ciclo viene eseguito anche se la lettura iniziale mostra già i valori
desiderati: la lettura da sola non prova che siano stati applicati al controller.
Gli interlock vengono ricontrollati prima delle transizioni. Durante lo stato
unconfigured non si pretende l'odometria che il controller ha smesso di pubblicare;
torna obbligatoria per la verifica finale.

Al primo errore la preparazione termina e le finestre dipendenti non avviano
i rispettivi processi. Nessun riavvio PAL, reset driver o recupero automatico:
se il controller è rimasto inattivo/unconfigured, verificare lo stato prima di
una nuova prova. La preparazione iniziale richiede wheel feedback corrente;
non sostituisce il recupero di un controller già guasto o disattivato.

I sorgenti necessari vengono estratti in una directory temporanea sul robot,
eseguiti nell'ambiente nativo e rimossi al termine. La configurazione di fabbrica
non viene sovrascritta; i moltiplicatori restano parametri runtime.

## Limiti di velocità e SNM

```bash
bash scripts/robocat-infra start --max-linear 0.3 --max-angular 0.2
```

Questi valori configurano una **nuova infrastruttura**. Rientrare in una sessione già
esistente non modifica il server attivo. Il launcher continua a usare le stesse
verifiche del modulo descritto in [robocat_snm.md](robocat_snm.md).

Nella finestra `snm`, il primo comando utile rimane:

```bash
bash scripts/robocat-entry
```

Per l'ingresso vero servono poi misura staging–soglia, fingerprint della mappa,
porte mantenute aperte e comando esplicito, come documentato nella guida SNM.
Il semplice avvio di Byobu non autorizza né programma quel movimento.

## Arresto

`F6` **non arresta** i processi. Per terminare una prova e poi la sessione:

1. `Ctrl+C` nella finestra `snm` se c'è una richiesta attiva; attendere il risultato
   di cancellazione e verificare fisicamente l'arresto.
2. Dal laptop chiudere la sola sessione temporanea:

```bash
byobu-tmux kill-session -t '=robocat-elevator'
```

Non usare `kill-session` come sostituto della cancellazione di un goal in corso.
La sessione temporanea richiede la cancellazione dei suoi client, lasciando
Giskard e DDS attivi. Se la terminazione non è confermata, nuove sessioni e
l'arresto dell'infrastruttura restano bloccati. I dettagli e i log sono descritti
in [robocat_test_lifecycle.md](robocat_test_lifecycle.md).

A fine lavoro, dopo aver verificato l'arresto del robot e chiuso le sessioni
temporanee, usare `bash scripts/robocat-infra stop`: richiede la conferma `STOP`
e arresta soltanto i servizi appartenenti all'infrastruttura.

Se la preparazione fallisce prima dell'avvio di bridge/server, correggere il
problema e chiudere la sessione fallita prima di ricrearla: il semplice riaggancio
non riavvia le finestre terminate. Questo evita riconfigurazioni automatiche
mentre l'operatore sta leggendo l'errore.

## Verifiche software

Il launcher è stato provato con Byobu reale sull'host e finestre che eseguivano
solo `sleep`, in socket/configurazione temporanei separati: creazione delle cinque
finestre e riaggancio senza duplicazioni. I test pytest nel container verificano
anche quoting dei percorsi, conferma esplicita, ordine dei passi del controller,
blocchi per goal/comandi/lock, errori dei parametri e regressioni del modulo SNM.
Nessuna riconfigurazione o prova di movimento sul robot è stata eseguita durante
lo sviluppo di questo launcher.

Verifica del 12 novembre 2026: **87 test mirati superati** nel container,
con `--orm-build never`, un thread BLAS e limite di memoria virtuale di 2,5 GiB.
Byobu e le dipendenze aggiunte per la sua installazione temporanea sono stati
rimossi dal container dopo la scelta definitiva di eseguirlo sull'host.
