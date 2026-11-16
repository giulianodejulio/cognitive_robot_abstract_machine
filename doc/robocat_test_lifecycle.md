# Chiusura delle sessioni temporanee Robocat

L'infrastruttura persistente `robocat-infra` contiene preparazione del controller,
DDS Router e Giskard. `robocat-elevator` contiene soltanto il terminale SNM e RViz.
La chiusura della sessione temporanea non deve arrestare l'infrastruttura.

## Nuove sessioni

Con l'infrastruttura già pronta, dal laptop:

```bash
bash scripts/robocat-byobu
```

Il terminale SNM e i processi che vi vengono avviati ereditano un identificatore
univoco. Il processo del terminale viene registrato nel container in
`/tmp/robocat-test.XXXXXX.snm.pid`. Sul laptop la registrazione rimane in
`<directory-infrastruttura>/test_sessions/robocat-test.XXXXXX.active`.

Alla chiusura del terminale o della sessione Byobu:

1. Il wrapper host avvia la chiusura del solo gruppo di processi identificato.
2. I processi Python della sessione ricevono `SIGINT`, come con `Ctrl+C`.
   Questo raggiunge anche gli script avviati dalla shell, non soltanto la shell.
   La cancellazione Nav2/Giskard rimane responsabilità dei client esistenti;
   non viene inviato un cancel-all e non viene modificato lo script Nav2-only.
3. Si attende la loro uscita prima di chiudere shell e trasporti residui.
4. Soltanto quando non rimangono processi identificati si rimuove la registrazione.
   Giskard persistente, DDS e processi con identificatori diversi restano attivi.

La chiusura concede al massimo 20 secondi ai processi della sessione: è un
limite nuovo del ciclo di vita del terminale, non un aumento dei timeout di
navigazione, della tolleranza ai feedback scaduti o della cancellazione del goal.
Non viene applicato un `SIGKILL` per trasformare una cancellazione incompleta in
un successo apparente.

I log della chiusura sono sul laptop, nella directory della sessione temporanea:
`/tmp/robocat-test.XXXXXX/cleanup.log`. Non sono una conferma di arresto fisico.

## Procedura operativa

Durante una prova, usare prima `Ctrl+C` nello script, attendere l'esito e
verificare fisicamente che il robot sia fermo. Solo dopo chiudere la sessione:

```bash
byobu-tmux kill-session -t robocat-elevator
```

Attendere la chiusura supervisionata prima di creare la sessione successiva.
`F6` scollega soltanto la visualizzazione e lascia la richiesta supervisionata
attiva. Non interrompe i processi.

Se la chiusura non è confermata, la registrazione resta presente e vengono
bloccati sia una nuova sessione temporanea sia `robocat-infra stop`. Controllare
il robot, `cleanup.log` e i processi prima di riprovare. Non eliminare la
registrazione per aggirare il controllo e non assumere che `navigation.close()`
o l'uscita del client certifichino da soli l'arresto delle ruote.

## Sessioni già aperte prima della modifica

Non ereditano retroattivamente l'identificatore e la gestione di chiusura.
Terminare manualmente le richieste con `Ctrl+C`, verificare l'arresto e uscire
dalla shell SNM con `exit`; poi chiudere e ricreare solo `robocat-elevator`.
Non occorre ripetere `APPLY`, riavviare Giskard o fermare DDS.
