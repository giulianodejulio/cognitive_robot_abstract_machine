# Robocat: verifica lidar del varco ascensore, 6 novembre 2026

Prove sul robot reale fermo, senza goal o comandi di movimento. Repository
originale con modifiche locali; clone milestone aggiornato non utilizzato.
Valutato direttamente `PassageClear.on_tick` con il modello Robocat e
`RobocatLidarSource` alimentato da `/scan` reale. Nessun ray casting simulato.

## Geometria e significato della prova

L'operatore indica robot a circa 1 m dalla soglia, approssimativamente
centrato e allineato, apertura circa 90 cm. Porta telescopica con ante
su piani a diversa profondità. Non è stato eseguito un fit di piano unico.

La superficie centrale a porta chiusa è osservata a circa 0,91 m dal laser;
aperta, i ritorni centrali sono a circa 2,50 m. Il segmento di controllo
è collocato a x=0,91 m nel frame laser, senza dipendere dalle TF mancanti
nella prima verifica. Questo è un riferimento approssimativo calibrato
localmente, non una stima automatica della posa dell'ascensore.

## Intera apertura e corridoio centrale

Il segmento largo 90 cm centrato sul robot risulta bloccato anche a porta
aperta: circa 10 raggi sul bordo sinistro, a y circa +41–45 cm, incontrano
una superficie compatibile con lo stipite. A porta chiusa sono bloccati
278–288 raggi su 292, distribuiti quasi su tutta la larghezza.

La larghezza trasversale del bounding box del modello Robocat è circa
40,04 cm. Verificato quindi un corridoio centrale largo 60 cm, estremi
(0,91; −0,30; 0) e (0,91; +0,30; 0) nel frame laser, con tolleranza
standard `PassageClear` 5 cm. I circa 10 cm per lato rispetto al modello
sono il margine geometrico del test, non una tolleranza di navigazione
validata né una misura degli accessori fisici fuori dal modello.

- Porta aperta: `TRUE` in 30/30 scansioni, 202 raggi attraversano il
  segmento, circa 196–198 ritorni finiti, nessun NaN nei campioni riportati.
  Alcuni raggi hanno +inf, che il monitor originale tratta come passanti.
  Minimo margine dei ritorni finiti oltre il segmento circa 0,370 m,
  prima di sottrarre la tolleranza di 5 cm.
- Porta chiusa: `FALSE` in 30/30 scansioni; nei campioni iniziale/finale
  circa 195–196 raggi sono bloccati su 202.

## Transizione continua con aperture manuali

Monitor continuo, aggiornamento su ogni nuova scansione. Tempi relativi
all'inizio della finestra, non ritardi rispetto al movimento fisico
della porta (non strumentato):

- 0,920 s: `FALSE`, porta inizialmente chiusa.
- 27,220 s: `TRUE`, apertura confermata dall'operatore.
- 53,920 s: `FALSE`, richiusura confermata dall'operatore. Già 4 raggi
  bloccati bastano per tornare a `FALSE`, senza attendere tutti i raggi.

Non inferire la latenza percettiva dai tempi dei messaggi in chat.

## Collegamento e limiti

Durante la preparazione si è persa la ricezione lidar nel container,
mentre il robot continuava a pubblicare `/scan` e PAL restava attivo.
Riavviato soltanto il bridge DDS; ricezione ripristinata.
Il bridge è stato lasciato in modalità **feedback-only**, senza inoltro
`/cmd_vel`. Nessun riavvio PAL né attivazione di controller hardware.

Validata soltanto l'osservazione lidar di un corridoio noto nella posa
attuale. Non validati riconoscimento autonomo dell'ascensore, localizzazione
automatica degli stipiti, spazio della cabina, soglia/gap, persone fuori
dal piano del lidar, controllo durante ingresso o ingresso autonomo.

Dati temporanei nel container `aicor-cram-demo`:
`/tmp/robocat-passage-60cm-open.json`,
`/tmp/robocat-passage-60cm-closed.json`,
`/tmp/robocat-passage-transitions.json` (al termine della finestra continua).

## Frame calibrato e vista RViz

Misurate 30 scansioni delle ante chiuse. Il cambio di profondità è usato
come riferimento approssimativo del centro, secondo l'indicazione
dell'operatore: non è una misura della soglia a pavimento. Modello a due
profondità, salto circa 3,9 cm, distanza dal laser circa 0,890 m e dal
riferimento base circa 1,003 m, scostamento laterale circa −1,6 cm.
Normale verso cabina stimata circa −3,2° rispetto alla base, sensibile
alla finestra di fit (circa −2°..−4,4°): calibrazione iniziale approssimativa.

La posa composta con le ruote è salvata in `robocat_elevator_doorway.yaml`:
x=1,664435 m, y=20,183488 m, yaw=0,386939 rad in `odom` ruote.
È valida per questa origine odometrica: ricalibrare dopo reset odometrico.

Verifica successiva: albero interno `odom -> base_footprint -> laser_link`
aggiornato dalle ruote, con `elevator_doorway` fissato sotto `odom` e
segmento centrale ±0,30 m definito nel frame porta. Il monitor originale
restituisce `FALSE` in 30/30 scan con porta chiusa. Nessun controllo o
integrazione in un goal Giskard eseguita in questa verifica.

`experiments.robocat_doorway_view` trasferisce esclusivamente osservazioni
dal dominio 103 a una vista isolata 104. Esclude le TF native che collocano
`odom` e `base_footprint`, pubblica invece `odom -> base_footprint` dalle
ruote e `odom -> elevator_doorway` dalla calibrazione. Il dominio di
controllo 103 e il robot non ricevono queste TF sostitutive.
Quattro test offline passati: frame fisso, aggiornamento ruote indipendente,
esclusione TF DLO e rifiuto frame odometrici inattesi.

```bash
# Nel container con ambiente ROS caricato; crea soltanto la vista, nessun goal.
export ROS_DOMAIN_ID=103
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/cram/doc/robocat_view_cyclonedds.xml
/opt/ros/cram-env/bin/python -m experiments.robocat_doorway_view \
  --calibration /workspace/cram/doc/robocat_elevator_doorway.yaml
```

RViz deve usare dominio 104, lo stesso XML loopback e
`doc/robocat_doorway.rviz`. Screenshot reale in `robocat_doorway_rviz.png`:
RobotModel alpha 0,5, scan ciano, assi base e varco. `odom` è il fixed frame;
la sua origine è lontana, fuori dallo zoom locale su robot/porta.
Per questa acquisizione RViz e relay sono nel container ausiliario
`robocat-doorway-view`, limitato a 1400 MiB e due CPU, con rendering software.
Può essere arrestato con `docker stop robocat-doorway-view`, senza fermare
PAL, bridge DDS o framework nel container principale.

## Pareti della cabina a porta aperta

Dopo la rotazione reale nominale 10° (ruote: circa 8,65°), stesso frame
calibrato senza ricalibrazione: porta chiusa `FALSE` 30/30, aperta `TRUE`
30/30 sul varco centrale. In successive verifiche dell'interno sono stati
osservati ritorni a y circa +0,30 m e x circa 0,58–0,80 m, che intersecano
il bordo sinistro del rettangolo diagnostico largo 60 cm. L'operatore
identifica la superficie come parete sinistra della cabina; non trattarla
automaticamente come ostacolo imprevisto né richiedere che un rettangolo
arbitrario sia interamente vuoto se incontra la struttura della cabina.

Nuova acquisizione: 30 scan a porta aperta e robot fermo, tre superfici
visibili coerenti con pareti laterali e fondo. Fit robusto su mediane
spaziali nel piano laser, coordinate nel frame porta attuale:

- Sinistra: y circa 0,359 − 0,0967 x (metri).
- Destra: y circa −0,721 − 0,1156 x.
- Fondo: x circa 1,625 + 0,1036 y.
- Larghezza stimata perpendicolarmente alle pareti: circa 1,07 m.
- Fondo lungo la linea centrale stimata: x circa 1,59 m dalla soglia
  approssimativa; non una misura a pavimento della profondità.
- Asse delle pareti circa −6,1° rispetto all'attuale `elevator_doorway`.
  Differenza tra direzioni delle due pareti circa 1,1°.
- Linea centrale estrapolata a x=0: y circa −0,18 m; a x=0,60 m:
  y circa −0,245 m. Questo può descrivere una cabina decentrata rispetto
  al varco oppure risentire della calibrazione: non spostare il frame
  della porta automaticamente per farlo coincidere con il centro cabina.

La continuità dei ritorni laterali lungo x e il parallelismo approssimato
supportano l'interpretazione come parete. Misura 2D locale, non una
ricostruzione completa: superfici vetrate, oggetti fuori piano, soglia,
gap e dislivello non sono validati. Nessun movimento o modifica del frame
durante questa acquisizione. Prima dell'ingresso rappresentare pareti
e varco separatamente e verificare ingombro e margini lungo il percorso.

Dati nel container principale: `/tmp/robocat-cabin-geometry-samples.json`
e `/tmp/robocat-cabin-geometry-estimate.json`. La vista screenshot è stata
fermata per non prolungare la preparazione grafica; non usarne eventuali
annotazioni temporanee come classificazione semantica definitiva.

## Correzione del parallelismo varco/fondo

Successiva misura reale a robot fermo: 30/30 scan a porta aperta.
L'operatore indica il vincolo fisico di porta parallela al fondo. Normale
stimata dal fondo: correzione di −5,753° rispetto al frame precedente.
Centro conservato; nuova yaw odometrica 0,2865285581 rad. Le rette delle
pareti sono trasformate coerentemente, senza spostare le superfici
misurate. Il fondo ha esattamente slope zero nel modello vincolato.
Le direzioni laterali residue sono circa +0,24° e −0,81°, compatibili
con l'approssimazione locale. Geometria e percorso ricalcolati nei file
dedicati; versioni precedenti conservate come backup.

Questa correzione usa il fondo per l'orientamento: non dimostra una nuova
misura precisa del centro soglia. Le ante restano fisicamente su due
profondità e il riferimento arancione non deve sovrapporsi a entrambe.
Il confronto delle ante chiuse con il vincolo di parallelismo deve ancora
essere verificato dal vivo; nessuna nuova acquisizione chiusa richiesta
o movimento eseguito durante questa correzione.

RViz usa `TopDownOrtho`; i segmenti di varco e pareti sono alla quota
laser 0,29771 m per eliminare lo spostamento prospettico tra scan e modello.
La proposta resta esclusivamente geometrica, senza goal Giskard inviato.

Verifica chiusa successiva: 30 scan, robot fermo. Separando le ante sul
salto effettivamente osservato, profondità mediane circa −1,54 cm e
+2,48 cm rispetto al riferimento, salto locale circa 3,88 cm. Normali
residue circa −1,79° e +1,22°, RMS dei fit circa 5 mm. Una suddivisione
arbitraria a y=0 mescolava le due ante e produceva un falso scarto vicino
a 9°: non usarlo come stima dell'orientamento della porta.

Il salto di profondità risulta ora a y circa −9 cm nel frame corretto.
Non interpretare automaticamente la giunzione delle ante come centro
esatto del varco: l'operatore l'aveva indicata come riferimento approssimato.
Prima dell'ingresso occorre verificare i due stipiti a porta aperta,
separando precisione del centro, geometria delle ante e possibili errori
della posa ruote. Non è stata applicata una traslazione automatica del
frame; i margini della proposta restano nominali rispetto alla calibrazione.
Dati: `/tmp/robocat-closed-parallel-check.json`,
`/tmp/robocat-leaf-depth-profile.json`,
`/tmp/robocat-closed-separated-leaves.json` nel container principale.

Verifica aperta successiva: 30 scan, profondità centrale mediana 1,610 m
nel frame doorway, compatibile con il fondo visibile. Nei punti delle
spallette a profondità x tra 0,14 e 0,23 m, bordo destro circa
y=−0,54 m e bordo sinistro circa y=+0,35…0,36 m. L'apertura indicativa
è quindi circa 0,89…0,90 m, con centro circa y=−0,09 m rispetto al
frame attuale. È una stima locale dai ritorni delle spallette, non una
misura metrologica dello spazio libero lungo tutto lo spessore della porta.
La concordanza con la giunzione delle ante sostiene uno scarto del centro;
non deriva dalla sola proiezione angolare dei raggi sul piano x=0.
Nessuna nuova traslazione della calibrazione applicata in questa verifica:
prima di utilizzarla per l'ingresso vanno aggiornate coerentemente anche
le coordinate delle pareti e la proposta geometrica. Dati grezzi:
`/tmp/robocat-door-jamb-samples.json` nel container principale.

Correzione del centro applicata su richiesta dell'operatore: traslazione
di −0,09 m lungo y del frame precedente, yaw invariata. Nuova origine
odom: x=1,6898714394 m, y=20,0971568281 m. Intercette delle pareti laterali
aumentate di 0,09 m; fondo invariato perché parallelo al varco. Verificata
l'invarianza delle coordinate odometriche di punti fisici dopo il cambio
di riferimento. La posa iniziale storica è trasformata nello stesso modo;
il punto di staging rimane centrato nel nuovo varco e il percorso viene
quindi ricalcolato. Screening statico: 189 pose, nessuna violazione del
modello con ingombro allargato di 10 cm. Nessun movimento eseguito.

Successiva prova reale di solo avvicinamento: rotazione pura −12,317°,
poi goal rettilineo 0,54512 m, massimo 0,1 m/s. Entrambi SUCCESS;
spostamento ruote finale circa 0,566 m, latenza richiesta/primo moto
circa 0,529 s. L'operatore segnala chassis parzialmente dentro la cabina,
in contrasto con la stima ruote x=−0,430 m nel frame porta. Annullata
qualsiasi prosecuzione, richiesta cancellazione e verificato arresto.
Nessun goal ulteriore di allineamento o ingresso inviato.

Misura ferma a porta aperta: il piano candidato del fondo ha intercetta
x=1,0602 m contro 1,6146 m del modello, pendenza −0,10444, RMS circa
6,4 mm. Se è lo stesso fondo, allinearlo richiede circa +0,560 m lungo x
e −5,96° di rotazione. Non è una nuova ground truth: associazione della
superficie, scala/geometria ruote, geometria laser e calibrazione restano
da verificare. Non applicare una compensazione automatica ai sensori.
La proposta geometrica non è validata per l'esecuzione.
Rosbag separati `view` (scan, TF ruote, URDF, marker in inglese) e `control`
(velocità, ruote, giunti, interblocchi, action feedback/status, rosout),
con copie dei log e della configurazione, sono salvati sul laptop in
`/home/gdj/workspaces/robocat_experiments/elevator_20261106_staging_01`.
Il rendering RViz è stato fermato durante il test per il carico CPU/RAM.
