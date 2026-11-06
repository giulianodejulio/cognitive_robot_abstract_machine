# Risultati odometrici del 6 novembre 2026

Prove rettilinee con Giskard, feedback wheel, goal cartesiano:

| Moltiplicatori raggi | Goal | Distanza fisica comunicata dall'operatore |
|---|---:|---:|
| 1,0 | 0,5 m | circa 1,00 m |
| 1,5 | 0,5 m | circa 0,65 m |
| 2,0 | 0,5 m | circa 0,50 m |

Con 2,0 anche 0,25 m e 1 m sono risultati soddisfacenti secondo l'operatore.
La prima impostazione a caldo di 1,5 non aveva modificato la scala:
la correzione ha avuto effetto dopo riconfigurazione del controller.

Rotazioni, mantenendo i moltiplicatori raggi a 2,0:

| Goal | Moltiplicatore interasse | Ruote | Laser |
|---|---:|---:|---:|
| +90° | 1,0 | +92,0746° | +75,2768° |
| +90° | 1,22 | +91,5746° | +92,7762° |
| -90°, tolleranza 2° | 1,22 | -91,6666° | -91,1070° |

I JSON conservano i campioni simultanei e i risultati. La precedente
`robocat-laser-turn-measurement.json` contiene la sola misura DLO +74,93°.
Gli angoli sono differenze di orientamento con unwrap, non letture yaw
assolute. Il criterio di fine richiede posa stabile per circa 5 secondi
(4 secondi nella prima prova laser).

La DLO è un riferimento provvisorio, non una ground truth indipendente.
Durante le ultime rotazioni ha stimato 1,68 e 2,72 cm di traslazione,
mentre le ruote stimavano quasi zero: discrepanza ancora aperta.
