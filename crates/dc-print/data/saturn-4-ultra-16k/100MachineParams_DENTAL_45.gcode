M5000 I0 A0 B230.000000      ;Corsa asse Z (mm)
M5000 I0 A0 C20.000000      ;Velocità massima motore asse Z (mm/s)
M5000 I0 A0 D0.000625000000  ;Distanza per passo del motore asse Z (mm/step)
M5000 I0 A0 E1      ;Direzione motore asse Z; 1 direzione positiva, -1 direzione negativa
M5000 I0 A0 F1      ;Modalità finecorsa asse Z; 0 finecorsa minimo, 1 finecorsa massimo, 2 doppio finecorsa
M5000 I0 A0 S1      ;Direzione di homing asse Z; 1 direzione positiva, -1 direzione negativa
M5000 I0 A0 P5.000000      ;Velocità del primo stadio di homing asse Z (mm/s)
M5000 I0 A0 R1.000000      ;Velocità del secondo stadio di homing asse Z (mm/s)
M5000 I0 A0 T0.000000      ;Distanza di movimento in direzione inversa prima dell'homing asse Z (mm); questo parametro serve principalmente, quando il finecorsa è montato lontano dallo zero fisico, a evitare che l'asse si schianti durante l'homing se si trova oltre il finecorsa
M5000 I0 A0 X5.000000      ;Distanza di ritorno in direzione inversa dopo il primo azionamento del finecorsa durante l'homing asse Z (mm)
M5000 I0 A0 Y0.450002      ;Posizione dello zero dell'asse Z
M5000 I0 A0 Z1      ;Impostazione dello zero Z e della posizione del finecorsa; 0: la posizione del finecorsa coincide con lo zero di Z, 1: la posizione del finecorsa è diversa dallo zero di Z
M5000 I5 A0 B0      ;Polarità finecorsa minimo asse Z; 0 si attiva a livello basso; 1 si attiva a livello alto
M5000 I5 A0 C0      ;Polarità finecorsa massimo asse Z; 0 si attiva a livello basso; 1 si attiva a livello alto
M5000 I5 A0 D7.000000      ;Accelerazione asse Z (mm/s^2)
M5000 I5 A0 E0.050000      ;Velocità iniziale asse Z (mm/s)
M5000 I6  A12      ;Corrente del motore asse X a riposo (0-31) : (N+1)/32 * 2.5 (A)
M5000 I6  B25      ;Corrente del motore asse X in funzionamento (0-31) : (N+1)/32 * 2.5 (A)
M5000 I6 C36.000000      ;Accelerazione asse X (gradi/s^2)
M5000 I6 D36.000000      ;Decelerazione asse X (gradi/s^2)
M5000 I6 E36.000000      ;Velocità iniziale asse X (gradi/s)
M5000 I6 P1      ;Valore del gradino per ogni variazione di corrente asse X (1-31)
M5000 I6 R1000      ;Intervallo di tempo tra una variazione di corrente e l'altra asse X (ms)
M5000 I6  F9      ;Corrente del motore asse Z a riposo (0-31) : (N+1)/32 * 1.915 (A)
M5000 I6  S18      ;Corrente del motore asse Z in funzionamento (0-31) : (N+1)/32 * 1.915 (A)
M5000 I7 E45      ;Temperatura target della resina (gradi Celsius) (intero)
M5000 I7 F45      ;Temperatura target di preriscaldo (gradi Celsius) (intero)
M5000 I7 S1440      ;Durata massima del preriscaldo (MIN) (intero)
M5000 I7 P35      ;Limite di tempo per singolo riscaldamento della vasca, timeout di un ciclo di riscaldamento (MIN) (intero)
M5000 I7 R1      ;Differenza di temperatura per l'azione di riscaldamento della vasca, tolleranza positiva ammessa (gradi Celsius) (intero)
M5000 I7 T5      ;Differenza di temperatura per l'azione di riscaldamento della vasca, tolleranza negativa ammessa (gradi Celsius) (intero)
M5000 I7 X0.000000      ;Valore di compensazione temperatura 1-10 (gradi Celsius) (virgola mobile)
M5000 I7 Y0.000000      ;Valore di compensazione temperatura 10-20 (gradi Celsius) (virgola mobile)
M5000 I7 Z0.000000      ;Valore di compensazione temperatura 20+ (gradi Celsius) (virgola mobile)
M5000 I0 A1 B140.000000      ;Angolo totale di distacco asse X (gradi)
M5000 I0 A1 C900.000000      ;Velocità massima motore asse X (gradi/s)
M5000 I0 A1 D0.007031250000  ;Angolo per passo del motore asse X (gradi/step)
M5000 I0 A1 E1      ;Direzione motore asse X; 1 direzione positiva, -1 direzione negativa
M5000 I0 A1 F1      ;Modalità finecorsa asse X; 0 finecorsa minimo, 1 finecorsa massimo, 2 doppio finecorsa
M5000 I0 A1 S1      ;Direzione di homing asse X; 1 direzione positiva, -1 direzione negativa
M5000 I0 A1 P36.000000      ;Velocità del primo stadio di homing asse X (gradi/s)
M5000 I0 A1 R36.000000      ;Velocità del secondo stadio di homing asse X (gradi/s)
M5000 I0 A1 T0.000000      ;Angolo di movimento in direzione inversa prima dell'homing asse X (gradi); questo parametro serve principalmente, quando il finecorsa è montato lontano dallo zero fisico, a evitare che l'asse si schianti durante l'homing se si trova oltre il finecorsa
M5000 I0 A1 X60.000000      ;Angolo di ritorno in direzione inversa dopo il primo azionamento del finecorsa durante l'homing asse X (gradi)
M5000 I0 A1 Y0.000000      ;Posizione dello zero dell'asse X
M5000 I0 A1 Z1      ;Impostazione dello zero X e della posizione del finecorsa; 0: la posizione del finecorsa coincide con lo zero, 1: la posizione del finecorsa è diversa dallo zero
M5000 I5 A1 B0      ;Polarità finecorsa minimo asse X; 0 si attiva a livello basso; 1 si attiva a livello alto
M5000 I5 A1 C0      ;Polarità finecorsa massimo asse X; 0 si attiva a livello basso; 1 si attiva a livello alto
M5000 I200 A36.000000      ;Asse X, primi N layer: velocità del primo stadio di riallineamento <alta> (gradi/s)
M5000 I200 B36.000000      ;Asse X, primi N layer: velocità del secondo stadio di riallineamento <bassa> (gradi/s)
M5000 I200 C36.000000      ;Asse X, primi N layer: velocità del primo stadio di distacco <bassa> (gradi/s)
M5000 I200 D36.000000      ;Asse X, primi N layer: velocità del secondo stadio di distacco <alta> (gradi/s)
M5000 I200 E0.000000      ;Asse X, primi N layer: angolo del secondo stadio di riallineamento (gradi)
M5000 I200 F0.000000      ;Asse X, primi N layer: angolo del primo stadio di distacco (gradi)
M5000 I200 S72.000000      ;Asse X, dopo N layer: velocità del primo stadio di riallineamento <alta> (gradi/s)
M5000 I200 P72.000000      ;Asse X, dopo N layer: velocità del secondo stadio di riallineamento <bassa> (gradi/s)
M5000 I200 R72.000000      ;Asse X, dopo N layer: velocità del primo stadio di distacco <bassa> (gradi/s)
M5000 I200 T72.000000      ;Asse X, dopo N layer: velocità del secondo stadio di distacco <alta> (gradi/s)
M5000 I200 X0.000000      ;Asse X, dopo N layer: angolo del secondo stadio di riallineamento (gradi)
M5000 I200 Y0.000000      ;Asse X, dopo N layer: angolo del primo stadio di distacco (gradi)
M5000 I201 A1      ;Modalità di riallineamento asse X (0: tramite finecorsa, 1: angolo fisso)
M5000 I201 B1      ;Modalità di distacco rotativo, 0: i due motori si muovono in sincrono, 1: prima il motore X arriva in posizione, poi parte Z, 2: prima si muove il motore Z, poi parte X
M5000 I201 E50      ;Valore N layer asse X
M5000 I210 A36.000000      ;S2 - Asse X, primi N layer: velocità del primo stadio di riallineamento <alta> (gradi/s)
M5000 I210 B36.000000      ;S2 - Asse X, primi N layer: velocità del secondo stadio di riallineamento <bassa> (gradi/s)
M5000 I210 C36.000000      ;S2 - Asse X, primi N layer: velocità del primo stadio di distacco <bassa> (gradi/s)
M5000 I210 D36.000000      ;S2 - Asse X, primi N layer: velocità del secondo stadio di distacco <alta> (gradi/s)
M5000 I210 E0.000000      ;S2 - Asse X, primi N layer: angolo del secondo stadio di riallineamento (gradi)
M5000 I210 F0.000000      ;S2 - Asse X, primi N layer: angolo del primo stadio di distacco (gradi)
M5000 I210 S144.000000      ;S2 - Asse X, dopo N layer: velocità del primo stadio di riallineamento <alta> (gradi/s)
M5000 I210 P144.000000      ;S2 - Asse X, dopo N layer: velocità del secondo stadio di riallineamento <bassa> (gradi/s)
M5000 I210 R144.000000      ;S2 - Asse X, dopo N layer: velocità del primo stadio di distacco <bassa> (gradi/s)
M5000 I210 T144.000000      ;S2 - Asse X, dopo N layer: velocità del secondo stadio di distacco <alta> (gradi/s)
M5000 I210 X0.000000      ;S2 - Asse X, dopo N layer: angolo del secondo stadio di riallineamento (gradi)
M5000 I210 Y0.000000      ;S2 - Asse X, dopo N layer: angolo del primo stadio di distacco (gradi)
M5000 I211 A1      ;S2 - Modalità di riallineamento asse X (0: tramite finecorsa, 1: angolo fisso)
M5000 I211 B1      ;S2 - Modalità di distacco rotativo, 0: i due motori si muovono in sincrono, 1: prima il motore X arriva in posizione, poi parte Z, 2: prima si muove il motore Z, poi parte X
M5000 I211 E50      ;S2 - Valore N layer asse X
M5000 I201 C40.000000      ;Velocità di controllo manuale asse X (gradi/s)
M5000 I201 D-10.000000      ;Posizione zero massima asse X, entità (gradi) di cui si può ancora proseguire il livellamento dopo l'homing
M5000 I202 A0.000000      ;Tempo di fermo T (secondi) dopo il rientro nei primi N layer locali (se 0 segue il file di slicing)
M5000 I1 A5.000000      ;Velocità di sollevamento a fine stampa (mm/s)
M5000 I1 B100.000000      ;Sollevamento di una distanza relativa dopo la fine della stampa (mm)
M5000 I1 C1      ;Modalità di sollevamento a fine stampa; 0 fino al punto più alto, 1 fino alla distanza relativa indicata, 2 nessuna azione
M5000 I1 D5.000000      ;Velocità di sollevamento in pausa di stampa (mm/s)
M5000 I1 E100.000000      ;Sollevamento di una distanza relativa dopo la pausa di stampa (mm)
M5000 I1 F1      ;Modalità di sollevamento in pausa di stampa; 0 fino al punto più alto, 1 fino alla distanza relativa indicata, 2 nessuna azione
M5000 I1 S5.000000      ;Velocità di sollevamento allo stop della stampa (mm/s)
M5000 I1 P100.000000      ;Sollevamento di una distanza relativa dopo lo stop della stampa (mm)
M5000 I1 R1      ;Modalità di sollevamento allo stop della stampa; 0 fino al punto più alto, 1 fino alla distanza relativa indicata, 2 nessuna azione
M5000 I1 T5.000000      ;Velocità di controllo manuale (mm/s)
M5000 I2 B200      ;Frequenza della ventola della sorgente luminosa
M5000 I2 C200      ;Frequenza della sorgente luminosa
M5000 I2 D200      ;Frequenza della luce di illuminazione
M5000 I2 E3000      ;Frequenza del buzzer
M5000 I3 B2      ;Modalità ventola della sorgente luminosa; 0 sempre accesa all'avvio, 1 spenta all'avvio, 2 segue lo stato di stampa, 3 segue lo stato di esposizione
M5000 I3 C2      ;Modalità della sorgente luminosa; 0 segue solo i parametri macchina, 1 segue solo il file di stampa, 2 combina file di stampa e parametri macchina
M5000 I4 B255      ;PWM ventola della sorgente luminosa (0-255)
M5000 I4 C255      ;PWM sorgente luminosa (0-255)
M5000 I4 D255      ;PWM luce di illuminazione (0-255)
M5000 I4 E100      ;Volume del buzzer (0-100)
M5000 I4 F0      ;Orientamento dello schermo (0-3), 0:0, 1:90, 2:180, 3:270
M5000 I4 S180      ;Tempo di ingresso nello screensaver (s)
M5000 I7 A80      ;Limite superiore di protezione temperatura (℃)
M5000 I7 B1      ;Esclusione del rilevamento temperatura; 0: esclusa, 1: non esclusa
M5000 I7 C1      ;Esclusione del rilevamento temperatura della vasca; 0: esclusa, 1: non esclusa
M5000 I7 D60      ;Limite superiore di protezione temperatura della vasca (℃)
M5000 I4 P5      ;Tempo di ingresso nello screensaver grigio (min)
M5000 I4 R0      ;Esclusione della stima della resina; 0: esclusa, 1: non esclusa
M5000 I4 T870.000000      ;Volume massimo della vasca (ml)
M5000 I4 X25.500000      ;Linea massima di anomalia nella fase di rilevamento effettivo della resina (mm)
M5000 I4 Y4.000000      ;Linea minima di anomalia nella fase di rilevamento effettivo della resina (mm)
M5000 I8 A50      ;Valore massimo del sensore di temperatura in modalità fabbrica (℃)
M5000 I8 B0      ;Valore minimo del sensore di temperatura in modalità fabbrica (℃)
M5000 I203 B4.500000      ;Velocità di movimento durante la calibrazione dello strain gauge (mm/s)
M5000 I203 C30000      ;Tempo massimo di calibrazione dello strain gauge (ms)
M5000 I203 D5000      ;Tempo di determinazione della quiete dello strain gauge (ms)
M5000 I203 E50      ;Soglia di determinazione della quiete dello strain gauge
M5000 I204 A45.000000      ;Posizione iniziale del rilevamento resina (mm); deve essere almeno superiore alla posizione strutturale del livello massimo per iniziare la misura
M5000 I204 B3.000000      ;Posizione finale del rilevamento resina (mm); si consiglia 1 mm o 2 mm. Non andare a contatto con il fondo: troppo vicino al fondo la tensione del film porta a errori di valutazione
M5000 I204 C0.050000      ;Passo del rilevamento resina (mm); si consiglia 0,05 mm: troppo piccolo allunga il tempo di rilevamento, troppo grande aumenta l'errore di stima
M5000 I204 D2.500000      ;Velocità di movimento durante il rilevamento resina (mm/s)
M5000 I204 E55      ;Soglia di attivazione del rilevamento resina; quando la tensione dovuta alla superficie del liquido durante la discesa supera questa soglia, si considera rilevato il livello
M5000 I204 S143.000000      ;Larghezza della vasca (mm); il firmware usa questo parametro insieme all'altezza del livello rilevato per stimare il volume nel contenitore
M5000 I204 P244.000000      ;Altezza della vasca (mm); il firmware usa questo parametro insieme all'altezza del livello rilevato per stimare il volume nel contenitore
M5000 I204 R100.000000      ;Valore di compensazione del volume stimato dal rilevamento resina (ml)
M5000 I205 A3.000000      ;Posizione iniziale dell'autolivellamento (mm); si consiglia maggiore di 1 mm, iniziando la discesa in un punto in cui il film non esercita ancora tensione sulla piattaforma
M5000 I205 B-3.000000      ;Posizione finale dell'autolivellamento (mm); si consiglia -1 mm, valori troppo grandi possono danneggiare lo schermo
M5000 I205 C0.050000      ;Passo dell'autolivellamento (mm); si consiglia 0,05 mm: troppo piccolo allunga il tempo di livellamento, troppo grande rischia di comprimere eccessivamente
M5000 I205 D2.000000      ;Velocità di movimento dell'autolivellamento (mm/s); non è opportuno usare velocità elevate
M5000 I205 E7750      ;Soglia di autolivellamento; in condizioni normali questo valore è maggiore di 0. Durante la regolazione di questo valore l'effetto della resina stessa è già stato considerato
M5000 I206 C10000      ;Soglia di attivazione del rilevamento di corpi estranei
M5000 I206 D1      ;Interruttore del rilevamento resina
M5000 I206 E10      ;Numero di rilevamenti resina
M5000 I206 F-55      ;Soglia minima del rilevamento resina
M5000 I206 S55      ;Soglia massima del rilevamento resina
M5000 I208 A1      ;Interruttore del rilevamento con telecamera AI; 0 spento, 1 acceso
M5000 I208 B100.000000      ;Posizione del rilevamento con telecamera AI
M5000 I209 A1      ;Interruttore del timelapse; 0 spento, 1 acceso
M5000 I209 B50.000000      ;Soglia di altezza del modello oltre la quale non si effettua il timelapse
M5000 I209 C30.000000      ;Altezza del modello da cui inizia il timelapse
M5000 I209 D10      ;Intervallo in layer del timelapse
M5000 I209 E0      ;Con l'homing manuale, Z si sposta alla posizione zero?
M5000 I209 F1      ;Con l'homing manuale, X si sposta alla posizione zero?
M5000 I215 A1000      ;Tempo di attesa del riscaldamento vasca (ms): distacco <(questo parametro)|(riscaldamento)|(questo parametro)>
M5000 I215 B100.000000      ;Posizione di movimento del motore X per il riscaldamento vasca
M5000 I215 C40.000000      ;Velocità di movimento del motore X per il riscaldamento vasca
M5000 I215 D3000      ;Intervallo di tempo del motore X per il riscaldamento vasca (ms)
M5000 I215 E18.000000      ;Posizione di movimento del motore Z per il riscaldamento vasca (in alto)
M5000 I215 Z5.000000      ;Posizione di movimento del motore Z per il riscaldamento vasca (in basso)
M5000 I215 F5.000000      ;Velocità di movimento del motore Z per il riscaldamento vasca
M5000 I215 X3000      ;Intervallo di tempo del motore Z per il riscaldamento vasca (ms)
M5000 I215 Y1      ;Modalità di agitazione durante il riscaldamento vasca; 0-X, 1-Z
M5000 I212 A8420      ;Modalità lenta prima di N layer, tempo di movimento asse X (ms)
M5000 I212 B4410      ;Modalità lenta dopo N layer, tempo di movimento asse X (ms)
M5000 I212 C8420      ;Modalità veloce prima di N layer, tempo di movimento asse X (ms)
M5000 I212 D2430      ;Modalità veloce dopo N layer, tempo di movimento asse X (ms)
M5000 I212 E0      ;Tempo di compensazione della stima di stampa (ms), può essere negativo
M5000 I209 S10000      ;Tempo di scatto rapido del timelapse (tempo reale, unità: ms)
M5000 I209 P70      ;Intervallo di tempo tra gli scatti rapidi del timelapse (ms)
M5999 I0           ;Salva i parametri macchina
