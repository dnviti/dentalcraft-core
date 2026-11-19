# Comandi predefiniti della sessione headless (dc-session): etichette e
# documentazione mostrate dalla CLI e dal server MCP.

session-cmd-project-new = Nuovo progetto
session-cmd-project-new-doc = Crea un nuovo progetto vuoto (non salvato). Parametri: patient_code (codice pseudonimizzato, facoltativo). Azzera la cronologia di annullamento e quella dei passaggi.
session-cmd-project-open = Apri progetto
session-cmd-project-open-doc = Apre un file di progetto .dcprj. Parametri: path. I file danneggiati o di una versione più recente si aprono in sola lettura; vengono restituiti i problemi trovati.
session-cmd-project-save = Salva progetto
session-cmd-project-save-doc = Salva il progetto come .dcprj (scrittura atomica, conserva un .bak). Parametri: path (facoltativo; per impostazione predefinita il file aperto, l'estensione .dcprj viene aggiunta se manca).
session-cmd-case-import = Importa scansione
session-cmd-case-import-doc = Importa nel caso scansioni STL, OBJ o PLY, in un unico passaggio annullabile. Le arcate vengono convertite in millimetri; una scansione del morso, riconosciuta dal ruolo o dal nome del file, resta com'è nel file, con un avviso units_suspect se le sue dimensioni fanno pensare a un'altra unità. Parametri: path (un file) oppure files (un elenco di oggetti con path e un role facoltativo), esattamente uno dei due; role (auto, upper, lower o bite). Con auto o senza role decide l'importazione: il nome del file, altrimenti un'arcata libera, segnata come provvisoria perché è una supposizione (un modulo può offrire un proprio comando import-scans che riconosce l'arcata anche dalla geometria). Con files, role è il valore predefinito per le voci senza un role proprio. Una scansione con un ruolo o un'indicazione nel nome sostituisce quella con lo stesso ruolo, tranne una scansione provvisoria, che si sposta invece sull'arcata libera; una scansione senza né l'uno né l'altra non ne sostituisce mai un'altra e, se nessuna arcata è libera, fallisce con needs_role. Un role upper o lower dato a due o più file fallisce con role_clash per tutti. Un caso contiene al massimo 2 scansioni del morso. Finché un modulo contiene dati che questa versione non sa leggere, un file che sostituirebbe o sposterebbe un'arcata fallisce con arches_locked; le arcate libere e le scansioni del morso funzionano comunque. Risultato: scan_id, role, role_source (explicit, file_name, detected, detected_unsure o provisional), source_name, source_units, mesh, vertices, faces e replaced descrivono la prima scansione importata; imported elenca tutte le scansioni importate con gli stessi campi più path; warnings (code, key, args, message), failed (path, code, reason, detail), moved, cleanup (facce rimosse) e module_cleared riportano il resto. Un gruppo con alcuni file non importati riesce comunque, con failed compilato; se non viene importato nulla il comando fallisce con i motivi e il caso resta invariato.
session-cmd-case-set-role = Imposta ruolo della scansione
session-cmd-case-set-role-doc = Cambia il ruolo di una scansione, in un unico passaggio annullabile. Parametri: from (upper, lower o bite: il ruolo attuale della scansione), to (il ruolo che riceve), scan_id (necessario solo quando from è bite e il caso ha due scansioni del morso). Il ruolo diventa esplicito; con from uguale a to non cambia altro, quindi conferma un ruolo che era solo una supposizione, anche detected_unsure (riconosciuto dalla geometria, ma da verificare). Una scansione già presente sull'arcata di destinazione prende il ruolo from: le due si scambiano. Un caso contiene al massimo 2 scansioni del morso: spostarne una terza sul morso fallisce. Finché un modulo contiene dati che questa versione non sa leggere, una modifica che tocca un'arcata fallisce. Quando cambia un'arcata, il sistema di riferimento del mondo viene azzerato e viene cancellato ciò che i moduli avevano calcolato da quelle scansioni (module_cleared). Risultato: scan_id, source_name, from, to, role_source, swapped (la scansione che ha preso l'altro ruolo, oppure null), world_frame_reset, module_cleared.
session-cmd-edit-undo = Annulla
session-cmd-edit-undo-doc = Annulla l'ultima modifica al caso.
session-cmd-edit-redo = Ripeti
session-cmd-edit-redo-doc = Ripete l'ultima modifica annullata.
session-cmd-steps-list = Elenca passaggi
session-cmd-steps-list-doc = Elenca i record dei passaggi di elaborazione e, quando la pipeline è nota, lo stato di ogni passaggio.
session-cmd-steps-rewind = Torna al passaggio
session-cmd-steps-rewind-doc = Torna allo stato immediatamente precedente a un passaggio di elaborazione: i record di quel passaggio e dei successivi vengono rimossi. Parametri: step.
session-cmd-view-set = Imposta vista
session-cmd-view-set-doc = Memorizza i parametri di vista per il renderer (ad es. preset, width, height, visible, model scan|teeth, stage). Le chiavi impostate a null vengono rimosse. Non modifica il caso.

# Comandi dello scanner 3D (scan.*), alpha.
scan-cmd-devices = Elenca scanner
scan-cmd-devices-doc = Elenca gli scanner 3D collegati di ogni backend (backend, id, nome, numero di serie) e lo stato di ciascun backend (available, con un error_code quando non è utilizzabile, ad esempio sdk-missing). Il backend dello scanner simulato sim è sempre disponibile.
scan-cmd-connect = Collega scanner
scan-cmd-connect-doc = Collega uno scanner. Parametri (tutti facoltativi): backend (revopoint-usb: un Revopoint MINI 2 via USB, Linux, senza SDK; revopoint-sdk: tramite l'SDK Revopoint installato dall'utente; sim: lo scanner simulato), device (id preso dall'elenco degli scanner). Senza parametri si usa il primo scanner hardware trovato, altrimenti il typodont superiore simulato. options (solo backend sim) regola il dispositivo simulato: realtime, frames (si ferma dopo questi fotogrammi), width, height, fps, noise_mm, dropout, period_frames, seed.
scan-cmd-disconnect = Scollega scanner
scan-cmd-disconnect-doc = Termina l'acquisizione, scarta la scansione in corso e chiude lo scanner. La scansione già fermata e modificata resta disponibile.
scan-cmd-start = Avvia scansione
scan-cmd-start-doc = Avvia l'acquisizione, oppure riprende la scansione fermata. I fotogrammi vengono tracciati e fusi nella nuvola di punti in tempo reale.
scan-cmd-pause = Metti in pausa la scansione
scan-cmd-pause-doc = Smette di fondere i fotogrammi; il tracciamento continua, così si può riposizionare lo scanner e riprendere la scansione.
scan-cmd-resume = Riprendi scansione
scan-cmd-resume-doc = Riprende la fusione dei fotogrammi dopo una pausa.
scan-cmd-stop = Ferma scansione
scan-cmd-stop-doc = Termina l'acquisizione. La nuvola di punti fusa diventa la scansione da pulire, trasformare in mesh e importare. Restituisce il numero di punti.
scan-cmd-reset = Azzera scansione
scan-cmd-reset-doc = Scarta la scansione acquisita e le sue modifiche; lo scanner resta collegato.
scan-cmd-status = Stato della scansione
scan-cmd-status-doc = Restituisce lo stato dell'acquisizione (state, device, frames, fused_frames, tracking_lost, tracking_ok, points, fps, error, world_angles_deg: gli angoli di rotazione e inclinazione della piattaforma girevole quando è stato fissato il sistema di riferimento; world_from_camera: il sistema di riferimento della scansione rispetto alla telecamera del primo fotogramma fuso, 3x4 per colonne; rig_world: il riferimento è quello calibrato della piattaforma, +Z l'asse di rotazione verso l'alto dal piatto, origine sul piatto, +X verso lo scanner, altrimenti +Z è l'alto vero misurato dall'IMU dello scanner se imu_world, se no l'alto della telecamera; imu_world: l'alto viene dall'IMU dello scanner, orientata da una calibrazione salvata di questo scanner; scanner_deg: l'inclinazione dello scanner, positiva se guarda in basso, e l'imbardata in quel riferimento), auto (avanzamento dell'ultima scansione automatica con piattaforma: state, step, steps, tilt_deg, error, oppure null) e, a scansione ferma, la scansione modificabile (points, mesh_vertices, mesh_faces, can_undo, can_redo).
scan-cmd-settings = Impostazioni di scansione
scan-cmd-settings-doc = Imposta e restituisce le impostazioni di acquisizione. Parametri (tutti facoltativi, applicati quando la scansione non è in corso): live_voxel_mm (da 0,05 a 5), min_depth_mm, max_depth_mm, exposure_us (0 per l'esposizione automatica), roi_radius_mm (da 5 a 200, predefinito 45) e roi_height_mm (da 5 a 300, predefinito 60): il cilindro attorno all'asse della piattaforma a cui vengono ritagliati i fotogrammi quando è in uso una calibrazione della piattaforma, use_roi (predefinito true).
scan-cmd-cloud = Modifica nuvola di punti
scan-cmd-cloud-doc = Applica una modifica alla nuvola di punti della scansione fermata, annullabile con scan.undo. Il parametro op è un oggetto con il campo op: downsample (voxel_mm), remove-statistical (k, std_ratio), remove-radius (radius_mm, min_neighbors), isolate (radius_mm), smooth (k, iterations, strength), delete-aabb (min, max), delete-sphere (center, radius_mm), delete-plane (point, normal), delete-indices (indices), estimate-normals (k). Una modifica alla nuvola elimina la mesh.
scan-cmd-mesh-build = Crea mesh
scan-cmd-mesh-build-doc = Crea la mesh della scansione fermata. Parametro: resolution_mm (da 0,05 a 5; più è piccolo, più è fine e pesante). Una scansione che ha conservato le soste (automatica con soste) e la cui nuvola non è stata modificata viene ricostruita dalle soste, come scan.reconstruct (da 0,05 a 1 mm); altrimenti la mesh viene creata dalla nuvola di punti. Restituisce il numero di vertici e di facce e method (stops o cloud).
scan-cmd-reconstruct = Ricostruisci mesh fine
scan-cmd-reconstruct-doc = Ricostruisce la mesh della scansione fermata dalle soste conservate da una scansione automatica con soste, alla risoluzione dello scanner, come passaggio annullabile: le pose delle soste vengono affinate l'una rispetto all'altra, poi le soste vengono fuse in un volume fine e trasformate in mesh. Parametri (facoltativi): resolution_mm (da 0,05 a 1, predefinito 0,1), refine (predefinito true). Restituisce il numero di vertici e di facce e il resoconto in reconstruction. Fallisce con scan-no-stops per le scansioni a mano e continue.
scan-cmd-mesh = Ripara mesh
scan-cmd-mesh-doc = Applica una riparazione alla mesh, annullabile con scan.undo. Il parametro op è un oggetto con il campo op: fill-holes (max_edges), smooth (iterations, lambda, mu), remove-islands (min_fraction), decimate (ratio), finish (min_island_fraction, predefinito 1: solo il componente principale; max_error_mm, predefinito 0,015): pulisce la mesh, chiude tutti i buchi tranne la base aperta e la ottimizza entro max_error_mm in un solo passaggio, con il resoconto in finish.
scan-cmd-undo = Annulla modifica della scansione
scan-cmd-undo-doc = Annulla l'ultima modifica alla nuvola di punti o alla mesh della scansione. Non tocca il caso.
scan-cmd-redo = Ripristina modifica della scansione
scan-cmd-redo-doc = Riapplica l'ultima modifica annullata alla nuvola di punti o alla mesh della scansione.
scan-cmd-export = Esporta scansione
scan-cmd-export-doc = Scrive la scansione. Parametri: path, what (cloud: nuvola di punti PLY; mesh: STL, PLY od OBJ secondo l'estensione di path, PLY per qualsiasi altra; stops: le soste conservate da una scansione automatica con soste, nella cartella path, stops.json e un file di profondità per sosta, per l'analisi offline).
scan-cmd-import = Importa scansione nel caso
scan-cmd-import-doc = Importa la mesh della scansione nel caso come passaggio annullabile, come un file importato. Parametro: role (upper, lower o bite; facoltativo). Usa il comando di importazione del modulo attivo se ne ha uno, altrimenti case.import. Serve una mesh: va creata prima.
scan-cmd-auto = Scansione automatica con piattaforma
scan-cmd-auto-doc = Scansiona in automatico, in background, l'oggetto sulla piattaforma girevole. Servono uno scanner e una piattaforma collegati e una calibrazione della piattaforma che corrisponda a entrambi (turntable.calibrate): i fotogrammi vengono ritagliati attorno alla piattaforma e gli angoli della piattaforma guidano il tracciamento. Senza calibrazione fallisce con rig-uncalibrated, a meno che allow_uncalibrated sia true (solo in continuo, senza ritaglio: va usato solo quando lo sfondo è oltre il campo di profondità). Con la calibrazione si ferma e acquisisce (mode stop-and-capture): per ogni inclinazione la piattaforma si inclina, poi si ferma ogni step_deg lungo rotation_deg e a ogni sosta fonde la mediana di frames_per_stop fotogrammi; mode continuous ruota senza fermarsi. Parametri (tutti facoltativi): tilts_deg (predefinito [-30, -15, 0, 15, 30]), rotation_deg (predefinito 360), mode, step_deg (predefinito 6), frames_per_stop (predefinito 3), deg_per_s, settle_s (predefinito 0,5), allow_uncalibrated (predefinito false), finish (predefinito true: alla fine l'acquisizione si ferma e la mesh viene ricostruita dalle soste e rifinita, pulita, chiusa e ottimizzata, pronta nell'editor della scansione), resolution_mm (risoluzione di quella mesh, da 0,05 a 1, predefinito 0,1), refine (affina prima le pose delle soste, predefinito true). Risponde subito con started e steps; l'avanzamento si segue con scan.status (auto: phase, reconstruction, finish). Durante la rifinitura l'acquisizione è finita: lo state di scan.status è stopped (nessun fotogramma viene tracciato, il flusso dello scanner e la sua luce sono spenti), busy di turntable.status è false, e auto resta running con phase refine, fuse, extract, clean, fill-holes, poi optimise. scan.pause e scan.resume la mettono in pausa e la riprendono, scan.stop la termina; con finish false al termine la scansione resta in pausa, pronta per scan.stop. Se il collegamento Bluetooth con la piattaforma cade, la scansione la ricollega (fino a 30 s) e prosegue dalla sosta in cui era (auto: reconnecting, reconnects); se non torna la scansione fallisce con ble-disconnected. Una scansione con soste che fallisce o viene annullata conserva le sue soste: la scansione viene fermata, auto kept_stops dice quante sono, e scan.reconstruct e scan.export (stops) funzionano su di esse.

# Comandi della piattaforma girevole (turntable.*), alpha.
turntable-cmd-devices = Elenca piattaforme girevoli
turntable-cmd-devices-doc = Cerca piattaforme girevoli via Bluetooth per timeout_s secondi (da 0 a 30, predefinito 3; con 0 non cerca) e le elenca (backend, id, nome) con lo stato di ciascun backend (available, con un error_code quando non è utilizzabile, ad esempio ble-no-adapter). La piattaforma simulata del backend sim è sempre elencata.
turntable-cmd-connect = Collega piattaforma girevole
turntable-cmd-connect-doc = Collega una piattaforma girevole. Parametri (tutti facoltativi): backend (revopoint-ble: la Revopoint Dual-axis Turntable via Bluetooth; sim: la piattaforma simulata, che lo scanner simulato vede ruotare), device (id preso dall'elenco delle piattaforme). Senza parametri si usa la prima piattaforma trovata via Bluetooth, altrimenti quella simulata. options (solo backend sim): fast (completa subito i movimenti quando nessuno scanner simulato li scandisce). Restituisce la piattaforma, i suoi assi e se è in uso una calibrazione corrispondente.
turntable-cmd-disconnect = Scollega piattaforma girevole
turntable-cmd-disconnect-doc = Ferma la scansione automatica o la calibrazione in corso e chiude la piattaforma. La regione di interesse e la previsione del movimento della calibrazione non vengono più applicate.
turntable-cmd-move = Muovi piattaforma girevole
turntable-cmd-move-doc = Muove un asse della piattaforma. Parametri: axis (rotate o tilt) ed esattamente uno tra by_deg (relativo, con segno) e to_deg (assoluto; l'inclinazione va di solito da -30 a 30). Risponde appena la piattaforma accetta il comando; busy nello stato resta true finché il movimento non finisce. Rifiutato durante una scansione automatica o una calibrazione.
turntable-cmd-speed = Velocità della piattaforma girevole
turntable-cmd-speed-doc = Imposta la velocità di un asse. Parametri: axis (rotate o tilt), deg_per_s (gradi al secondo; la piattaforma la limita al proprio intervallo, quella Revopoint ruota da 4 a 20 gradi al secondo).
turntable-cmd-stop = Ferma piattaforma girevole
turntable-cmd-stop-doc = Ferma subito tutti gli assi, e la scansione automatica o la calibrazione in corso.
turntable-cmd-home = Riporta la piattaforma a zero
turntable-cmd-home-doc = Riporta a zero la rotazione e l'inclinazione. Risponde appena la piattaforma accetta il comando.
turntable-cmd-status = Stato della piattaforma girevole
turntable-cmd-status-doc = Restituisce connected (false quando il collegamento con la piattaforma è caduto, con info ancora presente: un'operazione in corso la ricollega, altrimenti va collegata di nuovo), info, axes (tipo, intervallo, riscontro di posizione), rotate_deg e tilt_deg (stimati dai comandi e dalle letture), busy (un movimento o un'operazione è in corso), calibrated (è in uso una calibrazione corrispondente a questo scanner e a questa piattaforma) e calibration_job (avanzamento dell'ultima calibrazione: state, step, steps, error, reconnecting, reconnects).
turntable-cmd-calibrate = Calibra piattaforma girevole
turntable-cmd-calibrate-doc = Misura gli assi della piattaforma visti dallo scanner fisso, senza strumenti: metti un oggetto fermo sulla piattaforma, collega scanner e piattaforma e non spostarli. Lavora in background: la piattaforma fa prima un giro per riconoscere lo sfondo fisso, poi un altro giro a passi mentre lo scanner traccia l'oggetto, poi si inclina nei due sensi; gli assi vengono ricavati dalle pose tracciate e salvati. Parametri (tutti facoltativi): steps (da 3 a 36, predefinito 8), step_deg (da 5 a 120, predefinito 45; da 180 a 1080 gradi in tutto). Risponde subito; l'avanzamento si segue con turntable.status (calibration_job), il risultato si legge con turntable.calibration. La scansione acquisita durante la calibrazione viene scartata. Se il collegamento Bluetooth con la piattaforma cade, la calibrazione la ricollega (fino a 30 s) e prosegue dalla sosta in cui era.
turntable-cmd-calibration = Calibrazione della piattaforma girevole
turntable-cmd-calibration-doc = Restituisce la calibrazione salvata della piattaforma, oppure null: axis_point, axis_dir, platter_up, tilt_axis_point, tilt_axis_dir (sistema della camera dello scanner, millimetri), scanner_serial, turntable_id, created, fit_rms_deg, fit_rms_mm.

# Usato dai messaggi qui sotto: legge il loro $role (upper, lower, bite).
session-role = { $role ->
    [upper] arcata superiore
    [lower] arcata inferiore
   *[bite] morso
}

session-import-error-path-or-files = Indica esattamente uno tra path (un file) e files (un elenco di file).
session-import-error-no-files = L'elenco dei file è vuoto: indica almeno un file.
session-import-nothing = Nessuna scansione è stata importata. { $reasons }

session-set-role-error-bad-id = { $id } non è un id di scansione valido.
session-set-role-error-no-scan = Il caso non ha nessuna scansione con il ruolo { session-role }.
session-set-role-error-needs-scan-id = Il caso ha più di una scansione con il ruolo { session-role }: indica quale con scan_id.
session-set-role-error-wrong-scan = La scansione { $id } non ha il ruolo { session-role }.
session-set-role-error-bite-full = Un caso contiene al massimo { $max } scansioni del morso: cambia prima il ruolo di una di esse.
