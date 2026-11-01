# Comandi predefiniti della sessione headless (dc-session): etichette e
# documentazione mostrate dalla CLI e dal server MCP.

session-cmd-project-new = Nuovo progetto
session-cmd-project-new-doc = Crea un nuovo progetto vuoto (non salvato). Parametri: patient_code (codice pseudonimizzato, facoltativo). Azzera la cronologia di annullamento e quella dei passaggi.
session-cmd-project-open = Apri progetto
session-cmd-project-open-doc = Apre un file di progetto .dcprj. Parametri: path. I file danneggiati o di una versione più recente si aprono in sola lettura; vengono restituiti i problemi trovati.
session-cmd-project-save = Salva progetto
session-cmd-project-save-doc = Salva il progetto come .dcprj (scrittura atomica, conserva un .bak). Parametri: path (facoltativo; per impostazione predefinita il file aperto, l'estensione .dcprj viene aggiunta se manca).
session-cmd-case-import = Importa scansione
session-cmd-case-import-doc = Importa nel caso scansioni STL, OBJ o PLY, convertite in millimetri, in un unico passaggio annullabile. Parametri: path (un file) oppure files (un elenco di oggetti con path e un role facoltativo), esattamente uno dei due; role (auto, upper, lower o bite). Con auto o senza role decide l'importazione: il nome del file, altrimenti un'arcata libera, segnata come provvisoria perché è una supposizione (un modulo può offrire un proprio comando import-scans che riconosce l'arcata anche dalla geometria). Con files, role è il valore predefinito per le voci senza un role proprio. Una scansione sostituisce quella con lo stesso ruolo, tranne una scansione provvisoria, che si sposta invece sull'arcata libera. Un caso contiene al massimo 2 scansioni del morso. Risultato: scan_id, role, role_source (explicit, file_name, detected o provisional), source_name, source_units, mesh, vertices, faces e replaced descrivono la prima scansione importata; imported elenca tutte le scansioni importate con gli stessi campi più path; warnings (code, key, args, message), failed (path, code, reason, detail), moved, cleanup (facce rimosse) e module_cleared riportano il resto. Un gruppo con alcuni file non importati riesce comunque, con failed compilato; se non viene importato nulla il comando fallisce con i motivi e il caso resta invariato.
session-cmd-case-set-role = Imposta ruolo della scansione
session-cmd-case-set-role-doc = Cambia il ruolo di una scansione, in un unico passaggio annullabile. Parametri: from (upper, lower o bite: il ruolo attuale della scansione), to (il ruolo che riceve), scan_id (necessario solo quando from è bite e il caso ha due scansioni del morso). Il ruolo diventa esplicito. Una scansione già presente sull'arcata di destinazione prende il ruolo from: le due si scambiano. Un caso contiene al massimo 2 scansioni del morso: spostarne una terza sul morso fallisce. Quando cambia un'arcata, il sistema di riferimento del mondo viene azzerato e viene cancellato ciò che i moduli avevano calcolato da quelle scansioni (module_cleared). Risultato: scan_id, source_name, from, to, role_source, swapped (la scansione che ha preso l'altro ruolo, oppure null), world_frame_reset, module_cleared.
session-cmd-edit-undo = Annulla
session-cmd-edit-undo-doc = Annulla l'ultima modifica al caso.
session-cmd-edit-redo = Ripeti
session-cmd-edit-redo-doc = Ripete l'ultima modifica annullata.
session-cmd-steps-list = Elenca passaggi
session-cmd-steps-list-doc = Elenca i record dei passaggi di elaborazione e, quando la pipeline è nota, lo stato di ogni passaggio.
session-cmd-steps-rewind = Torna al passaggio
session-cmd-steps-rewind-doc = Torna allo stato immediatamente precedente a un passaggio di elaborazione: i record di quel passaggio e dei successivi vengono rimossi. Parametri: step.
session-cmd-view-set = Imposta vista
session-cmd-view-set-doc = Memorizza i parametri di vista per il renderer (ad es. preset, width, height, visible). Le chiavi impostate a null vengono rimosse. Non modifica il caso.

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
