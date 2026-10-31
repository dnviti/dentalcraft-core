# Comandi predefiniti della sessione headless (dc-session): etichette e
# documentazione mostrate dalla CLI e dal server MCP.

session-cmd-project-new = Nuovo progetto
session-cmd-project-new-doc = Crea un nuovo progetto vuoto (non salvato). Parametri: patient_code (codice pseudonimizzato, facoltativo). Azzera la cronologia di annullamento e quella dei passaggi.
session-cmd-project-open = Apri progetto
session-cmd-project-open-doc = Apre un file di progetto .dcprj. Parametri: path. I file danneggiati o di una versione più recente si aprono in sola lettura; vengono restituiti i problemi trovati.
session-cmd-project-save = Salva progetto
session-cmd-project-save-doc = Salva il progetto come .dcprj (scrittura atomica, conserva un .bak). Parametri: path (facoltativo; per impostazione predefinita il file aperto, l'estensione .dcprj viene aggiunta se manca).
session-cmd-case-import = Importa scansione
session-cmd-case-import-doc = Importa nel caso una scansione STL, OBJ o PLY, convertita in millimetri. Parametri: path, role (upper, lower o bite; dedotto dal nome del file se manca). Una scansione sostituisce quella con lo stesso ruolo. Annullabile.
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
