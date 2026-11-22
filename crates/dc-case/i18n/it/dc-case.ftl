# Pipeline di importazione delle scansioni (dc-case): avvisi, errori e
# avanzamento, condivisi da applicazione, CLI e server MCP.

case-import-progress-reading = Lettura delle scansioni

# Usato dai messaggi qui sotto: legge il loro $role (upper, lower, bite).
case-import-role = { $role ->
    [upper] arcata superiore
    [lower] arcata inferiore
   *[bite] morso
}

case-import-warn-rescaled = { $name }: riscalato × { $factor } in millimetri (il file era in { $units }).
case-import-warn-units-suspect = { $name }: le dimensioni fanno pensare a { $units }, ma le scansioni del morso e di controllo non vengono mai riscalate: è stato letto in millimetri. Verifica il file.
case-import-warn-tiny = { $name }: il modello misura solo { $size } mm. Verifica che il file sia in millimetri.
case-import-warn-low-polygons = { $name }: solo { $faces } facce, troppo poche per una scansione clinica. Verifica che sia il file giusto.
case-import-warn-obj-objects-merged = { $name }: i { $objects } oggetti del file OBJ sono stati uniti in un'unica scansione.
case-import-warn-bad-faces-removed = { $name }: facce rimosse: { $degenerate } degeneri, { $duplicate } duplicate.
case-import-warn-replaced = { $name } sostituisce { $old } come { case-import-role }.
case-import-warn-arch-conflict = La scansione { $name } sembra l'altra arcata, ma quel posto è già occupato: è stata assegnata come { case-import-role }. Conflitto di arcata, verifica.
case-import-warn-provisional = { $name }: non è stato possibile riconoscere l'arcata, quindi per ora è assegnata come { case-import-role }. Verifica.
case-import-warn-provisional-moved = { $name } era assegnata in modo provvisorio: ora è { case-import-role }, per fare posto alla nuova scansione.
case-import-warn-bite-limit-replaced = Un caso può avere al massimo { $max } scansioni del morso: { $name } sostituisce { $old }.
case-import-warn-bite-limit-skipped = Un caso può avere al massimo { $max } scansioni del morso: { $name } non è stata importata.

case-import-error-cancelled = Importazione annullata.
case-import-error-unsupported = { $name }: formato di file non supportato. Usa STL, OBJ o PLY.
case-import-error-unreadable = { $name }: impossibile leggere il file.
case-import-error-invalid = { $name }: il file è danneggiato o non è una scansione valida.
case-import-error-empty = { $name }: il file non contiene facce utilizzabili.
case-import-error-needs-role = { $name }: entrambe le arcate sono già occupate. Scegli il ruolo nel pannello Importa scansioni, sotto Ruolo delle prossime scansioni, e importa di nuovo il file (in un comando va nel parametro role).
case-import-error-role-clash = { $name }: più di un file di questa importazione è indicato come { case-import-role }. Indica quale file va assegnato.
case-import-error-arches-locked = Questo progetto contiene dati del modulo { $module } che questa versione non sa leggere, quindi le arcate non si possono sostituire, spostare né rimuovere: aggiorna DentalCraft. Una scansione può comunque andare su un'arcata libera e le scansioni del morso possono cambiare.
