//! Multi-level module menu.

/// Whether a menu entry or command is implemented.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MenuStatus {
    /// Implemented.
    #[default]
    Ready,
    /// Placeholder: shown, but invoking it says "not available yet".
    Stub,
}

/// What a menu entry does.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MenuKind {
    /// Invokes the command with this id.
    Command(&'static str),
    /// Opens a submenu.
    Submenu(Vec<MenuItem>),
}

/// What a menu leaf needs from the user before it can run. The shell asks for
/// it (a folder or file dialog) and runs the command with the answer as the
/// string parameter `param`; invoking such a leaf runs nothing by itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LeafInput {
    /// A folder.
    Folder {
        /// Parameter of the command that receives the path.
        param: &'static str,
    },
    /// A scan file (STL, OBJ, PLY).
    ScanFile {
        /// Parameter of the command that receives the path.
        param: &'static str,
    },
}

/// One menu entry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MenuItem {
    /// Unique id within the module's menu.
    pub id: &'static str,
    /// i18n key of the label.
    pub label_key: &'static str,
    /// Icon name, if any.
    pub icon: Option<&'static str>,
    /// Command or submenu.
    pub kind: MenuKind,
    /// Implemented or placeholder.
    pub status: MenuStatus,
    /// What the shell asks the user before running a leaf, if anything.
    pub input: Option<LeafInput>,
    /// The leaf is a step of the workflow, counted in the progress. `false`
    /// for what is not one: a shortcut over the steps, an optional tool.
    pub counts_as_step: bool,
}

impl MenuItem {
    /// Entry invoking `command`.
    pub fn command(id: &'static str, label_key: &'static str, command: &'static str) -> Self {
        Self {
            id,
            label_key,
            icon: None,
            kind: MenuKind::Command(command),
            status: MenuStatus::Ready,
            input: None,
            counts_as_step: true,
        }
    }

    /// Entry opening `children`.
    pub fn submenu(id: &'static str, label_key: &'static str, children: Vec<MenuItem>) -> Self {
        Self {
            id,
            label_key,
            icon: None,
            kind: MenuKind::Submenu(children),
            status: MenuStatus::Ready,
            input: None,
            counts_as_step: true,
        }
    }

    /// Same entry with an icon.
    pub fn with_icon(mut self, icon: &'static str) -> Self {
        self.icon = Some(icon);
        self
    }

    /// Same entry marked as a placeholder.
    pub fn stub(mut self) -> Self {
        self.status = MenuStatus::Stub;
        self
    }

    /// Same entry asking the user for `input` before running.
    pub fn with_input(mut self, input: LeafInput) -> Self {
        self.input = Some(input);
        self
    }

    /// Same entry left out of the workflow progress.
    pub fn not_a_step(mut self) -> Self {
        self.counts_as_step = false;
        self
    }

    /// Children of a submenu (empty for a command).
    pub fn children(&self) -> &[MenuItem] {
        match &self.kind {
            MenuKind::Submenu(c) => c,
            MenuKind::Command(_) => &[],
        }
    }

    /// Command id, for a command entry.
    pub fn command_id(&self) -> Option<&'static str> {
        match self.kind {
            MenuKind::Command(c) => Some(c),
            MenuKind::Submenu(_) => None,
        }
    }

    /// Command ids of the workflow steps at or under this entry.
    pub fn step_ids(&self) -> Vec<&'static str> {
        match self.kind {
            MenuKind::Command(c) => {
                if self.counts_as_step {
                    vec![c]
                } else {
                    Vec::new()
                }
            }
            MenuKind::Submenu(_) => self.children().iter().flat_map(Self::step_ids).collect(),
        }
    }
}

/// The top-level entries of a module menu.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct MenuTree {
    /// Top-level entries.
    pub roots: Vec<MenuItem>,
}

impl MenuTree {
    /// Tree with the given top-level entries.
    pub fn new(roots: Vec<MenuItem>) -> Self {
        Self { roots }
    }

    /// Depth-first, pre-order traversal yielding `(depth, item)`; top-level
    /// entries have depth 0.
    pub fn iter(&self) -> DepthFirst<'_> {
        DepthFirst {
            stack: self.roots.iter().rev().map(|i| (0, i)).collect(),
        }
    }

    /// Entry with the given id, at any depth.
    pub fn find(&self, id: &str) -> Option<&MenuItem> {
        self.iter().map(|(_, i)| i).find(|i| i.id == id)
    }

    /// Ids of the entries from a top-level entry down to `id` (included).
    pub fn path_to(&self, id: &str) -> Option<Vec<&'static str>> {
        fn walk(items: &[MenuItem], id: &str, path: &mut Vec<&'static str>) -> bool {
            for it in items {
                path.push(it.id);
                if it.id == id || walk(it.children(), id, path) {
                    return true;
                }
                path.pop();
            }
            false
        }
        let mut path = Vec::new();
        walk(&self.roots, id, &mut path).then_some(path)
    }

    /// Command ids of all command entries, in traversal order.
    pub fn command_ids(&self) -> Vec<&'static str> {
        self.iter().filter_map(|(_, i)| i.command_id()).collect()
    }

    /// Command ids of the workflow steps, in traversal order.
    pub fn step_ids(&self) -> Vec<&'static str> {
        self.roots.iter().flat_map(MenuItem::step_ids).collect()
    }

    /// Ids appearing more than once (should be empty).
    pub fn duplicate_ids(&self) -> Vec<&'static str> {
        let mut seen = std::collections::HashSet::new();
        let mut dup: Vec<&'static str> = self
            .iter()
            .map(|(_, i)| i.id)
            .filter(|id| !seen.insert(*id))
            .collect();
        dup.dedup();
        dup
    }
}

/// Iterator returned by [`MenuTree::iter`].
pub struct DepthFirst<'a> {
    stack: Vec<(usize, &'a MenuItem)>,
}

impl<'a> Iterator for DepthFirst<'a> {
    type Item = (usize, &'a MenuItem);

    fn next(&mut self) -> Option<Self::Item> {
        let (depth, item) = self.stack.pop()?;
        self.stack
            .extend(item.children().iter().rev().map(|c| (depth + 1, c)));
        Some((depth, item))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tree() -> MenuTree {
        MenuTree::new(vec![
            MenuItem::submenu(
                "case",
                "menu-case",
                vec![
                    MenuItem::command("case.import", "menu-import", "case.import")
                        .with_icon("upload"),
                    MenuItem::submenu(
                        "case.frame",
                        "menu-frame",
                        vec![
                            MenuItem::command("case.frame.auto", "menu-frame-auto", "frame.auto")
                                .stub(),
                        ],
                    ),
                ],
            ),
            MenuItem::command("export", "menu-export", "case.export"),
        ])
    }

    #[test]
    fn depth_first_order_and_depths() {
        let t = tree();
        let got: Vec<(usize, &str)> = t.iter().map(|(d, i)| (d, i.id)).collect();
        assert_eq!(
            got,
            vec![
                (0, "case"),
                (1, "case.import"),
                (1, "case.frame"),
                (2, "case.frame.auto"),
                (0, "export")
            ]
        );
    }

    #[test]
    fn find_path_and_commands() {
        let t = tree();
        let f = t.find("case.frame.auto").unwrap();
        assert_eq!(f.status, MenuStatus::Stub);
        assert_eq!(f.command_id(), Some("frame.auto"));
        assert_eq!(t.find("case.import").unwrap().icon, Some("upload"));
        assert!(t.find("missing").is_none());
        assert_eq!(
            t.path_to("case.frame.auto").unwrap(),
            vec!["case", "case.frame", "case.frame.auto"]
        );
        assert!(t.path_to("missing").is_none());
        assert_eq!(
            t.command_ids(),
            vec!["case.import", "frame.auto", "case.export"]
        );
        assert!(t.duplicate_ids().is_empty());
        assert!(t.find("case").unwrap().command_id().is_none());
    }

    #[test]
    fn a_leaf_starts_without_input_and_as_a_step() {
        let item = MenuItem::command("a", "menu-a", "a");
        assert_eq!(item.input, None);
        assert!(item.counts_as_step);
        let input = LeafInput::Folder { param: "dir" };
        assert_eq!(item.with_input(input).input, Some(input));
    }

    #[test]
    fn step_ids_leave_out_what_is_not_a_step() {
        let t = MenuTree::new(vec![
            MenuItem::submenu(
                "g",
                "menu-g",
                vec![
                    MenuItem::command("auto", "menu-auto", "auto").not_a_step(),
                    MenuItem::command("one", "menu-one", "one"),
                    MenuItem::submenu(
                        "tools",
                        "menu-tools",
                        vec![MenuItem::command("two", "menu-two", "two").not_a_step()],
                    ),
                ],
            ),
            MenuItem::command("three", "menu-three", "three"),
        ]);
        assert_eq!(t.step_ids(), vec!["one", "three"]);
        assert_eq!(t.roots[0].step_ids(), vec!["one"]);
        assert_eq!(t.command_ids().len(), 4);
    }

    #[test]
    fn detects_duplicates() {
        let mut t = tree();
        t.roots.push(MenuItem::command("export", "x", "y"));
        assert_eq!(t.duplicate_ids(), vec!["export"]);
    }
}
