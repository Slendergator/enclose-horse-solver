"""
enclose.horse solver (tkinter + OR-Tools CP-SAT)

Rules, taken from the game's own code:
  * The horse moves up/down/left/right through anything that is not water or a wall.
  * Portals with the same label are linked: reaching one lets the horse reach all of them.
  * If any tile the horse can reach is on the outer edge of the grid, it escapes (score 0).
  * Walls can only go on plain grass: never on water, the horse, cherries, apples, bees or portals.
  * Score = every tile the horse can reach (its own tile and bonus tiles included)
            + 3 per cherry + 10 per golden apple - 5 per bee swarm inside.

Levels can be loaded straight from the site: "Load from enclose.horse" accepts a level link,
a level id or a date (YYYY-MM-DD) and reads the level the page embeds as window.__LEVEL__.

Run:  python enclose_horse_solver.py      (needs:  pip install ortools)
"""

import json
import os
import queue
import re
import threading
import time
import tkinter as tk
import traceback
import urllib.request
from collections import deque
from dataclasses import dataclass, field
from tkinter import filedialog, messagebox, simpledialog, ttk

try:
    from ortools.sat.python import cp_model
except ImportError:  # handled at solve time with a friendly message
    cp_model = None

BONUS = {"cherry": 3, "apple": 10, "bee": -5}
BONUS_LABELS = {"cherry": "cherry", "apple": "apple", "bee": "bees"}
ITEM_CHARS = {"cherry": "C", "apple": "G", "bee": "S"}  # same letters the game uses
ITEM_FROM_CHAR = {ch: kind for kind, ch in ITEM_CHARS.items()}
PORTAL_CHARS = "0123456789abcdefghijklmnopqrstuvwxyz"

FONT = "Segoe UI"
PANEL_BG = "#f6f3ea"
COLORS = {
    "bg": "#1d6b3a",
    "grass": "#3f8f3a",
    "grass_line": "#347a30",
    "water": "#173a5e",
    "water_line": "#0f2a45",
    "hay": "#e2b857",
    "hay_line": "#c49a3c",
    "reach": "#79b85a",
    "reach_line": "#5f9c45",
}

DEFAULT_SIZE = 15
DEFAULT_BUDGET = 10
DEFAULT_TIME_LIMIT = 120
MAX_TIME_LIMIT = 3600
READY_TEXT = "Set up the puzzle, then press Solve."

SITE = "https://enclose.horse"
LEVEL_RE = re.compile(r"window\.__LEVEL__\s*=\s*")
BONUS_ROUND_CHARS = {"U": "unicorn", "K": "gate", "P": "pad"}  # tiles this solver doesn't model


def signed(n):
    """+3 / −5, with a real minus sign."""
    return f"+{n}" if n >= 0 else f"−{-n}"


def portal_name(label):
    return PORTAL_CHARS[label].upper()


TOOLS = [
    ("water", "Water", "1"),
    ("grass", "Grass / erase", "2"),
    ("horse", "Horse", "3"),
    ("cherry", f"Cherry  {signed(BONUS['cherry'])}", "4"),
    ("apple", f"Golden apple  {signed(BONUS['apple'])}", "5"),
    ("bee", f"Bees  {signed(BONUS['bee'])}", "6"),
    ("portal", "Portal", "7"),
    ("wall", "Wall (test)", "8"),
]
TOOL_NAMES = {key: name for key, name, _ in TOOLS}
TOOL_HOTKEYS = {hot: key for key, _, hot in TOOLS}
DRAG_TOOLS = {"water", "grass", "cherry", "apple", "bee", "wall"}


# --------------------------------------------------------------------------- puzzle model
class Puzzle:
    def __init__(self, rows=DEFAULT_SIZE, cols=DEFAULT_SIZE):
        self.rows = rows
        self.cols = cols
        self.water = [False] * (rows * cols)
        self.items = {}    # idx -> "cherry" | "apple" | "bee"
        self.portals = {}  # idx -> label (0..35)
        self.horse = None
        self.walls = set()

    @property
    def size(self):
        return self.rows * self.cols

    def copy(self):
        p = Puzzle(self.rows, self.cols)
        p.water = list(self.water)
        p.items = dict(self.items)
        p.portals = dict(self.portals)
        p.horse = self.horse
        p.walls = set(self.walls)
        return p

    def on_border(self, i):
        r, c = divmod(i, self.cols)
        return r == 0 or c == 0 or r == self.rows - 1 or c == self.cols - 1

    def neighbors(self, i):
        r, c = divmod(i, self.cols)
        if r > 0:
            yield i - self.cols
        if r < self.rows - 1:
            yield i + self.cols
        if c > 0:
            yield i - 1
        if c < self.cols - 1:
            yield i + 1

    def portal_groups(self):
        groups = {}
        for i, lab in self.portals.items():
            groups.setdefault(lab, []).append(i)
        return groups

    def portal_count(self, label):
        return sum(1 for lab in self.portals.values() if lab == label)

    def is_plain_grass(self, i):
        return (not self.water[i] and i != self.horse
                and i not in self.items and i not in self.portals)

    def describe(self, i):
        if i == self.horse:
            return "horse"
        if i in self.items:
            return self.items[i]
        if i in self.portals:
            return f"portal {portal_name(self.portals[i])}"
        if i in self.walls:
            return "wall"
        return "water" if self.water[i] else "grass"

    def state_of(self, i):
        """Everything about tile i, for detecting whether an edit changed it."""
        return (self.water[i], self.items.get(i), self.portals.get(i), i in self.walls, self.horse)

    def clear_cell(self, i):
        self.water[i] = False
        self.items.pop(i, None)
        self.portals.pop(i, None)
        self.walls.discard(i)
        if self.horse == i:
            self.horse = None

    def to_text(self, include_walls=True):
        lines = []
        for r in range(self.rows):
            row = []
            for c in range(self.cols):
                i = r * self.cols + c
                if i == self.horse:
                    ch = "H"
                elif i in self.items:
                    ch = ITEM_CHARS[self.items[i]]
                elif include_walls and i in self.walls:
                    ch = "W"
                elif self.water[i]:
                    ch = "~"
                elif i in self.portals:
                    ch = PORTAL_CHARS[self.portals[i]]
                else:
                    ch = "."
                row.append(ch)
            lines.append("".join(row))
        return "\n".join(lines)

    @classmethod
    def from_text(cls, text, ignore=frozenset()):
        """Parse map text. Returns (puzzle, warnings) where warnings is a list of strings.
        Characters in `ignore` become grass without a warning."""
        lines = [ln.strip() for ln in text.strip().splitlines() if ln.strip()]
        if not lines:
            raise ValueError("No map text found.")
        cols = max(len(ln) for ln in lines)
        p = cls(len(lines), cols)
        unknown = set()
        horses = 0
        for r, line in enumerate(lines):
            for c, ch in enumerate(line):
                i = r * cols + c
                if ch == "H":
                    p.horse = i
                    horses += 1
                elif ch in ITEM_FROM_CHAR:
                    p.items[i] = ITEM_FROM_CHAR[ch]
                elif ch == "W":
                    p.walls.add(i)
                elif ch == "~":
                    p.water[i] = True
                elif ch in PORTAL_CHARS:
                    p.portals[i] = PORTAL_CHARS.index(ch)
                elif ch != "." and ch not in ignore:
                    unknown.add(ch)
        warnings = []
        if unknown:
            warnings.append("These characters were treated as grass: " + " ".join(sorted(unknown)))
        if horses > 1:
            warnings.append(f"The map has {horses} horses; only the last one was kept.")
        return p, warnings


# --------------------------------------------------------------------------- loading from the site
def level_url(text):
    """A pasted link, a level id ("WldoiK") or a date ("2026-10-16") -> the level's page URL."""
    text = text.strip()
    if not text:
        raise ValueError("Enter a level link, id or date.")
    if "://" in text:
        return text
    if text.startswith(("enclose.horse", "www.enclose.horse")):
        return "https://" + text
    return f"{SITE}/play/{text}"


def fetch_level(text, timeout=15):
    """Download a level page and return the level it embeds as window.__LEVEL__
    (a dict with "map" in this solver's text format, "budget", "name", ...)."""
    url = level_url(text)
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        html = resp.read().decode("utf-8", errors="replace")
    m = LEVEL_RE.search(html)
    if not m:
        raise ValueError(f"No level found on {url}")
    level, _ = json.JSONDecoder().raw_decode(html, m.end())
    if not isinstance(level, dict) or not level.get("map"):
        raise ValueError(f"The level on {url} has no map.")
    return level


@dataclass
class Evaluation:
    escaped: bool
    reach: set     # every tile the horse can get to (including edge tiles if it escapes)
    counts: dict   # bonus kind -> how many are inside the reach

    @property
    def area(self):
        return 0 if self.escaped else len(self.reach)

    @property
    def score(self):
        if self.escaped:
            return 0
        return self.area + sum(BONUS[k] * n for k, n in self.counts.items())


def reachable(p, walls=frozenset()):
    """Tiles the horse can walk to (portals included), ignoring whether it escapes."""
    groups = p.portal_groups()
    seen = {p.horse}
    dq = deque([p.horse])
    while dq:
        v = dq.popleft()
        nxt = list(p.neighbors(v))
        if v in p.portals:
            nxt += groups[p.portals[v]]
        for n in nxt:
            if n not in seen and not p.water[n] and n not in walls:
                seen.add(n)
                dq.append(n)
    return seen


def evaluate(p, walls=None):
    """Score a wall layout exactly the way the game does. None if there is no horse."""
    if p.horse is None:
        return None
    reach = reachable(p, p.walls if walls is None else walls)
    counts = {k: sum(1 for i in reach if p.items.get(i) == k) for k in BONUS}
    return Evaluation(escaped=any(p.on_border(i) for i in reach), reach=reach, counts=counts)


# --------------------------------------------------------------------------- solver
@dataclass
class SolveResult:
    status: str                 # "optimal" | "feasible" | "infeasible" | "unknown" | "edge"
    walls: list = field(default_factory=list)
    time: float = 0.0
    bound: int | None = None    # upper bound on the score when not proven optimal


class _SolutionCallback(cp_model.CpSolverSolutionCallback if cp_model else object):
    def __init__(self, wall_vars, on_solution):
        super().__init__()
        self.wall_vars = wall_vars
        self.on_solution = on_solution

    def on_solution_callback(self):
        self.on_solution([v for v, var in self.wall_vars.items() if self.boolean_value(var)])


def _build_model(p, budget):
    """Maximise the enclosed score with at most `budget` walls.

    Model: inside[v] = horse can reach v. Inside is closed under moves (unless blocked by a
    wall), excludes the border, and is connected to the horse (single-commodity flow), so it
    equals the horse's reachable set exactly. Returns (model, wall_vars).
    """
    h = p.horse
    groups = p.portal_groups()
    V = sorted(reachable(p))  # tiles the horse could ever reach

    m = cp_model.CpModel()
    inside = {v: m.new_bool_var(f"in{v}") for v in V}
    wall = {v: m.new_bool_var(f"w{v}") for v in V if p.is_plain_grass(v)}

    m.add(inside[h] == 1)
    for v in V:
        if p.on_border(v):
            m.add(inside[v] == 0)
    for v, w in wall.items():
        m.add_implication(w, ~inside[v])

    arcs = []
    for v in V:
        for u in p.neighbors(v):
            if u not in inside:
                continue  # water
            arcs.append((v, u))
            if u in wall:
                m.add_bool_or([~inside[v], inside[u], wall[u]])
            else:
                m.add_implication(inside[v], inside[u])
        if v in p.portals:
            for u in groups[p.portals[v]]:
                if u != v:
                    arcs.append((v, u))
                    m.add_implication(inside[v], inside[u])

    # A wall is only worth placing next to the enclosure (prunes the search).
    for u, w in wall.items():
        m.add_bool_or([~w] + [inside[n] for n in p.neighbors(u) if n in inside])
    m.add(cp_model.LinearExpr.sum(list(wall.values())) <= budget)

    # Connectivity: horse sends one unit of flow to every inside tile.
    cap = max(1, len(V) - 1)
    in_arcs = {v: [] for v in V}
    out_arcs = {v: [] for v in V}
    for a, b in arcs:
        if b == h:
            continue
        f = m.new_int_var(0, cap, "")
        m.add(f == 0).only_enforce_if(~inside[a])
        m.add(f == 0).only_enforce_if(~inside[b])
        out_arcs[a].append(f)
        in_arcs[b].append(f)
    total_inside = cp_model.LinearExpr.sum([inside[v] for v in V])
    for v in V:
        out_sum = cp_model.LinearExpr.sum(out_arcs[v])
        if v == h:
            m.add(out_sum == total_inside - 1)
        else:
            m.add(cp_model.LinearExpr.sum(in_arcs[v]) - out_sum == inside[v])

    weights = [1 + BONUS.get(p.items.get(v), 0) for v in V]
    score = cp_model.LinearExpr.weighted_sum([inside[v] for v in V], weights)
    # Max score first; among equal scores prefer fewer walls.
    m.maximize(score * (budget + 1) - cp_model.LinearExpr.sum(list(wall.values())))
    return m, wall


def solve_puzzle(p, budget, time_limit=0, on_solution=None, on_solver=None):
    """Find the best wall layout. time_limit <= 0 means no limit.

    on_solution(walls) is called (from a solver thread) for each improving layout;
    on_solver(solver) receives the CpSolver just before the search starts, so it can be stopped.
    """
    if p.on_border(p.horse):
        return SolveResult("edge")
    model, wall = _build_model(p, budget)

    solver = cp_model.CpSolver()
    if time_limit > 0:
        solver.parameters.max_time_in_seconds = float(time_limit)
    solver.parameters.num_workers = max(1, min(16, os.cpu_count() or 8))
    if on_solver:
        on_solver(solver)
    status = solver.solve(model, _SolutionCallback(wall, on_solution) if on_solution else None)

    names = {cp_model.OPTIMAL: "optimal", cp_model.FEASIBLE: "feasible",
             cp_model.INFEASIBLE: "infeasible"}
    result = SolveResult(names.get(status, "unknown"), time=solver.wall_time)
    if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        result.walls = [v for v, var in wall.items() if solver.boolean_value(var)]
        result.bound = int((solver.best_objective_bound + budget) // (budget + 1))
    return result


class SolveJob:
    """Runs solve_puzzle on a background thread. The UI calls drain() to collect messages:
    ("sol", walls), then exactly one of ("done", SolveResult) or ("error", traceback)."""

    def __init__(self, puzzle, budget, time_limit):
        self.started = time.time()
        self.stop_requested = False
        self._messages = queue.Queue()
        self._solver = None
        self._thread = threading.Thread(target=self._run, args=(puzzle, budget, time_limit),
                                        daemon=True)
        self._thread.start()

    def _run(self, puzzle, budget, time_limit):
        try:
            res = solve_puzzle(puzzle, budget, time_limit,
                               on_solution=lambda walls: self._messages.put(("sol", walls)),
                               on_solver=self._set_solver)
            self._messages.put(("done", res))
        except Exception:
            self._messages.put(("error", traceback.format_exc()))

    def _set_solver(self, solver):
        self._solver = solver
        if self.stop_requested:
            solver.parameters.max_time_in_seconds = 0.0  # stopped while the model was building

    def stop(self):
        self.stop_requested = True
        if self._solver is not None:
            self._solver.stop_search()

    def drain(self):
        # stop_search() is ignored until the search has actually started, so keep re-sending it.
        if self.stop_requested:
            self.stop()
        msgs = []
        while True:
            try:
                msgs.append(self._messages.get_nowait())
            except queue.Empty:
                return msgs

    def join(self, timeout):
        self._thread.join(timeout)


# --------------------------------------------------------------------------- drawing
# kind: (fill, outline, text, text colour, font-size tweak)
ROUND_TOKENS = {
    "horse": ("#fafafa", "#6b4226", "H", "#6b4226", 2),
    "cherry": ("#d7263d", "#7a0c1c", signed(BONUS["cherry"]), "white", 0),
    "apple": ("#f4c430", "#8a6a00", signed(BONUS["apple"]), "#4a3600", -1),
    "bee": ("#ffd23f", "#111111", signed(BONUS["bee"]), "#111111", 0),
}


def draw_token(cv, kind, x, y, s, label=""):
    """Draw one tile's content with its top-left at (x, y) and size s."""
    pad = max(1.0, s * 0.12)
    x0, y0, x1, y1 = x + pad, y + pad, x + s - pad, y + s - pad
    fsize = max(6, int(s * 0.30))
    show_text = s >= 16
    if kind in ROUND_TOKENS:
        fill, outline, text, text_fill, tweak = ROUND_TOKENS[kind]
        cv.create_oval(x0, y0, x1, y1, fill=fill, outline=outline, width=2)
        if show_text:
            cv.create_text(x + s / 2, y + s / 2, text=text, fill=text_fill,
                           font=(FONT, max(6, fsize + tweak), "bold"))
    elif kind == "wall":
        cv.create_rectangle(x + 1, y + 1, x + s - 1, y + s - 1, fill="#9b9b9b",
                            outline="#4d4d4d", width=2)
        cv.create_line(x + 3, y + s * 0.55, x + s - 3, y + s * 0.55, fill="#6e6e6e")
        cv.create_rectangle(x + 3, y + 3, x + s - 3, y + s * 0.3, fill="#bdbdbd", width=0)
    elif kind == "portal":
        cv.create_oval(x0, y0, x1, y1, fill="#29a8e0", outline="#0b4f7a", width=2)
        q = s * 0.22
        cv.create_oval(x0 + q, y0 + q, x1 - q, y1 - q, fill="#bfe9ff", outline="")
        if show_text and label:
            cv.create_text(x + s / 2, y + s / 2, text=label.upper(), fill="#0b3a5a",
                           font=(FONT, fsize, "bold"))
    elif kind in ("water", "grass"):
        cv.create_rectangle(x, y, x + s, y + s, fill=COLORS[kind], outline=COLORS[kind + "_line"])


# --------------------------------------------------------------------------- app
class App:
    MARGIN = 30  # room for row/column numbers

    def __init__(self, root):
        self.root = root
        root.title("enclose.horse Solver")
        root.configure(bg=COLORS["bg"])
        self._maximize()

        self.p = Puzzle()
        self.tool = tk.StringVar(value="water")
        self.portal_label = 0
        self.undo_stack, self.redo_stack = [], []
        self.stroke = None
        self.job = None  # SolveJob while the solver runs
        self.best_text = ""

        self.rows_var = tk.StringVar(value=str(DEFAULT_SIZE))
        self.cols_var = tk.StringVar(value=str(DEFAULT_SIZE))
        self.budget_var = tk.StringVar(value=str(DEFAULT_BUDGET))
        self.time_var = tk.StringVar(value=str(DEFAULT_TIME_LIMIT))
        self.show_reach = tk.BooleanVar(value=True)
        self.show_numbers = tk.BooleanVar(value=True)

        self.cell = 20
        self.ox = self.oy = 0

        self._build_ui()
        self._bind_keys()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(50, self.redraw)

    def _maximize(self):
        for attempt in (lambda: self.root.state("zoomed"),               # Windows / macOS
                        lambda: self.root.attributes("-zoomed", True)):  # X11
            try:
                attempt()
                return
            except tk.TclError:
                pass

    @property
    def solving(self):
        return self.job is not None

    # ----------------------------------------------------------------- UI building
    def _build_ui(self):
        style = ttk.Style()
        style.configure("Side.TFrame", background=PANEL_BG)
        style.configure("Side.TLabelframe", background=PANEL_BG)
        style.configure("Side.TLabelframe.Label", background=PANEL_BG, font=(FONT, 10, "bold"))
        for name in ("Side.TLabel", "Side.TRadiobutton", "Side.TCheckbutton"):
            style.configure(name, background=PANEL_BG, font=(FONT, 10))

        side = self._build_sidebar()
        self._build_grid_section(side)
        self._build_tools_section(side)
        self._build_solve_section(side)
        self._build_score_section(side)
        self._build_edit_section(side)
        self._build_file_section(side)
        self._build_board()

    def _section(self, parent, title):
        frame = ttk.LabelFrame(parent, text=title, style="Side.TLabelframe", padding=(6, 2, 6, 4))
        frame.pack(fill="x", pady=3)
        return frame

    def _button_grid(self, parent, buttons, first_row=0):
        for n, (txt, cmd) in enumerate(buttons):
            ttk.Button(parent, text=txt, command=cmd).grid(
                row=first_row + n // 2, column=n % 2, sticky="we", padx=2, pady=2)
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)

    def _build_sidebar(self):
        """Scrollable sidebar (mouse wheel works anywhere over it). Returns the inner frame."""
        outer = tk.Frame(self.root, bg=PANEL_BG)
        outer.pack(side="left", fill="y")
        sc_canvas = tk.Canvas(outer, bg=PANEL_BG, highlightthickness=0)
        sbar = ttk.Scrollbar(outer, orient="vertical", command=sc_canvas.yview)
        sc_canvas.configure(yscrollcommand=sbar.set)
        sc_canvas.pack(side="left", fill="y")
        side = ttk.Frame(sc_canvas, style="Side.TFrame", padding=(10, 6))
        sc_canvas.create_window((0, 0), window=side, anchor="nw")

        def fit_sidebar(_e=None):
            sc_canvas.configure(scrollregion=(0, 0, side.winfo_reqwidth(), side.winfo_reqheight()),
                                width=side.winfo_reqwidth())
            if side.winfo_reqheight() > outer.winfo_height() > 1:
                if not sbar.winfo_ismapped():
                    sbar.pack(side="right", fill="y", before=sc_canvas)
            elif sbar.winfo_ismapped():
                sbar.pack_forget()
                sc_canvas.yview_moveto(0)

        side.bind("<Configure>", fit_sidebar)
        outer.bind("<Configure>", fit_sidebar)

        prefix = str(outer)

        def wheel(e):
            try:
                w = self.root.winfo_containing(e.x_root, e.y_root)
            except (KeyError, tk.TclError):  # e.g. over a popup Tk doesn't track
                return
            path = str(w) if w is not None else ""
            # match the sidebar and its children, but not a sibling like ".!frame2"
            if (path == prefix or path.startswith(prefix + ".")) and sbar.winfo_ismapped():
                sc_canvas.yview_scroll(int(-e.delta / 120) or (-1 if e.delta > 0 else 1), "units")

        self.root.bind_all("<MouseWheel>", wheel)
        return side

    def _build_grid_section(self, side):
        g = self._section(side, "Grid")
        g.pack_configure(pady=(0, 3))
        ttk.Label(g, text="Rows", style="Side.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(g, from_=3, to=60, width=5, textvariable=self.rows_var).grid(row=0, column=1, padx=4)
        ttk.Label(g, text="Cols", style="Side.TLabel").grid(row=0, column=2, sticky="w")
        ttk.Spinbox(g, from_=3, to=60, width=5, textvariable=self.cols_var).grid(row=0, column=3, padx=4)
        ttk.Button(g, text="New grid", command=self.new_grid).grid(row=0, column=4, padx=(6, 0))

    def _build_tools_section(self, side):
        t = self._section(side, "Tiles (keys 1-8)")
        for n, (key, name, hot) in enumerate(TOOLS):
            r, c = divmod(n, 2)
            f = ttk.Frame(t, style="Side.TFrame")
            f.grid(row=r, column=c, sticky="w", padx=2, pady=1)
            sw = tk.Canvas(f, width=22, height=22, highlightthickness=0, bg=PANEL_BG)
            sw.pack(side="left")
            draw_token(sw, "water" if key == "water" else "grass", 1, 1, 20)
            if key not in ("water", "grass"):
                draw_token(sw, key, 1, 1, 20)
            sw.bind("<Button-1>", lambda e, k=key: self.tool.set(k))
            ttk.Radiobutton(f, text=f"{hot}  {name}", value=key, variable=self.tool,
                            style="Side.TRadiobutton").pack(side="left", padx=3)
        pf = ttk.Frame(t, style="Side.TFrame")
        pf.grid(row=4, column=0, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Label(pf, text="Portal pair label:", style="Side.TLabel").pack(side="left")
        ttk.Button(pf, text="◀", width=3, command=lambda: self.shift_portal(-1)).pack(side="left", padx=2)
        self.portal_lbl = ttk.Label(pf, text="0", width=3, anchor="center", style="Side.TLabel",
                                    font=(FONT, 11, "bold"))
        self.portal_lbl.pack(side="left")
        ttk.Button(pf, text="▶", width=3, command=lambda: self.shift_portal(1)).pack(side="left", padx=2)
        ttk.Label(t, text="Left-click / drag: paint     Right-click / drag: erase\n"
                          "Portals auto-advance to a new label after each pair.",
                  style="Side.TLabel", foreground="#555").grid(row=5, column=0, columnspan=2,
                                                               sticky="w", pady=(4, 0))

    def _build_solve_section(self, side):
        s = self._section(side, "Solve")
        ttk.Label(s, text="Walls allowed", style="Side.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(s, from_=0, to=999, width=6, textvariable=self.budget_var,
                    command=self.redraw).grid(row=0, column=1, padx=4, sticky="w")
        ttk.Label(s, text="Time limit (s)", style="Side.TLabel").grid(row=1, column=0, sticky="w")
        ttk.Spinbox(s, from_=0, to=MAX_TIME_LIMIT, increment=10, width=6,
                    textvariable=self.time_var).grid(row=1, column=1, padx=4, sticky="w")
        ttk.Label(s, text="0 = no limit", style="Side.TLabel", foreground="#777").grid(row=1, column=2, sticky="w")
        bf = ttk.Frame(s, style="Side.TFrame")
        bf.grid(row=2, column=0, columnspan=3, sticky="we", pady=(6, 2))
        self.solve_btn = tk.Button(bf, text="SOLVE  (F5)", command=self.solve, bg="#f2c511",
                                   activebackground="#e0b400", font=(FONT, 12, "bold"),
                                   relief="raised", bd=2, padx=10)
        self.solve_btn.pack(side="left", fill="x", expand=True)
        self.stop_btn = tk.Button(bf, text="Stop (Esc)", command=self.stop, state="disabled",
                                  font=(FONT, 10), padx=6)
        self.stop_btn.pack(side="left", padx=(6, 0))
        self.solve_lbl = ttk.Label(s, text=READY_TEXT, style="Side.TLabel",
                                   wraplength=330, justify="left")
        self.solve_lbl.grid(row=3, column=0, columnspan=3, sticky="w", pady=(4, 0))

    def _build_score_section(self, side):
        sc = self._section(side, "Current layout")
        self.score_lbl = tk.Label(sc, text="", font=(FONT, 18, "bold"), bg=PANEL_BG, anchor="w")
        self.score_lbl.pack(fill="x")
        self.detail_lbl = ttk.Label(sc, text="", style="Side.TLabel", wraplength=330, justify="left")
        self.detail_lbl.pack(fill="x")

    def _build_edit_section(self, side):
        e = self._section(side, "Edit")
        self._button_grid(e, [
            ("Undo (Ctrl+Z)", self.undo), ("Redo (Ctrl+Y)", self.redo),
            ("Clear walls", self.clear_walls), ("Clear everything", self.clear_all),
        ])
        ttk.Checkbutton(e, text="Show where the horse can wander", variable=self.show_reach,
                        command=self.redraw, style="Side.TCheckbutton").grid(row=2, column=0, columnspan=2, sticky="w")
        ttk.Checkbutton(e, text="Show row / column numbers", variable=self.show_numbers,
                        command=self.redraw, style="Side.TCheckbutton").grid(row=3, column=0, columnspan=2, sticky="w")

    def _build_file_section(self, side):
        fl = self._section(side, "Save / share")
        ttk.Button(fl, text="Load from enclose.horse...", command=self.load_url).grid(
            row=0, column=0, columnspan=2, sticky="we", padx=2, pady=2)
        self._button_grid(fl, [
            ("Save puzzle...", self.save), ("Load puzzle...", self.load),
            ("Paste map text...", self.import_text), ("Copy map text", self.copy_text),
        ], first_row=1)

    def _build_board(self):
        right = tk.Frame(self.root, bg=COLORS["bg"])
        right.pack(side="left", fill="both", expand=True)
        self.canvas = cv = tk.Canvas(right, bg=COLORS["bg"], highlightthickness=0, cursor="hand2")
        cv.pack(fill="both", expand=True, padx=8, pady=(8, 0))
        self.status = tk.Label(right, text="", bg=COLORS["bg"], fg="white", anchor="w", font=(FONT, 10))
        self.status.pack(fill="x", padx=10, pady=4)

        cv.bind("<Configure>", lambda e: self.redraw())
        cv.bind("<Button-1>", lambda e: self.on_press(e, erase=False))
        cv.bind("<B1-Motion>", self.on_drag)
        cv.bind("<ButtonRelease-1>", self.on_release)
        cv.bind("<Button-3>", lambda e: self.on_press(e, erase=True))
        cv.bind("<B3-Motion>", self.on_drag)
        cv.bind("<ButtonRelease-3>", self.on_release)
        cv.bind("<Motion>", self.on_motion)
        cv.bind("<Leave>", lambda e: self.set_status(""))
        self.tool.trace_add("write", lambda *a: self.set_status(f"Tool: {self.tool_name()}"))

    def _bind_keys(self):
        def unless_typing(action):
            return lambda e: None if self._typing(e) else action()

        r = self.root
        r.bind("<Key>", self.on_key)
        r.bind("<Control-z>", unless_typing(self.undo))
        r.bind("<Control-y>", unless_typing(self.redo))
        r.bind("<Control-Z>", unless_typing(self.redo))
        r.bind("<F5>", lambda e: self.solve())
        r.bind("<Escape>", lambda e: self.stop())

    @staticmethod
    def _typing(e):
        return isinstance(e.widget, (tk.Entry, ttk.Entry, tk.Text, tk.Spinbox))

    def on_key(self, e):
        if not self._typing(e) and e.char in TOOL_HOTKEYS:
            self.tool.set(TOOL_HOTKEYS[e.char])

    def tool_name(self):
        return TOOL_NAMES[self.tool.get()]

    # ----------------------------------------------------------------- helpers
    @staticmethod
    def read_int(var, lo, hi, default):
        try:
            v = int(float(var.get()))
        except (ValueError, tk.TclError):
            v = default
        return max(lo, min(hi, v))

    def get_int(self, var, lo, hi, default):
        """read_int, and also write the cleaned-up value back into the field."""
        v = self.read_int(var, lo, hi, default)
        var.set(str(v))
        return v

    def budget(self, normalize=True):
        read = self.get_int if normalize else self.read_int
        return read(self.budget_var, 0, 999, DEFAULT_BUDGET)

    def set_status(self, text):
        self.status.config(text=text)

    def set_portal_label(self, label):
        self.portal_label = label
        self.portal_lbl.config(text=portal_name(label))

    def snapshot(self):
        return (self.p.copy(), self.portal_label)

    def push_undo(self, snap=None):
        self.undo_stack.append(snap or self.snapshot())
        del self.undo_stack[:-200]
        self.redo_stack.clear()

    def restore(self, snap):
        self.p = snap[0].copy()
        self.set_portal_label(snap[1])
        self.rows_var.set(str(self.p.rows))
        self.cols_var.set(str(self.p.cols))
        self.redraw()

    def busy(self):
        if self.solving:
            self.set_status("The solver is running - press Stop (Esc) before editing.")
        return self.solving

    # ----------------------------------------------------------------- edit actions
    def _reset_puzzle(self, rows, cols):
        self.push_undo()
        self.p = Puzzle(rows, cols)
        self.set_portal_label(0)
        self.solve_lbl.config(text=READY_TEXT)
        self.redraw()

    def new_grid(self):
        if self.busy():
            return
        self._reset_puzzle(self.get_int(self.rows_var, 3, 60, DEFAULT_SIZE),
                           self.get_int(self.cols_var, 3, 60, DEFAULT_SIZE))

    def clear_all(self):
        if not self.busy():
            self._reset_puzzle(self.p.rows, self.p.cols)

    def clear_walls(self):
        if self.busy() or not self.p.walls:
            return
        self.push_undo()
        self.p.walls.clear()
        self.redraw()

    def undo(self):
        if self.busy() or not self.undo_stack:
            return
        self.redo_stack.append(self.snapshot())
        self.restore(self.undo_stack.pop())

    def redo(self):
        if self.busy() or not self.redo_stack:
            return
        self.undo_stack.append(self.snapshot())
        self.restore(self.redo_stack.pop())

    def shift_portal(self, d):
        self.set_portal_label((self.portal_label + d) % len(PORTAL_CHARS))
        self.tool.set("portal")

    def next_free_portal(self):
        used = set(self.p.portals.values())
        return next((lab for lab in range(len(PORTAL_CHARS)) if lab not in used), self.portal_label)

    # ----------------------------------------------------------------- mouse
    def cell_at(self, x, y):
        if x < self.ox or y < self.oy:
            return None
        c = int((x - self.ox) // self.cell)
        r = int((y - self.oy) // self.cell)
        if r < self.p.rows and c < self.p.cols:
            return r * self.p.cols + c
        return None

    def on_press(self, e, erase):
        # Take keyboard focus off any spinbox so the 1-8 / Ctrl+Z shortcuts work after a click.
        self.canvas.focus_set()
        if self.busy():
            return
        i = self.cell_at(e.x, e.y)
        if i is None:
            return
        p = self.p
        tool = "grass" if erase else self.tool.get()
        if tool == "water":
            paint = not p.water[i]
        elif tool in BONUS:
            paint = p.items.get(i) != tool
        elif tool == "wall":
            paint = i not in p.walls
        else:
            paint = True
        self.stroke = {"tool": tool, "paint": paint, "snap": self.snapshot(),
                       "changed": False, "last": i, "start": i}
        self.apply(i)

    def on_drag(self, e):
        st = self.stroke
        if not st or st["tool"] not in DRAG_TOOLS:
            return
        i = self.cell_at(e.x, e.y)
        if i is None or i == st["last"]:
            return
        # walk the straight line from the previous cell so fast drags don't leave gaps
        r0, c0 = divmod(st["last"], self.p.cols)
        r1, c1 = divmod(i, self.p.cols)
        steps = max(abs(r1 - r0), abs(c1 - c0))
        for k in range(1, steps + 1):
            r = round(r0 + (r1 - r0) * k / steps)
            c = round(c0 + (c1 - c0) * k / steps)
            self.apply(r * self.p.cols + c, redraw=False)
        st["last"] = i
        self.redraw()
        self.on_motion(e)

    def on_release(self, e):
        st, self.stroke = self.stroke, None
        if st and st["changed"]:
            self.push_undo(st["snap"])

    def apply(self, i, redraw=True):
        st = self.stroke
        p, tool, paint = self.p, st["tool"], st["paint"]
        before = p.state_of(i)
        if tool == "grass":
            if i != p.horse or i == st["start"]:  # dragging the eraser never deletes the horse
                p.clear_cell(i)
        elif tool == "water":
            if i != p.horse:
                if paint:
                    p.clear_cell(i)
                    p.water[i] = True
                else:
                    p.water[i] = False
        elif tool == "horse":
            if p.horse != i:
                p.clear_cell(i)
                p.horse = i
        elif tool in BONUS:
            if i != p.horse:
                if paint:
                    p.clear_cell(i)
                    p.items[i] = tool
                elif p.items.get(i) == tool:
                    del p.items[i]
        elif tool == "portal":
            if i in p.portals:
                del p.portals[i]
            elif i != p.horse:
                p.clear_cell(i)
                p.portals[i] = self.portal_label
                if p.portal_count(self.portal_label) >= 2:
                    self.set_portal_label(self.next_free_portal())
        elif tool == "wall":
            if not paint:
                p.walls.discard(i)
            elif not p.is_plain_grass(i):
                self.set_status("Walls can only go on plain grass.")
            elif i not in p.walls:
                budget = self.budget()
                if len(p.walls) >= budget:
                    self.set_status(f"Out of walls ({budget} allowed).")
                else:
                    p.walls.add(i)
        if p.state_of(i) != before:
            st["changed"] = True
            if redraw:
                self.redraw()

    def on_motion(self, e):
        i = self.cell_at(e.x, e.y)
        if i is None:
            self.set_status(f"Tool: {self.tool_name()}")
            return
        r, c = divmod(i, self.p.cols)
        self.set_status(f"Row {r + 1}, Col {c + 1}  ({self.p.describe(i)})      Tool: {self.tool_name()}")

    # ----------------------------------------------------------------- drawing
    def redraw(self):
        cv, p = self.canvas, self.p
        cv.delete("all")
        w, h = max(cv.winfo_width(), 100), max(cv.winfo_height(), 100)
        m = self.MARGIN if self.show_numbers.get() else 6
        cs = max(6, min((w - m - 6) // p.cols, (h - m - 6) // p.rows))
        self.cell = cs
        self.ox = (w - cs * p.cols + m) // 2
        self.oy = (h - cs * p.rows + m) // 2

        ev = evaluate(p)
        reach = ev.reach if ev else set()
        enclosed = ev is not None and not ev.escaped
        tint_reach = ev is not None and ev.escaped and self.show_reach.get()

        for i in range(p.size):
            r, c = divmod(i, p.cols)
            x, y = self.ox + c * cs, self.oy + r * cs
            if p.water[i]:
                base = "water"
            elif enclosed and i in reach:
                base = "hay"
            elif tint_reach and i in reach:
                base = "reach"
            else:
                base = "grass"
            cv.create_rectangle(x, y, x + cs, y + cs, fill=COLORS[base], outline=COLORS[base + "_line"])
            if i in p.walls:
                draw_token(cv, "wall", x, y, cs)
            if i == p.horse:
                draw_token(cv, "horse", x, y, cs)
            elif i in p.items:
                draw_token(cv, p.items[i], x, y, cs)
            elif i in p.portals:
                draw_token(cv, "portal", x, y, cs, PORTAL_CHARS[p.portals[i]])

        # outline + escape-path hint
        cv.create_rectangle(self.ox, self.oy, self.ox + cs * p.cols, self.oy + cs * p.rows,
                            outline="#0b2f18", width=2)
        if tint_reach:
            for i in reach:
                if p.on_border(i):
                    r, c = divmod(i, p.cols)
                    x, y = self.ox + c * cs, self.oy + r * cs
                    cv.create_rectangle(x + 2, y + 2, x + cs - 2, y + cs - 2, outline="#ff3b3b", width=2)

        if self.show_numbers.get() and cs >= 10:
            step = 1 if cs >= 18 else 5
            font = (FONT, max(7, min(10, cs // 3)))
            for c in range(step - 1, p.cols, step):
                cv.create_text(self.ox + c * cs + cs / 2, self.oy - 10, text=str(c + 1),
                               fill="white", font=font)
            for r in range(step - 1, p.rows, step):
                cv.create_text(self.ox - 12, self.oy + r * cs + cs / 2, text=str(r + 1),
                               fill="white", font=font)
        self.update_score(ev)

    def update_score(self, ev):
        p = self.p
        walls_txt = f"Walls used: {len(p.walls)} / {self.budget(normalize=False)}"
        lone = [portal_name(lab) for lab, g in sorted(p.portal_groups().items()) if len(g) < 2]
        warn = ""
        if len(lone) == 1:
            warn = f"\nPortal {lone[0]} has no partner yet."
        elif lone:
            warn = f"\nPortals {', '.join(lone)} have no partner yet."
        if ev is None:
            self.score_lbl.config(text="No horse yet", fg="#8a6d00")
            self.detail_lbl.config(text=f"Pick tool 3 and click a tile to place the horse.\n{walls_txt}{warn}")
            return
        if ev.escaped:
            self.score_lbl.config(text="Horse escapes  →  0", fg="#c0392b")
            extra = ("Light green = where it can wander; red boxes = edge tiles it reaches."
                     if self.show_reach.get() else "")
            self.detail_lbl.config(text=f"{extra}\n{walls_txt}{warn}".strip())
            return
        parts = [f"{ev.area} tiles"]
        for kind, n in ev.counts.items():
            if n:
                parts.append(f"{n} {BONUS_LABELS[kind]} ({signed(BONUS[kind] * n)})")
        self.score_lbl.config(text=f"Score: {ev.score}", fg="#1e7d32")
        self.detail_lbl.config(text=" + ".join(parts) + f"\n{walls_txt}{warn}")

    # ----------------------------------------------------------------- solving
    def solve(self):
        if self.solving:
            return
        if cp_model is None:
            messagebox.showerror("OR-Tools missing", "The solver needs OR-Tools.\n\nRun:  pip install ortools")
            return
        p = self.p
        if p.horse is None:
            messagebox.showinfo("No horse", "Place the horse first (tool 3).")
            return
        if p.on_border(p.horse):
            messagebox.showinfo("Horse on the edge",
                                "The horse is on the outer edge, so it can never be enclosed (score 0).")
            return
        budget = self.budget()
        tlim = self.get_int(self.time_var, 0, MAX_TIME_LIMIT, DEFAULT_TIME_LIMIT)
        self.push_undo()  # lets you Undo back to your own walls
        p.walls.clear()
        self.redraw()
        self.best_text = "no enclosure found yet"
        self.job = SolveJob(p.copy(), budget, tlim)
        self.solve_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self.canvas.config(cursor="watch")
        self.solve_lbl.config(text="Building model...")
        self.root.after(100, self._poll)

    def _poll(self):
        if self.job is None:
            return
        got_solution = False
        for kind, payload in self.job.drain():
            if kind == "sol":
                self.p.walls = set(payload)
                got_solution = True
            elif kind == "done":
                self._finish(payload)
                return
            else:  # "error"
                self._end_solving()
                self.solve_lbl.config(text="Solver error - see the popup.")
                messagebox.showerror("Solver error", payload)
                return
        if got_solution:
            self.redraw()
            ev = evaluate(self.p)
            if not ev.escaped:
                self.best_text = f"best so far: score {ev.score}"
        elapsed = time.time() - self.job.started
        verb = "Stopping" if self.job.stop_requested else "Searching"
        self.solve_lbl.config(text=f"{verb}... {elapsed:.0f}s  ({self.best_text})")
        self.root.after(150, self._poll)

    def _end_solving(self):
        self.job = None
        self.solve_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.canvas.config(cursor="hand2")

    def _finish(self, res):
        stopped = self.job.stop_requested
        self._end_solving()
        budget = self.budget()
        self.p.walls = set(res.walls)
        self.redraw()
        ev = evaluate(self.p)
        t = res.time
        if res.status == "edge":
            text = "The horse is on the edge - it can't be enclosed."
        elif res.status == "infeasible":
            text = f"Impossible: the horse can't be enclosed with {budget} walls."
        elif res.status == "unknown":
            text = ("Stopped before any enclosure was found." if stopped else
                    f"No enclosure found in {t:.1f}s. Raise the time limit or wall count.")
        elif res.status == "optimal":
            n = len(self.p.walls)
            text = (f"✔ OPTIMAL: score {ev.score} using {n} wall{'s' * (n != 1)}"
                    f" (proven best, {t:.1f}s). Walls shown in gray.")
        else:
            why = "Stopped" if stopped else "Time limit reached"
            text = (f"{why}: best found score {ev.score} (the best possible is "
                    f"at most {res.bound}). Re-solve with more time to prove it.")
        if (res.status in ("optimal", "feasible") and ev.score < 0
                and evaluate(self.p, walls=set()).escaped):
            text += " That's below 0, so placing no walls (horse escapes, score 0) is better."
        self.solve_lbl.config(text=text)

    def stop(self):
        if self.job is not None:
            self.job.stop()
            self.solve_lbl.config(text="Stopping...")

    # ----------------------------------------------------------------- files
    def save(self):
        path = filedialog.asksaveasfilename(defaultextension=".json", initialfile="horse_puzzle.json",
                                            filetypes=[("Puzzle", "*.json"), ("All files", "*.*")])
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump({"budget": self.budget(), "map": self.p.to_text()}, f, indent=2)
        except OSError as ex:
            messagebox.showerror("Couldn't save", str(ex))
            return
        self.set_status(f"Saved {os.path.basename(path)}")

    def load(self):
        if self.busy():
            return
        path = filedialog.askopenfilename(filetypes=[("Puzzle", "*.json *.txt"), ("All files", "*.*")])
        if not path:
            return
        try:
            with open(path, encoding="utf-8") as f:
                raw = f.read()
            budget = None
            if path.lower().endswith(".json"):
                data = json.loads(raw)
                raw, budget = data["map"], data.get("budget")
            self._load_text(raw)
            if budget is not None:
                self.budget_var.set(str(budget))
                self.redraw()
        except Exception as ex:
            messagebox.showerror("Couldn't load", str(ex))

    def load_url(self):
        if self.busy():
            return
        text = simpledialog.askstring(
            "Load from enclose.horse",
            "Paste a level link, a level id, or a date like 2026-10-16.\n"
            "Leave it as it is for today's daily puzzle.",
            initialvalue=SITE, parent=self.root)
        if not text:
            return
        self.root.config(cursor="watch")
        self.root.update_idletasks()
        try:
            level = fetch_level(text)
        except (OSError, ValueError) as ex:  # network errors are OSErrors, bad pages ValueErrors
            messagebox.showerror("Couldn't load level", str(ex))
            return
        finally:
            self.root.config(cursor="")
        found = sorted({BONUS_ROUND_CHARS[ch] for ch in level["map"] if ch in BONUS_ROUND_CHARS})
        notes = []
        if found:
            notes.append(f"This is a bonus-round level ({', '.join(found)}). Those tiles follow "
                         "rules this solver doesn't know, so they were loaded as grass.")
        self._load_text(level["map"], notes, ignore=set(BONUS_ROUND_CHARS))
        if level.get("budget") is not None:
            self.budget_var.set(str(level["budget"]))
            self.redraw()
        name = level.get("name") or level.get("id") or "level"
        par = level.get("optimalScore")
        self.solve_lbl.config(text=f"Loaded “{name}”."
                                   + (f" The game's best score is {par}." if par is not None else "")
                                   + " Press Solve.")

    def _load_text(self, text, notes=(), ignore=frozenset()):
        """Load map text. notes are extra warnings to show; ignore = characters not to warn about."""
        p, warnings = Puzzle.from_text(text, ignore)
        warnings = list(notes) + warnings
        self.push_undo()
        self.p = p
        self.rows_var.set(str(p.rows))
        self.cols_var.set(str(p.cols))
        self.set_portal_label(self.next_free_portal())
        self.redraw()
        if warnings:
            messagebox.showwarning("Map warnings", "\n".join(warnings))

    def copy_text(self):
        self.root.clipboard_clear()
        self.root.clipboard_append(self.p.to_text())
        self.set_status("Map text copied to the clipboard.")

    def import_text(self):
        if self.busy():
            return
        win = tk.Toplevel(self.root)
        win.title("Paste map text")
        win.transient(self.root)
        ttk.Label(win, text="One line per row.   .  grass    ~  water    H  horse    C  cherry    "
                            "G  golden apple    S  bees    W  wall    0-9 / a-z  portal pairs",
                  padding=8).pack(anchor="w")
        txt = tk.Text(win, width=70, height=24, font=("Consolas", 12))
        txt.pack(fill="both", expand=True, padx=8)
        txt.insert("1.0", self.p.to_text())
        txt.focus_set()

        def ok():
            try:
                self._load_text(txt.get("1.0", "end"))
                win.destroy()
            except Exception as ex:
                messagebox.showerror("Couldn't read map", str(ex), parent=win)

        bf = ttk.Frame(win, padding=8)
        bf.pack(fill="x")
        ttk.Button(bf, text="Load", command=ok).pack(side="right")
        ttk.Button(bf, text="Cancel", command=win.destroy).pack(side="right", padx=6)

    def on_close(self):
        if self.job is not None:
            self.job.stop()
            self.job.join(timeout=2)  # let OR-Tools wind down before the interpreter exits
        self.root.destroy()


def main():
    try:  # crisp text on high-DPI Windows screens
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
