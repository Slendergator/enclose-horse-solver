"""
enclose.horse solver (tkinter + OR-Tools CP-SAT)

Rules, taken from the game's own code:
  * The horse moves up/down/left/right through anything that is not water or a wall.
  * Portals with the same label are linked: reaching one lets the horse reach all of them.
  * If any tile the horse can reach is on the outer edge of the grid, it escapes (score 0).
  * Walls can only go on plain grass: never on water, the horse, cherries, apples, bees or portals.
  * Score = every tile the horse can reach (its own tile and bonus tiles included)
            + 3 per cherry + 10 per golden apple - 5 per bee swarm inside.

Run:  python enclose_horse_solver.py      (needs:  pip install ortools)
"""

import json
import os
import queue
import threading
import time
import tkinter as tk
import traceback
from collections import deque
from tkinter import filedialog, messagebox, ttk

try:
    from ortools.sat.python import cp_model
except ImportError:  # handled at solve time with a friendly message
    cp_model = None

BONUS = {"cherry": 3, "apple": 10, "bee": -5}
ITEM_CHARS = {"cherry": "C", "apple": "G", "bee": "S"}  # same letters the game uses
PORTAL_CHARS = "0123456789abcdefghijklmnopqrstuvwxyz"

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

TOOLS = [
    ("water", "Water", "1"),
    ("grass", "Grass / erase", "2"),
    ("horse", "Horse", "3"),
    ("cherry", "Cherry  +3", "4"),
    ("apple", "Golden apple  +10", "5"),
    ("bee", "Bees  −5", "6"),
    ("portal", "Portal", "7"),
    ("wall", "Wall (test)", "8"),
]
DRAG_TOOLS = {"water", "grass", "cherry", "apple", "bee", "wall"}


# --------------------------------------------------------------------------- puzzle model
class Puzzle:
    def __init__(self, rows=15, cols=15):
        self.rows = rows
        self.cols = cols
        self.water = [False] * (rows * cols)
        self.items = {}    # idx -> "cherry" | "apple" | "bee"
        self.portals = {}  # idx -> label (0..35)
        self.horse = None
        self.walls = set()

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

    def is_plain_grass(self, i):
        return (not self.water[i] and i != self.horse
                and i not in self.items and i not in self.portals)

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
    def from_text(cls, text):
        lines = [ln.strip() for ln in text.strip().splitlines() if ln.strip()]
        if not lines:
            raise ValueError("No map text found.")
        cols = max(len(ln) for ln in lines)
        p = cls(len(lines), cols)
        rev = {v: k for k, v in ITEM_CHARS.items()}
        unknown = set()
        for r, line in enumerate(lines):
            for c, ch in enumerate(line):
                i = r * cols + c
                if ch == "H":
                    p.horse = i
                elif ch in rev:
                    p.items[i] = rev[ch]
                elif ch == "W":
                    p.walls.add(i)
                elif ch == "~":
                    p.water[i] = True
                elif ch in PORTAL_CHARS:
                    p.portals[i] = PORTAL_CHARS.index(ch)
                elif ch != ".":
                    unknown.add(ch)
        return p, unknown


def evaluate(p, walls=None):
    """Score a wall layout exactly the way the game does. None if there is no horse."""
    walls = p.walls if walls is None else walls
    if p.horse is None:
        return None
    groups = p.portal_groups()
    seen = {p.horse}
    dq = deque([p.horse])
    escaped = False
    while dq:
        v = dq.popleft()
        if p.on_border(v):
            escaped = True
        nxt = list(p.neighbors(v))
        if v in p.portals:
            nxt += groups[p.portals[v]]
        for n in nxt:
            if n not in seen and not p.water[n] and n not in walls:
                seen.add(n)
                dq.append(n)
    counts = {k: sum(1 for i in seen if p.items.get(i) == k) for k in BONUS}
    area = 0 if escaped else len(seen)
    score = 0 if escaped else area + sum(BONUS[k] * counts[k] for k in BONUS)
    return {"escaped": escaped, "reach": seen, "area": area, "counts": counts, "score": score}


# --------------------------------------------------------------------------- solver
class _SolutionCallback(cp_model.CpSolverSolutionCallback if cp_model else object):
    def __init__(self, wall_vars, on_solution):
        super().__init__()
        self.wall_vars = wall_vars
        self.on_solution = on_solution

    def on_solution_callback(self):
        walls = [v for v, var in self.wall_vars.items() if self.boolean_value(var)]
        self.on_solution(walls, self.wall_time)


def solve_puzzle(p, budget, time_limit, on_solution, holder):
    """Maximise the enclosed score with at most `budget` walls.

    Model: inside[v] = horse can reach v. Inside is closed under moves (unless blocked by a
    wall), excludes the border, and is connected to the horse (single-commodity flow), so it
    equals the horse's reachable set exactly.
    """
    h = p.horse
    if p.on_border(h):
        return {"status": "edge", "walls": []}

    groups = p.portal_groups()
    cand = evaluate(p, walls=set())["reach"]  # tiles the horse could ever reach
    V = sorted(cand)

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

    solver = cp_model.CpSolver()
    if time_limit > 0:
        solver.parameters.max_time_in_seconds = float(time_limit)
    solver.parameters.num_workers = max(1, min(16, os.cpu_count() or 8))
    holder["solver"] = solver
    status = solver.solve(m, _SolutionCallback(wall, on_solution))

    result = {"walls": [], "time": solver.wall_time}
    if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        result["walls"] = [v for v, var in wall.items() if solver.boolean_value(var)]
        result["bound"] = int((solver.best_objective_bound + budget) // (budget + 1))
    result["status"] = {cp_model.OPTIMAL: "optimal", cp_model.FEASIBLE: "feasible",
                        cp_model.INFEASIBLE: "infeasible"}.get(status, "unknown")
    return result


# --------------------------------------------------------------------------- drawing
def draw_token(cv, kind, x, y, s, label="", tags=()):
    """Draw one tile's content with its top-left at (x, y) and size s."""
    pad = max(1.0, s * 0.12)
    x0, y0, x1, y1 = x + pad, y + pad, x + s - pad, y + s - pad
    fsize = max(6, int(s * 0.30))
    show_text = s >= 16
    if kind == "wall":
        cv.create_rectangle(x + 1, y + 1, x + s - 1, y + s - 1, fill="#9b9b9b",
                            outline="#4d4d4d", width=2, tags=tags)
        cv.create_line(x + 3, y + s * 0.55, x + s - 3, y + s * 0.55, fill="#6e6e6e", tags=tags)
        cv.create_rectangle(x + 3, y + 3, x + s - 3, y + s * 0.3, fill="#bdbdbd", width=0, tags=tags)
    elif kind == "horse":
        cv.create_oval(x0, y0, x1, y1, fill="#fafafa", outline="#6b4226", width=2, tags=tags)
        if show_text:
            cv.create_text(x + s / 2, y + s / 2, text="H", fill="#6b4226",
                           font=("Segoe UI", fsize + 2, "bold"), tags=tags)
    elif kind == "cherry":
        cv.create_oval(x0, y0, x1, y1, fill="#d7263d", outline="#7a0c1c", width=2, tags=tags)
        if show_text:
            cv.create_text(x + s / 2, y + s / 2, text="+3", fill="white",
                           font=("Segoe UI", fsize, "bold"), tags=tags)
    elif kind == "apple":
        cv.create_oval(x0, y0, x1, y1, fill="#f4c430", outline="#8a6a00", width=2, tags=tags)
        if show_text:
            cv.create_text(x + s / 2, y + s / 2, text="+10", fill="#4a3600",
                           font=("Segoe UI", max(6, fsize - 1), "bold"), tags=tags)
    elif kind == "bee":
        cv.create_oval(x0, y0, x1, y1, fill="#ffd23f", outline="#111111", width=2, tags=tags)
        if show_text:
            cv.create_text(x + s / 2, y + s / 2, text="−5", fill="#111111",
                           font=("Segoe UI", fsize, "bold"), tags=tags)
    elif kind == "portal":
        cv.create_oval(x0, y0, x1, y1, fill="#29a8e0", outline="#0b4f7a", width=2, tags=tags)
        q = s * 0.22
        cv.create_oval(x0 + q, y0 + q, x1 - q, y1 - q, fill="#bfe9ff", outline="", tags=tags)
        if show_text and label:
            cv.create_text(x + s / 2, y + s / 2, text=label.upper(), fill="#0b3a5a",
                           font=("Segoe UI", fsize, "bold"), tags=tags)
    elif kind == "water":
        cv.create_rectangle(x, y, x + s, y + s, fill=COLORS["water"], outline=COLORS["water_line"], tags=tags)
    elif kind == "grass":
        cv.create_rectangle(x, y, x + s, y + s, fill=COLORS["grass"], outline=COLORS["grass_line"], tags=tags)


# --------------------------------------------------------------------------- app
class App:
    MARGIN = 30  # room for row/column numbers

    def __init__(self, root):
        self.root = root
        root.title("enclose.horse Solver")
        root.configure(bg=COLORS["bg"])
        try:
            root.state("zoomed")
        except tk.TclError:
            root.attributes("-zoomed", True)

        self.p = Puzzle(15, 15)
        self.tool = tk.StringVar(value="water")
        self.portal_label = 0
        self.undo_stack, self.redo_stack = [], []
        self.stroke = None
        self.solving = False
        self.solver_holder = {}
        self.solve_stopped = False
        self.q = queue.Queue()

        self.rows_var = tk.StringVar(value="15")
        self.cols_var = tk.StringVar(value="15")
        self.budget_var = tk.StringVar(value="10")
        self.time_var = tk.StringVar(value="120")
        self.show_reach = tk.BooleanVar(value=True)
        self.show_numbers = tk.BooleanVar(value=True)

        self.cell = 20
        self.ox = self.oy = 0

        self._build_ui()
        self._bind_keys()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(50, self.redraw)

    # ----------------------------------------------------------------- UI building
    def _build_ui(self):
        style = ttk.Style()
        style.configure("Side.TFrame", background="#f6f3ea")
        style.configure("Side.TLabelframe", background="#f6f3ea")
        style.configure("Side.TLabelframe.Label", background="#f6f3ea", font=("Segoe UI", 10, "bold"))
        style.configure("Side.TLabel", background="#f6f3ea", font=("Segoe UI", 10))
        style.configure("Side.TRadiobutton", background="#f6f3ea", font=("Segoe UI", 10))
        style.configure("Side.TCheckbutton", background="#f6f3ea", font=("Segoe UI", 10))

        # Scrollable sidebar (mouse wheel works anywhere over it)
        outer = tk.Frame(self.root, bg="#f6f3ea")
        outer.pack(side="left", fill="y")
        self.side_outer = outer
        sc_canvas = tk.Canvas(outer, bg="#f6f3ea", highlightthickness=0)
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

        def wheel(e):
            w = self.root.winfo_containing(e.x_root, e.y_root)
            if w is not None and str(w).startswith(str(outer)) and sbar.winfo_ismapped():
                sc_canvas.yview_scroll(int(-e.delta / 120) or (-1 if e.delta > 0 else 1), "units")

        self.root.bind_all("<MouseWheel>", wheel)

        # Grid size
        g = ttk.LabelFrame(side, text="Grid", style="Side.TLabelframe", padding=(6, 2, 6, 4))
        g.pack(fill="x", pady=(0, 3))
        ttk.Label(g, text="Rows", style="Side.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(g, from_=3, to=60, width=5, textvariable=self.rows_var).grid(row=0, column=1, padx=4)
        ttk.Label(g, text="Cols", style="Side.TLabel").grid(row=0, column=2, sticky="w")
        ttk.Spinbox(g, from_=3, to=60, width=5, textvariable=self.cols_var).grid(row=0, column=3, padx=4)
        ttk.Button(g, text="New grid", command=self.new_grid).grid(row=0, column=4, padx=(6, 0))

        # Tools
        t = ttk.LabelFrame(side, text="Tiles (keys 1-8)", style="Side.TLabelframe", padding=(6, 2, 6, 4))
        t.pack(fill="x", pady=3)
        for n, (key, name, hot) in enumerate(TOOLS):
            r, c = divmod(n, 2)
            f = ttk.Frame(t, style="Side.TFrame")
            f.grid(row=r, column=c, sticky="w", padx=2, pady=1)
            sw = tk.Canvas(f, width=22, height=22, highlightthickness=0, bg="#f6f3ea")
            sw.pack(side="left")
            base = "water" if key == "water" else "grass"
            draw_token(sw, base, 1, 1, 20)
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
                                    font=("Segoe UI", 11, "bold"))
        self.portal_lbl.pack(side="left")
        ttk.Button(pf, text="▶", width=3, command=lambda: self.shift_portal(1)).pack(side="left", padx=2)
        ttk.Label(t, text="Left-click / drag: paint     Right-click / drag: erase\n"
                          "Portals auto-advance to a new label after each pair.",
                  style="Side.TLabel", foreground="#555").grid(row=5, column=0, columnspan=2, sticky="w", pady=(4, 0))

        # Solve
        s = ttk.LabelFrame(side, text="Solve", style="Side.TLabelframe", padding=(6, 2, 6, 4))
        s.pack(fill="x", pady=3)
        ttk.Label(s, text="Walls allowed", style="Side.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(s, from_=0, to=999, width=6, textvariable=self.budget_var,
                    command=self.redraw).grid(row=0, column=1, padx=4, sticky="w")
        ttk.Label(s, text="Time limit (s)", style="Side.TLabel").grid(row=1, column=0, sticky="w")
        ttk.Spinbox(s, from_=0, to=3600, increment=10, width=6,
                    textvariable=self.time_var).grid(row=1, column=1, padx=4, sticky="w")
        ttk.Label(s, text="0 = no limit", style="Side.TLabel", foreground="#777").grid(row=1, column=2, sticky="w")
        bf = ttk.Frame(s, style="Side.TFrame")
        bf.grid(row=2, column=0, columnspan=3, sticky="we", pady=(6, 2))
        self.solve_btn = tk.Button(bf, text="SOLVE  (F5)", command=self.solve, bg="#f2c511",
                                   activebackground="#e0b400", font=("Segoe UI", 12, "bold"),
                                   relief="raised", bd=2, padx=10)
        self.solve_btn.pack(side="left", fill="x", expand=True)
        self.stop_btn = tk.Button(bf, text="Stop (Esc)", command=self.stop, state="disabled",
                                  font=("Segoe UI", 10), padx=6)
        self.stop_btn.pack(side="left", padx=(6, 0))
        self.solve_lbl = ttk.Label(s, text="Set up the puzzle, then press Solve.", style="Side.TLabel",
                                   wraplength=330, justify="left")
        self.solve_lbl.grid(row=3, column=0, columnspan=3, sticky="w", pady=(4, 0))

        # Score
        sc = ttk.LabelFrame(side, text="Current layout", style="Side.TLabelframe", padding=(6, 2, 6, 4))
        sc.pack(fill="x", pady=3)
        self.score_lbl = tk.Label(sc, text="", font=("Segoe UI", 18, "bold"), bg="#f6f3ea", anchor="w")
        self.score_lbl.pack(fill="x")
        self.detail_lbl = ttk.Label(sc, text="", style="Side.TLabel", wraplength=330, justify="left")
        self.detail_lbl.pack(fill="x")

        # Edit
        e = ttk.LabelFrame(side, text="Edit", style="Side.TLabelframe", padding=(6, 2, 6, 4))
        e.pack(fill="x", pady=3)
        buttons = [
            ("Undo (Ctrl+Z)", self.undo), ("Redo (Ctrl+Y)", self.redo),
            ("Clear walls", self.clear_walls), ("Clear everything", self.clear_all),
        ]
        for n, (txt, cmd) in enumerate(buttons):
            ttk.Button(e, text=txt, command=cmd).grid(row=n // 2, column=n % 2, sticky="we", padx=2, pady=2)
        ttk.Checkbutton(e, text="Show where the horse can wander", variable=self.show_reach,
                        command=self.redraw, style="Side.TCheckbutton").grid(row=2, column=0, columnspan=2, sticky="w")
        ttk.Checkbutton(e, text="Show row / column numbers", variable=self.show_numbers,
                        command=self.redraw, style="Side.TCheckbutton").grid(row=3, column=0, columnspan=2, sticky="w")
        e.columnconfigure(0, weight=1)
        e.columnconfigure(1, weight=1)

        # File
        fl = ttk.LabelFrame(side, text="Save / share", style="Side.TLabelframe", padding=(6, 2, 6, 4))
        fl.pack(fill="x", pady=3)
        buttons = [
            ("Save puzzle...", self.save), ("Load puzzle...", self.load),
            ("Paste map text...", self.import_text), ("Copy map text", self.copy_text),
        ]
        for n, (txt, cmd) in enumerate(buttons):
            ttk.Button(fl, text=txt, command=cmd).grid(row=n // 2, column=n % 2, sticky="we", padx=2, pady=2)
        fl.columnconfigure(0, weight=1)
        fl.columnconfigure(1, weight=1)

        # Board
        right = tk.Frame(self.root, bg=COLORS["bg"])
        right.pack(side="left", fill="both", expand=True)
        self.canvas = tk.Canvas(right, bg=COLORS["bg"], highlightthickness=0, cursor="hand2")
        self.canvas.pack(fill="both", expand=True, padx=8, pady=(8, 0))
        self.status = tk.Label(right, text="", bg=COLORS["bg"], fg="white", anchor="w",
                               font=("Segoe UI", 10))
        self.status.pack(fill="x", padx=10, pady=4)

        cv = self.canvas
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
        r = self.root
        r.bind("<Key>", self.on_key)
        r.bind("<Control-z>", lambda e: self.undo())
        r.bind("<Control-y>", lambda e: self.redo())
        r.bind("<Control-Z>", lambda e: self.redo())
        r.bind("<F5>", lambda e: self.solve())
        r.bind("<Escape>", lambda e: self.stop())

    def on_key(self, e):
        if isinstance(e.widget, (tk.Entry, ttk.Entry, tk.Text, ttk.Spinbox, tk.Spinbox)):
            return
        for key, _, hot in TOOLS:
            if e.char == hot:
                self.tool.set(key)
                return

    def tool_name(self):
        return next(name for key, name, _ in TOOLS if key == self.tool.get())

    # ----------------------------------------------------------------- helpers
    def get_int(self, var, lo, hi, default):
        try:
            v = int(float(var.get()))
        except (ValueError, tk.TclError):
            v = default
        v = max(lo, min(hi, v))
        var.set(str(v))
        return v

    def budget(self):
        return self.get_int(self.budget_var, 0, 999, 10)

    def set_status(self, text):
        self.status.config(text=text)

    def snapshot(self):
        return (self.p.copy(), self.portal_label)

    def push_undo(self, snap=None):
        self.undo_stack.append(snap or self.snapshot())
        del self.undo_stack[:-200]
        self.redo_stack.clear()

    def restore(self, snap):
        self.p = snap[0].copy()
        self.portal_label = snap[1]
        self.rows_var.set(str(self.p.rows))
        self.cols_var.set(str(self.p.cols))
        self.portal_lbl.config(text=PORTAL_CHARS[self.portal_label].upper())
        self.redraw()

    def busy(self):
        if self.solving:
            self.set_status("The solver is running - press Stop (Esc) before editing.")
        return self.solving

    # ----------------------------------------------------------------- edit actions
    def new_grid(self):
        if self.busy():
            return
        rows = self.get_int(self.rows_var, 3, 60, 15)
        cols = self.get_int(self.cols_var, 3, 60, 15)
        self.push_undo()
        self.p = Puzzle(rows, cols)
        self.portal_label = 0
        self.portal_lbl.config(text="0")
        self.solve_lbl.config(text="Set up the puzzle, then press Solve.")
        self.redraw()

    def clear_all(self):
        if self.busy():
            return
        self.push_undo()
        self.p = Puzzle(self.p.rows, self.p.cols)
        self.portal_label = 0
        self.portal_lbl.config(text="0")
        self.solve_lbl.config(text="Set up the puzzle, then press Solve.")
        self.redraw()

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
        self.portal_label = (self.portal_label + d) % len(PORTAL_CHARS)
        self.portal_lbl.config(text=PORTAL_CHARS[self.portal_label].upper())
        self.tool.set("portal")

    def next_free_portal(self):
        used = set(self.p.portals.values())
        for lab in range(len(PORTAL_CHARS)):
            if lab not in used:
                return lab
        return self.portal_label

    # ----------------------------------------------------------------- mouse
    def cell_at(self, x, y):
        c = int((x - self.ox) // self.cell)
        r = int((y - self.oy) // self.cell)
        if 0 <= r < self.p.rows and 0 <= c < self.p.cols and x >= self.ox and y >= self.oy:
            return r * self.p.cols + c
        return None

    def on_press(self, e, erase):
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
        self.stroke = {"tool": tool, "paint": paint, "snap": self.snapshot(), "changed": False, "last": i, "start": i}
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
        st = self.stroke
        self.stroke = None
        if st and st["changed"]:
            self.push_undo(st["snap"])

    def apply(self, i, redraw=True):
        st = self.stroke
        p, tool, paint = self.p, st["tool"], st["paint"]
        before = (p.water[i], p.items.get(i), p.portals.get(i), i in p.walls, p.horse)
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
                if list(p.portals.values()).count(self.portal_label) >= 2:
                    self.portal_label = self.next_free_portal()
                    self.portal_lbl.config(text=PORTAL_CHARS[self.portal_label].upper())
        elif tool == "wall":
            if paint:
                if p.is_plain_grass(i) and i not in p.walls:
                    if len(p.walls) >= self.budget():
                        self.set_status(f"Out of walls ({self.budget()} allowed).")
                    else:
                        p.walls.add(i)
                elif not p.is_plain_grass(i):
                    self.set_status("Walls can only go on plain grass.")
            else:
                p.walls.discard(i)
        after = (p.water[i], p.items.get(i), p.portals.get(i), i in p.walls, p.horse)
        if after != before:
            st["changed"] = True
            if redraw:
                self.redraw()

    def on_motion(self, e):
        i = self.cell_at(e.x, e.y)
        if i is None:
            self.set_status(f"Tool: {self.tool_name()}")
            return
        r, c = divmod(i, self.p.cols)
        p = self.p
        what = ("horse" if i == p.horse else p.items.get(i)
                or (f"portal {PORTAL_CHARS[p.portals[i]].upper()}" if i in p.portals else None)
                or ("wall" if i in p.walls else None) or ("water" if p.water[i] else "grass"))
        self.set_status(f"Row {r + 1}, Col {c + 1}  ({what})      Tool: {self.tool_name()}")

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
        reach = ev["reach"] if ev else set()
        enclosed = ev is not None and not ev["escaped"]
        tint_reach = ev is not None and ev["escaped"] and self.show_reach.get()

        for i in range(p.rows * p.cols):
            r, c = divmod(i, p.cols)
            x, y = self.ox + c * cs, self.oy + r * cs
            if p.water[i]:
                fill, line = COLORS["water"], COLORS["water_line"]
            elif enclosed and i in reach:
                fill, line = COLORS["hay"], COLORS["hay_line"]
            elif tint_reach and i in reach:
                fill, line = COLORS["reach"], COLORS["reach_line"]
            else:
                fill, line = COLORS["grass"], COLORS["grass_line"]
            cv.create_rectangle(x, y, x + cs, y + cs, fill=fill, outline=line)
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
            font = ("Segoe UI", max(7, min(10, cs // 3)))
            for c in range(p.cols):
                if (c + 1) % step == 0 or step == 1:
                    cv.create_text(self.ox + c * cs + cs / 2, self.oy - 10, text=str(c + 1),
                                   fill="white", font=font)
            for r in range(p.rows):
                if (r + 1) % step == 0 or step == 1:
                    cv.create_text(self.ox - 12, self.oy + r * cs + cs / 2, text=str(r + 1),
                                   fill="white", font=font)
        self.update_score(ev)

    def update_score(self, ev):
        p = self.p
        walls_txt = f"Walls used: {len(p.walls)} / {self.budget()}"
        lone = [PORTAL_CHARS[lab].upper() for lab, g in p.portal_groups().items() if len(g) < 2]
        warn = f"\nPortal {', '.join(lone)} has no partner yet." if lone else ""
        if ev is None:
            self.score_lbl.config(text="No horse yet", fg="#8a6d00")
            self.detail_lbl.config(text=f"Pick tool 3 and click a tile to place the horse.\n{walls_txt}{warn}")
            return
        if ev["escaped"]:
            self.score_lbl.config(text="Horse escapes  →  0", fg="#c0392b")
            extra = ("Light green = where it can wander; red boxes = edge tiles it reaches."
                     if self.show_reach.get() else "")
            self.detail_lbl.config(text=f"{extra}\n{walls_txt}{warn}".strip())
            return
        cnt = ev["counts"]
        parts = [f"{ev['area']} tiles"]
        if cnt["cherry"]:
            parts.append(f"{cnt['cherry']} cherry (+{3 * cnt['cherry']})")
        if cnt["apple"]:
            parts.append(f"{cnt['apple']} apple (+{10 * cnt['apple']})")
        if cnt["bee"]:
            parts.append(f"{cnt['bee']} bees (−{5 * cnt['bee']})")
        self.score_lbl.config(text=f"Score: {ev['score']}", fg="#1e7d32")
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
        tlim = self.get_int(self.time_var, 0, 36000, 60)
        self.push_undo()  # lets you Undo back to your own walls
        work = p.copy()
        work.walls = set()
        p.walls.clear()
        self.redraw()
        self.solving = True
        self.solve_stopped = False
        self.solver_holder = {}
        self.solve_start = time.time()
        self.solve_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self.canvas.config(cursor="watch")
        self.solve_lbl.config(text="Building model...")
        threading.Thread(target=self._solve_worker, args=(work, budget, tlim), daemon=True).start()
        self.root.after(100, self._poll)

    def _solve_worker(self, work, budget, tlim):
        try:
            res = solve_puzzle(work, budget, tlim,
                               lambda walls, t: self.q.put(("sol", walls, t)), self.solver_holder)
            self.q.put(("done", res))
        except Exception:
            self.q.put(("error", traceback.format_exc()))

    def _poll(self):
        best = None
        try:
            while True:
                msg = self.q.get_nowait()
                if msg[0] == "sol":
                    self.p.walls = set(msg[1])
                    best = msg
                elif msg[0] == "done":
                    self._finish(msg[1])
                    return
                elif msg[0] == "error":
                    self._end_solving()
                    self.solve_lbl.config(text="Solver error - see the popup.")
                    messagebox.showerror("Solver error", msg[1])
                    return
        except queue.Empty:
            pass
        elapsed = time.time() - self.solve_start
        if best is not None:
            self.redraw()
        ev = evaluate(self.p) if self.p.walls else None
        so_far = f"best so far: score {ev['score']}" if ev and not ev["escaped"] else "no enclosure found yet"
        self.solve_lbl.config(text=f"Searching... {elapsed:.0f}s  ({so_far})")
        self.root.after(150, self._poll)

    def _end_solving(self):
        self.solving = False
        self.solve_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.canvas.config(cursor="hand2")

    def _finish(self, res):
        self._end_solving()
        budget = self.budget()
        st = res["status"]
        self.p.walls = set(res.get("walls", []))
        self.redraw()
        ev = evaluate(self.p)
        t = res.get("time", time.time() - self.solve_start)
        if st == "edge":
            self.solve_lbl.config(text="The horse is on the edge - it can't be enclosed.")
        elif st == "infeasible":
            self.solve_lbl.config(text=f"Impossible: the horse can't be enclosed with {budget} walls.")
        elif st == "unknown":
            self.solve_lbl.config(text=f"No enclosure found in {t:.1f}s. Raise the time limit or wall count.")
        elif st == "optimal":
            n = len(self.p.walls)
            self.solve_lbl.config(text=f"✔ OPTIMAL: score {ev['score']} using {n} wall{'s' * (n != 1)}"
                                       f" (proven best, {t:.1f}s). Walls shown in gray.")
        else:
            why = "Stopped" if self.solve_stopped else "Time limit reached"
            self.solve_lbl.config(text=f"{why}: best found score {ev['score']} (the best possible is "
                                       f"at most {res.get('bound', '?')}). Re-solve with more time to prove it.")

    def stop(self):
        if self.solving and "solver" in self.solver_holder:
            self.solve_stopped = True
            self.solver_holder["solver"].stop_search()
            self.solve_lbl.config(text="Stopping...")

    # ----------------------------------------------------------------- files
    def save(self):
        path = filedialog.asksaveasfilename(defaultextension=".json", initialfile="horse_puzzle.json",
                                            filetypes=[("Puzzle", "*.json"), ("All files", "*.*")])
        if not path:
            return
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"budget": self.budget(), "map": self.p.to_text()}, f, indent=2)
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

    def _load_text(self, text):
        p, unknown = Puzzle.from_text(text)
        self.push_undo()
        self.p = p
        self.rows_var.set(str(p.rows))
        self.cols_var.set(str(p.cols))
        self.portal_label = self.next_free_portal()
        self.portal_lbl.config(text=PORTAL_CHARS[self.portal_label].upper())
        self.redraw()
        if unknown:
            messagebox.showwarning("Unknown tiles", "These characters were treated as grass: "
                                   + " ".join(sorted(unknown)))

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
        if self.solving and "solver" in self.solver_holder:
            self.solver_holder["solver"].stop_search()
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
