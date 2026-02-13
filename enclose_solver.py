"""
Enclose Horse 100% Solver.
Ask grid size, edit level (grass/water/horse/cherry/portals), then solve with CP-SAT
and display optimal walls and enclosed region.

Libraries to install (fresh Python):
  - ortools   (pip install ortools)
  - tkinter   (usually included with Python; on Linux you may need: python3-tk)

  pip install ortools
"""
from __future__ import annotations

import queue
import threading
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Any

# Cell type constants (for editor state)
GRASS = "."
WATER = "~"
HORSE = "H"
CHERRY = "C"
APPLE = "a"
BEE = "e"
PORTAL_A = "P"
PORTAL_B = "Q"

CELL_CYCLE = [GRASS, WATER, HORSE, CHERRY, APPLE, BEE, PORTAL_A, PORTAL_B]

NEIGHBORS_4 = [(1, 0), (-1, 0), (0, 1), (0, -1)]


def build_neighbors(
    M: int,
    N: int,
    grid_water: list[list[bool]],
    portal_pairs: list[list[tuple[int, int]]],
) -> dict[tuple[int, int], list[tuple[int, int]]]:
    """Precompute per-cell neighbor list: 4-direction non-water + portal links."""
    result: dict[tuple[int, int], list[tuple[int, int]]] = {}

    def in_bounds(i: int, j: int) -> bool:
        return 0 <= i < M and 0 <= j < N

    for i in range(M):
        for j in range(N):
            if grid_water[i][j]:
                result[(i, j)] = []
                continue
            neighbors: list[tuple[int, int]] = []
            for di, dj in NEIGHBORS_4:
                ni, nj = i + di, j + dj
                if in_bounds(ni, nj) and not grid_water[ni][nj]:
                    neighbors.append((ni, nj))
            result[(i, j)] = neighbors

    # Add portal links: each portal pair (list A, list B) connects every A to every B
    for pair in portal_pairs:
        if len(pair) != 2:
            continue
        list_a, list_b = pair[0], pair[1]
        for (ai, aj) in list_a:
            if (ai, aj) not in result:
                continue
            for (bi, bj) in list_b:
                if (bi, bj) not in result[(ai, aj)]:
                    result[(ai, aj)].append((bi, bj))
        for (bi, bj) in list_b:
            if (bi, bj) not in result:
                continue
            for (ai, aj) in list_a:
                if (ai, aj) not in result[(bi, bj)]:
                    result[(bi, bj)].append((ai, aj))

    return result


def solve_enclose(
    grid_water: list[list[bool]],
    horse_pos: tuple[int, int],
    is_cherry: list[list[bool]],
    is_apple: list[list[bool]],
    is_bee: list[list[bool]],
    portal_pairs: list[list[tuple[int, int]]],
    wall_budget: int,
    time_limit_s: float = 60.0,
) -> tuple[str, int, set[tuple[int, int]], set[tuple[int, int]]]:
    """
    CP-SAT optimal solver with reachability and flow constraints.
    Returns (status_name, score, walls_set, reachable_set).
    """
    from ortools.sat.python import cp_model

    M = len(grid_water)
    N = len(grid_water[0]) if grid_water else 0
    hi, hj = horse_pos
    K = M * N

    def in_bounds(i: int, j: int) -> bool:
        return 0 <= i < M and 0 <= j < N

    def is_border(i: int, j: int) -> bool:
        return i == 0 or j == 0 or i == M - 1 or j == N - 1

    neighbors_map = build_neighbors(M, N, grid_water, portal_pairs)

    model = cp_model.CpModel()
    w: dict[tuple[int, int], Any] = {}
    r: dict[tuple[int, int], Any] = {}
    passable: dict[tuple[int, int], Any] = {}

    for i in range(M):
        for j in range(N):
            water = grid_water[i][j]
            cherry = is_cherry[i][j]
            apple = is_apple[i][j]
            bee = is_bee[i][j]
            r[i, j] = model.NewBoolVar(f"r_{i}_{j}")
            if water:
                w[i, j] = model.NewConstant(0)
                passable[i, j] = model.NewConstant(0)
                model.Add(r[i, j] == 0)
                continue
            if (i, j) == (hi, hj) or cherry or apple or bee:
                w[i, j] = model.NewConstant(0)
            else:
                w[i, j] = model.NewBoolVar(f"w_{i}_{j}")
            passable[i, j] = model.NewBoolVar(f"p_{i}_{j}")
            model.Add(passable[i, j] + w[i, j] == 1)
            model.Add(r[i, j] <= passable[i, j])

    model.Add(r[hi, hj] == 1)
    for i in range(M):
        for j in range(N):
            if is_border(i, j):
                model.Add(r[i, j] == 0)

    for (i, j), adj in neighbors_map.items():
        for (ni, nj) in adj:
            if not in_bounds(ni, nj):
                continue
            # r[i,j] and passable[ni,nj] => r[ni,nj]
            model.Add(r[ni, nj] == 1).OnlyEnforceIf(
                [r[i, j], passable[ni, nj]]
            )

    model.Add(
        sum(w[i, j] for i in range(M) for j in range(N)) <= wall_budget
    )

    # Flow: directed edges (i,j) -> (ni,nj)
    flows: dict[tuple[int, int, int, int], Any] = {}
    for i in range(M):
        for j in range(N):
            for (ni, nj) in neighbors_map.get((i, j), []):
                if in_bounds(ni, nj):
                    f = model.NewIntVar(0, K, f"f_{i}_{j}_to_{ni}_{nj}")
                    flows[(i, j, ni, nj)] = f
                    model.Add(f <= K * r[i, j])
                    model.Add(f <= K * r[ni, nj])

    # Reverse map: from (i,j) to list of (pi,pj) that have (i,j) as neighbor
    rev_neighbors: dict[tuple[int, int], list[tuple[int, int]]] = {
        (i, j): [] for i in range(M) for j in range(N)
    }
    for (pi, pj), adj in neighbors_map.items():
        for (ni, nj) in adj:
            if in_bounds(ni, nj):
                rev_neighbors[ni, nj].append((pi, pj))

    total_reached = sum(r[i, j] for i in range(M) for j in range(N))
    for i in range(M):
        for j in range(N):
            outflow = [
                flows[(i, j, ni, nj)]
                for (ni, nj) in neighbors_map.get((i, j), [])
                if in_bounds(ni, nj) and (i, j, ni, nj) in flows
            ]
            inflow = [
                flows[(pi, pj, i, j)]
                for (pi, pj) in rev_neighbors.get((i, j), [])
                if (pi, pj, i, j) in flows
            ]
            if (i, j) == (hi, hj):
                model.Add(sum(outflow) - sum(inflow) == total_reached - 1)
            else:
                model.Add(sum(inflow) - sum(outflow) == r[i, j])

    weights: dict[tuple[int, int], int] = {}
    for i in range(M):
        for j in range(N):
            if grid_water[i][j]:
                weights[i, j] = 0
            elif is_cherry[i][j]:
                weights[i, j] = 3
            elif is_apple[i][j]:
                weights[i, j] = 10
            elif is_bee[i][j]:
                weights[i, j] = -5
            else:
                weights[i, j] = 1
    model.Maximize(
        sum(weights[i, j] * r[i, j] for i in range(M) for j in range(N))
    )

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = float(time_limit_s)
    status = solver.Solve(model)
    status_name = solver.StatusName(status)

    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return status_name, 0, set(), set()

    score = 0
    walls_set: set[tuple[int, int]] = set()
    reachable_set: set[tuple[int, int]] = set()
    for i in range(M):
        for j in range(N):
            if solver.Value(r[i, j]) == 1:
                reachable_set.add((i, j))
                score += weights[i, j]
            if (
                not grid_water[i][j]
                and not is_cherry[i][j]
                and not is_apple[i][j]
                and not is_bee[i][j]
                and (i, j) != (hi, hj)
                and solver.Value(w[i, j]) == 1
            ):
                walls_set.add((i, j))

    return status_name, score, walls_set, reachable_set


def parse_editor_state(
    grid_cells: list[list[str]],
    budget: int,
) -> tuple[
    list[list[bool]],
    tuple[int, int] | None,
    list[list[bool]],
    list[list[bool]],
    list[list[bool]],
    list[list[tuple[int, int]]],
]:
    """
    Convert editor grid and budget to solver inputs.
    Returns (grid_water, horse_pos, is_cherry, is_apple, is_bee, portal_pairs).
    portal_pairs is a list of [portal_a_cells, portal_b_cells] for each pair.
    """
    M = len(grid_cells)
    N = len(grid_cells[0]) if grid_cells else 0
    grid_water = [[False] * N for _ in range(M)]
    is_cherry = [[False] * N for _ in range(M)]
    is_apple = [[False] * N for _ in range(M)]
    is_bee = [[False] * N for _ in range(M)]
    horse_pos: tuple[int, int] | None = None
    portal_a_cells: list[tuple[int, int]] = []
    portal_b_cells: list[tuple[int, int]] = []

    for i in range(M):
        for j in range(N):
            ch = grid_cells[i][j]
            if ch == WATER:
                grid_water[i][j] = True
            elif ch == HORSE:
                horse_pos = (i, j)
            elif ch == CHERRY:
                is_cherry[i][j] = True
            elif ch == APPLE:
                is_apple[i][j] = True
            elif ch == BEE:
                is_bee[i][j] = True
            elif ch == PORTAL_A:
                portal_a_cells.append((i, j))
            elif ch == PORTAL_B:
                portal_b_cells.append((i, j))

    portal_pairs = [portal_a_cells, portal_b_cells]
    if portal_a_cells or portal_b_cells:
        portal_pairs = [[portal_a_cells, portal_b_cells]]
    else:
        portal_pairs = []

    return grid_water, horse_pos, is_cherry, is_apple, is_bee, portal_pairs


# --- GUI ---

CELL_COLORS = {
    GRASS: "#7cb342",
    WATER: "#1976d2",
    HORSE: "#5d4037",
    CHERRY: "#c62828",
    APPLE: "#ff8c00",
    BEE: "#ffeb3b",
    PORTAL_A: "#7b1fa2",
    PORTAL_B: "#00838f",
}

CELL_LABELS = {
    GRASS: "",
    WATER: "~",
    HORSE: "H",
    CHERRY: "C",
    APPLE: "a",
    BEE: "e",
    PORTAL_A: "A",
    PORTAL_B: "B",
}


def ask_grid_size(root: tk.Tk) -> tuple[int, int] | None:
    """Show dialog to enter rows and columns; return (rows, cols) or None."""
    root.title("Enclose Horse — Grid size")
    root.geometry("320x140")
    root.resizable(False, False)
    result: list[tuple[int, int] | None] = [None]
    done = tk.BooleanVar(value=False)

    f = ttk.Frame(root, padding=15)
    f.pack(fill=tk.BOTH, expand=True)
    ttk.Label(f, text="Rows (3–30):").grid(row=0, column=0, sticky="e", padx=4, pady=4)
    row_var = tk.StringVar(value="15")
    row_spin = ttk.Spinbox(f, from_=3, to=30, width=6, textvariable=row_var)
    row_spin.grid(row=0, column=1, padx=4, pady=4)
    ttk.Label(f, text="Columns (3–30):").grid(row=1, column=0, sticky="e", padx=4, pady=4)
    col_var = tk.StringVar(value="15")
    col_spin = ttk.Spinbox(f, from_=3, to=30, width=6, textvariable=col_var)
    col_spin.grid(row=1, column=1, padx=4, pady=4)

    def ok() -> None:
        try:
            r = int(row_var.get())
            c = int(col_var.get())
            if 3 <= r <= 30 and 3 <= c <= 30:
                result[0] = (r, c)
            else:
                messagebox.showerror("Error", "Rows and columns must be between 3 and 30.", parent=root)
                return
        except ValueError:
            messagebox.showerror("Error", "Enter valid numbers.", parent=root)
            return
        done.set(True)

    def cancel() -> None:
        done.set(True)

    btn_f = ttk.Frame(f)
    btn_f.grid(row=2, column=0, columnspan=2, pady=12)
    ttk.Button(btn_f, text="OK", command=ok).pack(side=tk.LEFT, padx=4)
    ttk.Button(btn_f, text="Cancel", command=cancel).pack(side=tk.LEFT, padx=4)
    root.wait_variable(done)
    return result[0]


class EditorWindow:
    def __init__(self, root: tk.Tk, rows: int, cols: int) -> None:
        self.root = root
        self.rows = rows
        self.cols = cols
        self.cell_type: list[list[str]] = [
            [GRASS] * cols for _ in range(rows)
        ]
        self.walls_set: set[tuple[int, int]] = set()
        self.reachable_set: set[tuple[int, int]] = set()
        self.solution_score: int | None = None
        self.solution_status: str | None = None
        self.solving = False
        self.result_queue: queue.Queue = queue.Queue()

        self.win = tk.Toplevel(root)
        self.win.title("Enclose Horse — Level editor")
        self.win.protocol("WM_DELETE_WINDOW", self._on_close)

        top = ttk.Frame(self.win, padding=5)
        top.pack(fill=tk.X)
        ttk.Label(top, text="Wall budget:").pack(side=tk.LEFT, padx=2)
        self.budget_var = tk.StringVar(value="10")
        self.budget_spin = ttk.Spinbox(
            top, from_=0, to=999, width=5, textvariable=self.budget_var
        )
        self.budget_spin.pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Solve", command=self._solve).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Clear solution", command=self._clear_solution).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(top, text="Clear grid", command=self._clear_grid).pack(
            side=tk.LEFT, padx=2
        )
        self.status_label = ttk.Label(top, text="Click cells to set type: Grass → Water → Horse → Cherry → Apple → Bee → Portal A → Portal B")
        self.status_label.pack(side=tk.LEFT, padx=8)

        # Scrollable grid
        canvas = tk.Canvas(self.win)
        scroll_y = ttk.Scrollbar(self.win)
        scroll_x = ttk.Scrollbar(self.win, orient=tk.HORIZONTAL)
        self.cell_size = 28
        self.buttons: list[list[tk.Button]] = [
            [None for _ in range(cols)] for _ in range(rows)
        ]

        frame = ttk.Frame(self.win)
        frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        for i in range(rows):
            for j in range(cols):
                btn = tk.Button(
                    frame,
                    width=2,
                    height=1,
                    text=CELL_LABELS.get(self.cell_type[i][j], ""),
                    bg=CELL_COLORS.get(self.cell_type[i][j], "#7cb342"),
                    relief=tk.RAISED,
                    command=lambda i=i, j=j: self._cycle_cell(i, j),
                )
                btn.grid(row=i, column=j, padx=0, pady=0, sticky="nsew")
                self.buttons[i][j] = btn

        for i in range(rows):
            frame.rowconfigure(i, minsize=self.cell_size)
        for j in range(cols):
            frame.columnconfigure(j, minsize=self.cell_size)

        self.win.after(100, self._poll_result)

    def _cycle_cell(self, i: int, j: int) -> None:
        if self.solving:
            return
        idx = CELL_CYCLE.index(self.cell_type[i][j])
        next_type = CELL_CYCLE[(idx + 1) % len(CELL_CYCLE)]
        if next_type == HORSE:
            for ri in range(self.rows):
                for cj in range(self.cols):
                    if self.cell_type[ri][cj] == HORSE:
                        self.cell_type[ri][cj] = GRASS
                        self._update_button(ri, cj)
                        break
        self.cell_type[i][j] = next_type
        self._update_button(i, j)

    def _update_button(self, i: int, j: int) -> None:
        btn = self.buttons[i][j]
        ct = self.cell_type[i][j]
        btn.config(
            text=CELL_LABELS.get(ct, ""),
            bg=CELL_COLORS.get(ct, "#7cb342"),
        )
        # If we have a solution, overlay wall/reachable
        if (i, j) in self.walls_set:
            btn.config(text="W", bg="#795548")
        elif (i, j) in self.reachable_set and self.reachable_set:
            if ct == GRASS:
                btn.config(bg="#9ccc65")

    def _clear_solution(self) -> None:
        self.walls_set = set()
        self.reachable_set = set()
        self.solution_score = None
        self.solution_status = None
        for i in range(self.rows):
            for j in range(self.cols):
                self._refresh_button_cell(i, j)
        self.status_label.config(text="Solution cleared. Edit and Solve again.")

    def _refresh_button_cell(self, i: int, j: int) -> None:
        ct = self.cell_type[i][j]
        self.buttons[i][j].config(
            text=CELL_LABELS.get(ct, ""),
            bg=CELL_COLORS.get(ct, "#7cb342"),
        )

    def _clear_grid(self) -> None:
        if self.solving:
            return
        for i in range(self.rows):
            for j in range(self.cols):
                self.cell_type[i][j] = GRASS
                self._refresh_button_cell(i, j)
        self.walls_set = set()
        self.reachable_set = set()
        self.solution_score = None
        self.solution_status = None
        self.status_label.config(text="Grid cleared.")

    def _solve(self) -> None:
        if self.solving:
            return
        try:
            budget = int(self.budget_var.get())
        except ValueError:
            messagebox.showerror("Error", "Enter a valid wall budget.", parent=self.win)
            return
        if budget < 0:
            messagebox.showerror("Error", "Wall budget must be >= 0.", parent=self.win)
            return

        grid_water, horse_pos, is_cherry, is_apple, is_bee, portal_pairs = parse_editor_state(
            self.cell_type, budget
        )
        if horse_pos is None:
            messagebox.showerror(
                "Error",
                "Place exactly one Horse (H) by clicking cells until you see H.",
                parent=self.win,
            )
            return

        hi, hj = horse_pos
        if grid_water[hi][hj]:
            messagebox.showerror("Error", "Horse cannot be on water.", parent=self.win)
            return

        self.solving = True
        self.status_label.config(text="Solving… (up to 60 s)")

        def run() -> None:
            status_name, score, walls, reachable = solve_enclose(
                grid_water, horse_pos, is_cherry, is_apple, is_bee, portal_pairs, budget, time_limit_s=60.0
            )
            self.result_queue.put((status_name, score, walls, reachable))

        t = threading.Thread(target=run, daemon=True)
        t.start()

    def _poll_result(self) -> None:
        try:
            status_name, score, walls, reachable = self.result_queue.get_nowait()
        except queue.Empty:
            self.win.after(100, self._poll_result)
            return
        self.solving = False
        self.walls_set = walls
        self.reachable_set = reachable
        self.solution_score = score
        self.solution_status = status_name

        if status_name in ("OPTIMAL", "FEASIBLE"):
            for i in range(self.rows):
                for j in range(self.cols):
                    self._update_button(i, j)
            self.status_label.config(
                text=f"Score: {score}  Walls: {len(walls)}  Status: {status_name}"
            )
        else:
            self.status_label.config(text=f"No solution: {status_name}")
            messagebox.showinfo(
                "Solver",
                f"Solver returned: {status_name}. Try different layout or higher wall budget.",
                parent=self.win,
            )
        self.win.after(100, self._poll_result)

    def _on_close(self) -> None:
        self.win.destroy()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    root.title("Enclose Horse Solver")
    size = ask_grid_size(root)
    if size is None:
        root.destroy()
        return
    rows, cols = size
    # Clear the grid-size form from root so we can reuse it as the app root
    for w in root.winfo_children():
        w.destroy()
    root.withdraw()
    EditorWindow(root, rows, cols)
    root.mainloop()


if __name__ == "__main__":
    main()
