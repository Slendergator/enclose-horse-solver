# Enclose Horse Solver

Optimal solver for [enclose.horse](https://enclose.horse/) puzzles. Design a level in the grid editor, set the wall budget, and get a 100% solution using OR-Tools CP-SAT.

## What is enclose.horse?

A puzzle game where you place a limited number of walls on grass so the horse cannot reach the map border. Your score is the number of enclosed cells (plus bonus for cherries). This tool finds the maximum possible score for any layout you design.

## Requirements

- Python 3.8+
- **ortools** — `pip install ortools`
- **tkinter** — usually included with Python; on Linux you may need `python3-tk`

## Quick start

```bash
pip install ortools
python enclose_solver.py
```

1. Enter **rows** and **columns** (e.g. 15×15) and click **OK**.
2. In the editor, **click cells** to cycle type: Grass → Water → Horse → Cherry → Portal A → Portal B.
3. Place **exactly one Horse (H)** and set the **wall budget**.
4. Click **Solve** to get optimal wall positions and enclosed region (may take up to ~60 seconds).

## Cell types

| Type      | Key   | Description |
|-----------|-------|-------------|
| Grass     | (default) | Can place walls here. |
| Water     | ~     | Horse cannot cross. |
| Horse     | H     | Exactly one per level; must be enclosed. |
| Cherry    | C     | +3 score if enclosed; no walls on cherries. |
| Portal A/B| A, B  | Paired teleports; horse can move between any A and any B. |

## How it works

The solver models the puzzle as a constraint program (OR-Tools CP-SAT):

- **Variables:** which cells are walls, and which cells are reachable by the horse after placing walls.
- **Constraints:** horse is reachable; no reachable cell on the border; wall budget; single-commodity flow so the enclosed region is connected (no "horseless islands").
- **Objective:** maximize enclosed area (grass = 1, cherry = 4).

When the status is **OPTIMAL** or **FEASIBLE**, the grid shows where to place walls (W) and the enclosed region (lighter green).

## Project layout

```
python/
  enclose_solver.py   # Main script (solver + GUI)
  README.md           # This file
```

## License

Use and modify as you like. The game [enclose.horse](https://enclose.horse/) is by Shivers.
