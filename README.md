# enclose.horse Solver

A desktop solver for the [enclose.horse](https://enclose.horse) puzzle game. Draw the puzzle, set how many walls you're allowed, and press **Solve**. It uses Google's [OR-Tools](https://developers.google.com/optimization) CP-SAT solver to find the wall placement with the highest possible score, and proves when that placement is optimal.

## The puzzle

You place a limited number of walls to trap the horse in the biggest pen you can.

- The horse moves up, down, left and right through anything except water and walls.
- Portals with the same label are linked. If the horse can reach one, it can reach all of them.
- If the horse can reach any tile on the outer edge of the grid, it escapes and you score 0.
- Walls can only go on plain grass, not on water, the horse, items or portals.
- **Score** = every tile the horse can reach (including its own tile and item tiles), **+3** per cherry, **+10** per golden apple and **−5** per bee swarm inside the pen.

## Features

- **Puzzle editor** with paint and erase, drag to fill, undo and redo, and grid sizes from 3×3 to 60×60
- **Optimal solver** that maximizes the score and, among equally good answers, uses the fewest walls
- **Live progress** that draws the best layout found so far while the solver runs
- **Time limit and Stop button**. If the solver is stopped early, it shows the best score found and an upper bound on the best possible score.
- **Live scoring** of any layout, including walls you place by hand with the Wall tool
- **Escape view** that shades where the horse can wander and outlines the edge tiles it reaches
- **Save and load** puzzles as JSON, or copy and paste them as plain text

## Requirements

- Python 3.8 or newer, with tkinter (included with the python.org installers for Windows and macOS)
- OR-Tools 9.8 or newer

```bash
pip install ortools
```

## Usage

```bash
python enclose_horse_solver.py
```

1. Set the grid size and click **New grid**, or paste a map (see below).
2. Paint the water, then place the horse, items and portals.
3. Set **Walls allowed** to the number of walls the level gives you.
4. Press **Solve** or `F5`. The walls in the answer are drawn in gray, and the pen is shaded in hay yellow.

Solving clears any walls you placed yourself. Press `Ctrl+Z` to get them back.

### Controls

| Input | Action |
|---|---|
| Left-click / drag | Paint with the selected tile |
| Right-click / drag | Erase to grass |
| `1`–`8` | Pick a tile: water, grass, horse, cherry, golden apple, bees, portal, wall |
| `F5` | Solve |
| `Esc` | Stop the solver and keep the best answer so far |
| `Ctrl+Z` / `Ctrl+Y` | Undo / redo |

Portals are placed in pairs. After you place two portals with the same label, the label moves to the next unused one. Use the ◀ ▶ buttons to pick a label yourself.

### Map text format

**Paste map text** and **Copy map text** use one line per row, with these characters:

| Character | Tile |
|---|---|
| `.` | Grass |
| `~` | Water |
| `H` | Horse |
| `C` | Cherry (+3) |
| `G` | Golden apple (+10) |
| `S` | Bee swarm (−5) |
| `W` | Wall |
| `0`–`9`, `a`–`z` | Portal (matching characters are linked) |

For example, this 7×7 map with 1 wall allowed solves to a score of 13: one wall to the right of the cherry traps the horse in a 10-tile pen that includes the cherry.

```
.......
.~~~~~.
.~...~.
.~.H.C.
.~...~.
.~~~~~.
.......
```

**Save puzzle** writes a JSON file containing the map text and the wall count.

## How it works

The solver models the puzzle as a constraint program with one yes/no variable per tile for whether it's inside the pen and one for whether it has a wall:

- The horse's tile is inside, and edge tiles are not.
- If a tile is inside, every neighbor and linked portal is also inside unless that neighbor has a wall. This means the pen has no openings.
- A flow from the horse's tile to every inside tile keeps the pen connected, so it matches the set of tiles the horse can actually reach.
- The solver maximizes the score first and then uses as few walls as possible.

Only tiles the horse could reach with no walls are included in the model. Walls are only allowed next to the pen, which reduces the search.

## Notes

- This is a fan-made tool and isn't affiliated with enclose.horse.
- Built and tested on Windows. The interface uses the Segoe UI font, so other systems will substitute a different font.
- Large, open maps can take a long time to prove optimal. If the time limit runs out, the solver still shows the best layout it found.
