import numpy as np

def save_map(filename, grid):
    h, w = grid.shape
    with open(filename, "w") as f:
        f.write(f"height {h}\nwidth {w}\n")
        for y in range(h):
            row_str = " ".join(str(int(grid[y, x])) for x in range(w)) + " \n"
            f.write(row_str)
    print(f"Saved {filename}")

# Map 3: Staggered Shelves (S-curve corridor)
grid3 = np.zeros((50, 50), dtype=int)
grid3[20:24, 0:22] = 1   # Left shelf
grid3[28:32, 28:50] = 1  # Right shelf
save_map("../maps/map3.txt", grid3)

# Map 4: Clustered Obstacle Pillars
grid4 = np.zeros((50, 50), dtype=int)
pillars = [
    (8, 12, 5, 5),
    (36, 12, 5, 5),
    (14, 26, 5, 5),
    (30, 26, 5, 5),
    (22, 38, 5, 5)
]
for x, y, w, h in pillars:
    grid4[y:y+h, x:x+w] = 1
save_map("../maps/map4.txt", grid4)

# Map 5: Central Floating Box
grid5 = np.zeros((50, 50), dtype=int)
grid5[18:32, 20:30] = 1 # Central box
save_map("../maps/map5.txt", grid5)
