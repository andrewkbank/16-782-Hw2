import numpy as np

def create_challenge_map():
    # 50x50 grid
    grid = np.zeros((50, 50), dtype=int)
    
    # 1. Top barrier with a narrow slot on the right
    # Wall from x=0 to 38, at y=17..19 (gap at x=39..49, width 11)
    grid[17:20, 0:38] = 1
    
    # 2. Middle baffle hanging down from top barrier or up from bottom barrier
    # Obstacle in middle chamber to prevent straight diagonal shortcuts
    grid[20:30, 20:25] = 1
    
    # 3. Bottom barrier with a narrow slot on the left
    # Wall from x=12 to 50, at y=32..34 (gap at x=0..11, width 12)
    grid[32:35, 12:50] = 1
    
    # 4. A small post near the bottom slot to make navigating through it even tighter
    grid[38:43, 8:12] = 1

    filename = "../maps/map_challenge.txt"
    h, w = grid.shape
    with open(filename, "w") as f:
        f.write(f"height {h}\nwidth {w}\n")
        for y in range(h):
            row_str = " ".join(str(int(grid[y, x])) for x in range(w)) + " \n"
            f.write(row_str)
            
    print(f"Created {filename} with Double-Slit & Central Baffle maze.")

if __name__ == "__main__":
    create_challenge_map()
