import tkinter as tk
from tkinter import ttk, Toplevel, messagebox, filedialog
import heapq
import math
import random
import time
import statistics
import json
import os



# --- Terrain Setup ---
TERRAIN_TYPES = ['flat', 'sandy', 'rock', 'recharge', 'hazardous', 'trap', 'cliff']
TERRAIN_INDEX = {t: i for i, t in enumerate(TERRAIN_TYPES)}
TERRAIN_COSTS = {
    'flat': 5.0,
    'sandy': 10.0,
    'rock': 1e3,
    'recharge': 5.0,
    'hazardous': 15.0,
    'trap': 20.0,
    'cliff': 1e4
}
TERRAIN_COLORS = {
    'flat': '#32CD32',      # green
    'sandy': '#FFD700',     # yellow
    'rock': '#A52A2A',      # brown
    'recharge': '#00BFFF',  # blue
    'hazardous': '#FF4500', # orange-red
    'trap': '#800080',      # purple
    'cliff': '#696969'      # dark gray
}
BATTERY_FULL = 100.0


# --- Heuristics (unchanged) ---
def manhattan(a, b, grid=None):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def euclidean(a, b, grid=None):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def magnetic_field(a, b, grid):
    """
    Calculates a heuristic based on an artificial potential field.
    - The goal exerts an attractive force.
    - High-cost terrain exerts a repulsive force.
    """
    k_att = 1.0
    k_rep = 200.0
    max_influence_dist = 5
    repulsive_terrains = {'rock': 10, 'sandy': 2, 'cliff': 20}
    dist_to_goal_sq = (a[0] - b[0])**2 + (a[1] - b[1])**2
    attractive_potential = 0.5 * k_att * dist_to_goal_sq
    repulsive_potential = 0.0
    min_x = max(0, a[0] - max_influence_dist)
    max_x = min(len(grid[0]) - 1, a[0] + max_influence_dist)
    min_y = max(0, a[1] - max_influence_dist)
    max_y = min(len(grid) - 1, a[1] + max_influence_dist)
    for y in range(min_y, max_y + 1):
        for x in range(min_x, max_x + 1):
            terrain_type = grid[y][x]
            if terrain_type in repulsive_terrains:
                dist_to_obstacle = math.hypot(a[0] - x, a[1] - y)
                if 0 < dist_to_obstacle <= max_influence_dist:
                    terrain_strength = repulsive_terrains[terrain_type]
                    repulsion = terrain_strength * 0.5 * k_rep * ((1 / dist_to_obstacle) - (1 / max_influence_dist))**2
                    repulsive_potential += repulsion
    return attractive_potential + repulsive_potential

def path_of_least_regret(a, b, grid):
    """
    A risk-averse heuristic that penalizes paths entering volatile, high-variance terrain.
    """
    RISK_AVERSION_FACTOR = 10.0
    PROBE_RADIUS = 3
    base_heuristic = euclidean(a, b)
    nearby_costs = []
    min_x = max(0, a[0] - PROBE_RADIUS)
    max_x = min(len(grid[0]) - 1, a[0] + PROBE_RADIUS)
    min_y = max(0, a[1] - PROBE_RADIUS)
    max_y = min(len(grid) - 1, a[1] + PROBE_RADIUS)
    for y in range(min_y, max_y + 1):
        for x in range(min_x, max_x + 1):
            terrain = grid[y][x]
            nearby_costs.append(TERRAIN_COSTS.get(terrain, 5.0))
    variance = statistics.variance(nearby_costs) if len(nearby_costs) > 1 else 0.0
    risk_penalty = RISK_AVERSION_FACTOR * math.log(1 + variance)
    return base_heuristic + risk_penalty

HEUR_FN_MAP = {
    "Manhattan": manhattan,
    "Euclidean": euclidean,
    "Terrain Penalized": magnetic_field,
    "Path of Least Regret": path_of_least_regret
}

HEUR_COLORS = {
    "Manhattan": "cyan",
    "Euclidean": "yellow",
    "Terrain Penalized": "magenta",
    "Path of Least Regret": "purple"
}

# --- Grid Generation ---
def generate_grid(rows, cols):

    probs = {
        'sandy': 0.1,
        'rock': 0.08,
        'recharge': 0.03,
        'hazardous': 0.05,
        'trap': 0.03,
        'cliff': 0.02
    }
    grid = []
    for r in range(rows):
        row = []
        for c in range(cols):
            p = random.random()
            cell = 'flat'
            cum = 0
            for t, pv in probs.items():
                cum += pv
                if p < cum:
                    cell = t
                    break
            row.append(cell)
        grid.append(row)
    # Ensure at least one recharge cell
    if not any(cell == 'recharge' for row in grid for cell in row):
        grid[random.randrange(rows)][random.randrange(cols)] = 'recharge'
    return grid


# --- Helpers ---
def in_bounds(node, grid):
    r, c = node
    return 0 <= r < len(grid) and 0 <= c < len(grid[0])

def neighbors_4(node):
    r, c = node
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        yield (r + dr, c + dc), 1.0

# --- A* Algorithm ---
def astar(grid, start, goal, heur_name, avoid_cells=None):
    if avoid_cells is None:
        avoid_cells = set()
    h_fn = HEUR_FN_MAP.get(heur_name, euclidean)
    if start == goal:
        return [start], 0, 0.0
    open_heap = []
    heapq.heappush(open_heap, (h_fn(start, goal, grid), 0.0, start))
    came_from = {}
    gscore = {start: 0.0}
    closed = set()
    explored = 0
    while open_heap:
        f, g, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        explored += 1
        if current == goal:
            path = [current]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            return path[::-1], explored, gscore[current]
        for nb, mult in neighbors_4(current):
            if not in_bounds(nb, grid):
                continue
            # avoid only hard obstacles
            if (grid[nb[0]][nb[1]] in ('rock', 'cliff') or nb in avoid_cells):
                    continue
            # allow trap and hazardous so rover can step on them

            tentative = gscore[current] + TERRAIN_COSTS[grid[nb[0]][nb[1]]] * mult
            if tentative < gscore.get(nb, float('inf')):
                gscore[nb] = tentative
                came_from[nb] = current
                heapq.heappush(open_heap, (tentative + h_fn(nb, goal, grid), tentative, nb))
    return None, explored, float('inf')

# --- Battery-Aware Planner (from Rover GUI v3) ---
def plan_with_recharges(grid, start, goal, battery, h_fn, recharge_sites, avoid_cells=None):
    if avoid_cells is None:
        avoid_cells = set()
    direct_path, _, direct_cost = astar(grid, start, goal, h_fn)
    if direct_path and direct_cost <= battery:
        return direct_path, direct_cost, "direct"
    visited = set()
    current = start
    remaining = battery
    full_path = []
    total_cost = 0.0
    max_iter = 8
    for _ in range(max_iter):
        p_goal, _, c_goal = astar(grid, current, goal, h_fn)
        if p_goal and c_goal <= remaining:
            if full_path and full_path[-1] == p_goal[0]:
                full_path.extend(p_goal[1:])
            else:
                full_path.extend(p_goal)
            total_cost += c_goal
            return full_path, total_cost, "chain_to_goal"
        reachable = []
        for rc in recharge_sites:
            if rc in visited:
                continue
            p_rc, _, c_rc = astar(grid, current, rc, h_fn)
            if p_rc and c_rc <= remaining:
                reachable.append((rc, p_rc, c_rc))
        if not reachable:
            return None, total_cost, "no_recharge_reachable"
        reachable.sort(key=lambda x: h_fn(x[0], goal, grid))
        rc, p_rc, c_rc = reachable[0]
        if full_path and full_path[-1] == p_rc[0]:
            full_path.extend(p_rc[1:])
        else:
            full_path.extend(p_rc)
        total_cost += c_rc
        current = rc
        remaining = BATTERY_FULL
        visited.add(rc)
        if current == goal:
            return full_path, total_cost, "recharge_equal_goal"
    return None, total_cost, "chain_limit_reached"

def find_nearest_recharge_within(grid, start, recharge_sites, max_euclid_dist):
    candidates = []
    for rc in recharge_sites:
        if math.hypot(start[0]-rc[0], start[1]-rc[1]) <= max_euclid_dist:
            p, _, c = astar(grid, start, rc, "Euclidean")  # Use Euclidean for recharge search
            if p:
                candidates.append((math.hypot(start[0]-rc[0], start[1]-rc[1]), rc, p, c))
    if not candidates:
        return None, None, None
    candidates.sort(key=lambda x: (x[0], x[3]))
    _, rc, p, c = candidates[0]
    return rc, p, c

# --- Rover Simulator GUI ---
class RoverSim:
    def __init__(self, root):
        self.root = root
        root.title("A* Rover Reflex Agent")
        self.replan_attempts = 0          #  how many times we tried to replan after hazard
        self.max_replan_attempts = 3
        self.rows, self.cols, self.cell_size = 20, 20, 25
        self.grid = generate_grid(self.rows, self.cols)
        self.start = (0, 0)
        self.goal = (self.rows - 1, self.cols - 1)
        self.results = []
        self.rover = None
        self.trails = []
        self.battery = BATTERY_FULL
        self.total_cost_used = 0.0
        self.running = False
        self.pos = None
        self.avoid_cells = set()
        self.planned_path = None
        self.heading_to_recharge = False  # Track if rover is heading to a recharge station
        self.simulate_change = False
        self.change_triggered = False
        self.status = tk.Label(root, text="Ready", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.status.pack(side='bottom', fill='x')
        self.terrain_change_scheduled = False
        

        menubar = tk.Menu(root)
        filemenu = tk.Menu(menubar, tearoff=0)
        filemenu.add_command(label="Exit", command=root.quit)
        menubar.add_cascade(label="File", menu=filemenu)
        gridmenu = tk.Menu(menubar, tearoff=0)
        gridmenu.add_command(label="Randomize", command=self.randomize_grid)
        menubar.add_cascade(label="Grid", menu=gridmenu)
        helpmenu = tk.Menu(menubar, tearoff=0)
        helpmenu.add_command(label="About", command=lambda: messagebox.showinfo("About", "A*-Rover Reflex Agent\n4 Heuristics with Battery"))
        menubar.add_cascade(label="Help", menu=helpmenu)
        root.config(menu=menubar)

        self.canvas = tk.Canvas(root, width=self.cols * self.cell_size, height=self.rows * self.cell_size, bg='white')
        self.canvas.pack(side='left', fill='both', expand=True)
        self.canvas.bind("<Button-1>", self.set_position)
        self.canvas.bind("<Button-3>", self.change_terrain)
        

        self.sidebar = tk.Frame(root, bg='#e0e0e0', width=250)
        self.sidebar.pack(side='right', fill='y')

        tk.Label(self.sidebar, text="Select Heuristic:", bg='#e0e0e0').pack(pady=5)
        self.heur_box = ttk.Combobox(self.sidebar, values=list(HEUR_FN_MAP.keys()), state='readonly')
        self.heur_box.current(0)
        self.heur_box.pack(pady=5)
        self.heur_box.bind('<<ComboboxSelected>>', self.on_heuristic_change)

        tk.Button(self.sidebar, text="Plan Path", command=self.plan_path, bg='#4CAF50', fg='white').pack(pady=5)
        tk.Button(self.sidebar, text="Run Path", command=self.run_path, bg='#2196F3', fg='white').pack(pady=5)

        tk.Button(self.sidebar, text="View Report", command=self.show_report, bg='#9C27B0', fg='white').pack(pady=5)
        tk.Button(self.sidebar, text="Compare All", command=self.compare_all_heuristics, bg='#607D8B', fg='white').pack(pady=5)
        tk.Button(self.sidebar, text="Clear Paths", command=self.clear_trails, bg="#8ADA6B", fg='black').pack(pady=5) 
        tk.Button(self.sidebar, text="Clear Results", command=self.clear_results, bg='#FF5722', fg='white').pack(pady=5)
        tk.Button(self.sidebar, text="Randomize Terrain",command=self.enable_simulate_change,bg='#FFA500', fg='black').pack(pady=5)
        tk.Button(self.sidebar, text="Randomize Grid", command=self.randomize_grid, bg='#2196F3', fg='white').pack(pady=5)
        tk.Button(self.sidebar, text="Save Grid", command=self.save_grid,bg='#8BC34A', fg='white').pack(pady=5)
        tk.Button(self.sidebar, text="Load Grid", command=self.load_grid,bg='#00BCD4', fg='white').pack(pady=5)

        tk.Label(self.sidebar, text="Animation Delay (s):", bg='#e0e0e0').pack(pady=5)
        self.delay_var = tk.DoubleVar(value=0.3)
        self.delay_scale = tk.Scale(self.sidebar, from_=0.0, to=1.0, resolution=0.1, orient='horizontal', variable=self.delay_var, bg='#e0e0e0')
        self.delay_scale.pack(pady=5)

        tk.Label(self.sidebar, text="Battery:", bg='#e0e0e0').pack(pady=5)
        self.battery_bar = ttk.Progressbar(self.sidebar, orient='horizontal', length=200, mode='determinate', maximum=100)
        self.battery_bar.pack(pady=5)
        self.battery_label = tk.Label(self.sidebar, text=f"Battery: {self.battery:.1f}%", bg='#e0e0e0')
        self.battery_label.pack(pady=2)

        self.battery_bar['value'] = self.battery
        
        self.draw_grid()
        
        
        tk.Label(self.sidebar, text="Radar (5x5 around rover):", bg='#e0e0e0').pack(pady=5)
        self.radar_canvas = tk.Canvas(self.sidebar, width=125, height=125, bg='white')
        self.radar_canvas.pack(pady=5)
        
        
        
    # ----------------------------------------------------------------------
    #  SAVE / LOAD GRID
    # ----------------------------------------------------------------------
    def _grid_to_dict(self):
        return {
            "rows": self.rows,
            "cols": self.cols,
            "grid": self.grid,          
            "start": self.start,
            "goal": self.goal
        }

    def save_grid(self):
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Save Grid"
        )
        if not filename:
            return
        data = self._grid_to_dict()
        try:
            with open(filename, "w") as f:
                json.dump(data, f, indent=2)
            self.status.config(text=f"Grid saved to {os.path.basename(filename)}")
        except Exception as e:
            messagebox.showerror("Save error", str(e))

    def load_grid(self):
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Load Grid"
        )
        if not filename:
            return
        try:
            with open(filename, "r") as f:
                data = json.load(f)

            if data.get("rows") != self.rows or data.get("cols") != self.cols:
                messagebox.showerror(
                    "Load error",
                    f"File is for a {data.get('rows')}×{data.get('cols')} grid, "
                    f"but UI is {self.rows}×{self.cols}."
                )
                return

            self.grid = data["grid"]
            self.start = tuple(data["start"])
            self.goal  = tuple(data["goal"])

            # reset everything that depends on the grid
            self.pos = self.start
            self.planned_path = None
            self.avoid_cells.clear()
            self.trails = []
            self.battery = BATTERY_FULL
            self.total_cost_used = 0.0
            self.running = False
            self.heading_to_recharge = False
            self.terrain_change_scheduled = False
            if hasattr(self, "change_events"):
                self.change_events = []

            self.draw_grid()
            self.update_battery_gui()
            self.status.config(text=f"Grid loaded from {os.path.basename(filename)}")
        except Exception as e:
            messagebox.showerror("Load error", str(e))
        
    def apply_scheduled_change(self, cell):
        if not self.running or cell not in self.planned_path:
            return

        r, c = cell
        current = self.grid[r][c]
        options = [t for t in ['flat', 'sandy', 'hazardous', 'trap'] if t != current]
        if not options:
            return
        self.grid[r][c] = random.choice(options)

        # Visual flash
        x1, y1 = c * self.cell_size, r * self.cell_size
        x2, y2 = x1 + self.cell_size, y1 + self.cell_size
        flash = self.canvas.create_rectangle(x1, y1, x2, y2, outline="yellow", width=4)
        self.root.after(1200, self.canvas.delete, flash)

        # Update path line
        self.draw_grid()
        self.draw_planned_path(self.planned_path, self.heur_box.get())

        # Remove from scheduled
        if self.change_events:
            self.change_events = [e for e in self.change_events if e[1] != cell]

        # Schedule next if any
        if self.change_events:
            delay, next_cell = self.change_events[0]
            self.root.after(int(delay), lambda: self.apply_scheduled_change(next_cell))
        else:
            self.terrain_change_scheduled = False
            self.status.config(text="All terrain changes applied.")
        
    def schedule_terrain_changes(self):
        if not self.terrain_change_scheduled or not self.running or not self.planned_path:
            return

        # Find current position in path
        try:
            cur_idx = self.planned_path.index(self.pos)
        except ValueError:
            return

        # Upcoming path (exclude start, current, goal, and last 2 cells)
        upcoming = self.planned_path[cur_idx + 3 : -2]  
        upcoming = [p for p in upcoming 
                   if self.grid[p[0]][p[1]] not in ('recharge', 'rock', 'cliff')]

        if len(upcoming) < 4:
            return  

        if len(upcoming) <= 6:
            idx1, idx2 = 0, len(upcoming) - 1
        else:
            idx1 = random.randint(0, len(upcoming) // 3)
            idx2 = random.randint(2 * len(upcoming) // 3, len(upcoming) - 1)
        cell1 = upcoming[idx1]
        cell2 = upcoming[idx2]

        # Schedule changes at random future steps
        steps_to_cell1 = self.planned_path.index(cell1) - cur_idx
        steps_to_cell2 = self.planned_path.index(cell2) - cur_idx

        delay1 = max(800, steps_to_cell1 * int(self.delay_var.get() * 1000) * random.uniform(0.8, 1.2))
        delay2 = max(800, steps_to_cell2 * int(self.delay_var.get() * 1000) * random.uniform(0.8, 1.2))

        self.change_events = [
            (delay1, cell1),
            (delay2, cell2)
        ]

        self.root.after(int(delay1), lambda: self.apply_scheduled_change(cell1))
        
    def enable_simulate_change(self):
        if not self.planned_path or len(self.planned_path) < 8:
            messagebox.showinfo("Info", "Plan a longer path first (need at least 8 cells).")
            return

        if hasattr(self, 'terrain_change_scheduled') and self.terrain_change_scheduled:
            messagebox.showinfo("Info", "Terrain changes already scheduled.")
            return

        self.terrain_change_scheduled = True
        self.change_events = []
        self.status.config(text="Terrain changes scheduled during movement.")
        messagebox.showinfo(
            "Ready",
            "2 terrain changes will occur at random times on the upcoming path."
        )
        
    def simulate_terrain_change(self):
    
        if not self.planned_path or len(self.planned_path) < 5:
            messagebox.showinfo("Info", "Plan a path first before simulating terrain changes.")
            return

        # Choose 2–4 random cells from rover's upcoming path (excluding recharge and current cell)
        safe_path = [cell for cell in self.planned_path[2:-2]
                    if self.grid[cell[0]][cell[1]] != 'recharge']
        if not safe_path:
            messagebox.showinfo("Info", "No suitable cells available to change.")
            return

        num_changes = random.randint(2, min(4, len(safe_path)))
        changed_cells = random.sample(safe_path, num_changes)

        for (r, c) in changed_cells:
            current = self.grid[r][c]
            possible_new = [t for t in ['flat', 'sandy', 'hazardous', 'rock']
                        if t != current and t != 'recharge']
            new_terrain = random.choice(possible_new)
            self.grid[r][c] = new_terrain

        self.draw_grid()
        # Keep showing old dotted path for visual clarity
        self.draw_planned_path(self.planned_path, self.heur_box.get())

        messagebox.showinfo("Simulation", f"Terrain changed at {len(changed_cells)} path cells.")
        self.status.config(text="Terrain changed along planned path — rover will adapt dynamically.")



    def update_radar(self):
    
        if not self.pos:
            return
        self.radar_canvas.delete('all')
        r0, c0 = self.pos
        radar_size = 5
        cell_px = 25
        half = radar_size // 2
        for dy in range(-half, half + 1):
            for dx in range(-half, half + 1):
                rr, cc = r0 + dy, c0 + dx
                if 0 <= rr < self.rows and 0 <= cc < self.cols:
                    terrain = self.grid[rr][cc]
                    color = TERRAIN_COLORS[terrain]
                else:
                    color = '#CCCCCC'  # out of bounds
                x1 = (dx + half) * cell_px
                y1 = (dy + half) * cell_px
                x2, y2 = x1 + cell_px, y1 + cell_px
                self.radar_canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline='gray')

        # Rover center mark
        center = radar_size * cell_px / 2
        self.radar_canvas.create_oval(center - 5, center - 5, center + 5, center + 5, fill='red')

        
    def update_battery_gui(self):
    
        percent = max(0, min(100, self.battery))
        self.battery_bar['value'] = percent

        # Choose color based on battery %
        if percent >= 80:
            color = 'green'
        elif percent >= 40:
            color = 'yellow'
        else:
            color = 'red'

        # Update bar color 
        style = ttk.Style()
        style.configure("Battery.Horizontal.TProgressbar", troughcolor='white', background=color)
        self.battery_bar.configure(style="Battery.Horizontal.TProgressbar")

        # Update text label
        if hasattr(self, 'battery_label'):
            self.battery_label.config(text=f"Battery: {percent:.1f}%")


    def on_heuristic_change(self, event):
        self.pos = self.start
        self.running = False
        self.replan_attempts = 0
        self.simulate_change = False
        self.battery = BATTERY_FULL
        self.total_cost_used = 0.0
        self.battery_bar['value'] = self.battery
        self.heading_to_recharge = False
        self.change_triggered = False  
        self.terrain_change_scheduled = False
        self.change_events = []

        self.clear_trails()
        self.planned_path = None
        self.draw_grid()
        self.status.config(text=f"Heuristic changed to {self.heur_box.get()}, rover reset to start")

    def draw_grid(self):
        self.canvas.delete('all')
        self.trails = []
        for r in range(self.rows):
            for c in range(self.cols):
                terrain = self.grid[r][c]
                color = TERRAIN_COLORS[terrain]
                x1, y1 = c * self.cell_size, r * self.cell_size
                x2, y2 = x1 + self.cell_size, y1 + self.cell_size
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline='gray')
        self.canvas.create_text(self.start[1] * self.cell_size + self.cell_size / 2,
                                self.start[0] * self.cell_size + self.cell_size / 2,
                                text="S", font=('Arial', 12, 'bold'), fill='black')
        self.canvas.create_text(self.goal[1] * self.cell_size + self.cell_size / 2,
                                self.goal[0] * self.cell_size + self.cell_size / 2,
                                text="G", font=('Arial', 12, 'bold'), fill='black')
        if self.rover and self.pos:
            x, y = self.pos[1] * self.cell_size + self.cell_size / 2, self.pos[0] * self.cell_size + self.cell_size / 2
            self.rover = self.canvas.create_oval(x - 10, y - 10, x + 10, y + 10, fill=HEUR_COLORS.get(self.heur_box.get(), 'lime'), outline='black')

    def draw_legend(self):
    
        legend_frame = tk.Frame(self.root, bg='#e0e0e0')
        legend_frame.pack(side='bottom', fill='x', pady=3)

        for terrain, color in TERRAIN_COLORS.items():
            fg = 'black' if terrain == 'sandy' else 'white'
            tk.Label(
                legend_frame,
                text=f"{TERRAIN_INDEX[terrain]}: {terrain.capitalize()}",
                bg=color,
                fg=fg,
                width=12,
                height=1
            ).pack(side='left', padx=2, pady=2)
            
    def mark_as_avoided(self, position):
        r, c = position
        if self.grid[r][c] in ('hazardous', 'trap'):
            self.grid[r][c] = 'rock'  # Make it impassable
            # Visual flash
            x1 = c * self.cell_size
            y1 = r * self.cell_size
            x2 = x1 + self.cell_size
            y2 = y1 + self.cell_size
            flash = self.canvas.create_rectangle(x1, y1, x2, y2, outline="red", width=4)
            self.root.after(800, self.canvas.delete, flash)
            self.draw_grid()

    def set_position(self, event):
        c = event.x // self.cell_size
        r = event.y // self.cell_size
        if in_bounds((r, c), self.grid):
            if self.grid[r][c] == 'rock':
                messagebox.showerror("Invalid", "Cannot place Start or Goal on rock terrain!")
                return
            if messagebox.askyesno("Set Position", "Set as Start? (No for Goal)"):
                self.start = (r, c)
                self.pos = self.start  # Update pos to new start
            else:
                self.goal = (r, c)
            self.draw_grid()
            self.status.config(text="Position updated")
            
        def temporarily_avoid(self, position, duration_ms=10000):
            r, c = position
            original = self.grid[r][c]
            if original in ('hazardous', 'trap'):
                self.grid[r][c] = 'rock'
                self.draw_grid()
                # Revert after delay
                self.root.after(duration_ms, lambda: self.revert_terrain(r, c, original))

        def revert_terrain(self, r, c, original):
            if 0 <= r < self.rows and 0 <= c < self.cols:
                self.grid[r][c] = original
                self.draw_grid()

    def change_terrain(self, event):
        c = event.x // self.cell_size
        r = event.y // self.cell_size
        if in_bounds((r, c), self.grid):
            current = self.grid[r][c]
            next_idx = (TERRAIN_INDEX[current] + 1) % len(TERRAIN_TYPES)
            self.grid[r][c] = TERRAIN_TYPES[next_idx]
            self.draw_grid()
            self.status.config(
                text=f"Terrain at ({r},{c}) changed to {self.grid[r][c]}"
            )


    def randomize_grid(self):
        self.grid = generate_grid(self.rows, self.cols)
        if self.grid[self.start[0]][self.start[1]] == 'rock':
            self.grid[self.start[0]][self.start[1]] = 'flat'
        if self.grid[self.goal[0]][self.goal[1]] == 'rock':
            self.grid[self.goal[0]][self.goal[1]] = 'flat'
        self.draw_grid()
        self.terrain_change_scheduled = False
        self.change_events = []
        self.status.config(text="Grid randomized")

    def plan_path(self):
        if not self.start or not self.goal:
            messagebox.showerror("Error", "Set Start and Goal first!")
            return
        h = self.heur_box.get()
        recharge_points = [(r, c) for r in range(self.rows) for c in range(self.cols) if self.grid[r][c] == 'recharge']
        path, cost, info = plan_with_recharges(self.grid, self.start, self.goal, self.battery, HEUR_FN_MAP[h], recharge_points)
        if not path:
            messagebox.showerror("Error", f"No path found: {info}")
            self.status.config(text=f"No path found: {info}")
            return
        self.planned_path = path
        self.replan_attempts = 0
        self.results.append((h, len(path) - 1, cost, info))
        self.status.config(text=f"Planned ({info}), est cost={cost:.1f}, steps={len(path)-1}")
        self.draw_planned_path(path, h)

    def draw_planned_path(self, path, heur):
        for t in self.trails:
            self.canvas.delete(t)
        self.trails = []
        color = HEUR_COLORS.get(heur, 'lime')
        for idx in range(len(path) - 1):
            r1, c1 = path[idx]
            r2, c2 = path[idx + 1]
            x1 = c1 * self.cell_size + self.cell_size / 2
            y1 = r1 * self.cell_size + self.cell_size / 2
            x2 = c2 * self.cell_size + self.cell_size / 2
            y2 = r2 * self.cell_size + self.cell_size / 2
            self.trails.append(self.canvas.create_line(x1, y1, x2, y2, fill=color, width=2, dash=(4, 3)))

    def run_path(self):
        if not hasattr(self, 'planned_path') or not self.planned_path:
            self.plan_path()
            if not self.planned_path:
                return
        else:
            self.status.config(text="Running existing path...")

        if self.running:
            return

        self.running = True
        self.pos = self.start
        self.battery = BATTERY_FULL
        self.total_cost_used = 0.0
        self.heading_to_recharge = False
        self.battery_bar['value'] = self.battery
        self.schedule_terrain_changes()  # Schedule 2 changes
        self.animate_rover()



    def animate_rover(self):
        if not self.running or not self.planned_path:
            self.running = False
            self.heading_to_recharge = False
            return

        try:
            cur_idx = self.planned_path.index(self.pos)
        except ValueError:
            cur_idx = -1

        next_idx = cur_idx + 1
        if next_idx >= len(self.planned_path):
            self.status.config(text="Reached goal or path ended.")
            self.running = False
            self.heading_to_recharge = False
            return

        next_pos = self.planned_path[next_idx]
        prev_pos = self.pos
        self.pos = next_pos

        if self.rover:
            self.canvas.delete(self.rover)

        x, y = self.pos[1] * self.cell_size + self.cell_size / 2, self.pos[0] * self.cell_size + self.cell_size / 2
        self.rover = self.canvas.create_oval(
            x - 10, y - 10, x + 10, y + 10,
            fill=HEUR_COLORS.get(self.heur_box.get(), 'lime'),
            outline='black'
        )

        if prev_pos != self.pos:
            self.trails.append(
                self.canvas.create_line(
                    prev_pos[1] * self.cell_size + self.cell_size / 2,
                    prev_pos[0] * self.cell_size + self.cell_size / 2,
                    self.pos[1] * self.cell_size + self.cell_size / 2,
                    self.pos[0] * self.cell_size + self.cell_size / 2,
                    fill=HEUR_COLORS.get(self.heur_box.get(), 'lime'),
                    width=2
                )
            )

        terrain = self.grid[self.pos[0]][self.pos[1]]
        step_cost = TERRAIN_COSTS[terrain]
        
        if self.simulate_change and not self.change_triggered and random.random() < 0.03:  # 3% chance per move
            
            change_candidates = [p for p in self.planned_path if p not in [self.start, self.goal]
                                and self.grid[p[0]][p[1]] not in ['recharge', 'rock', 'cliff']]
            if change_candidates:
                chosen = random.choice(change_candidates)
                old_terrain = self.grid[chosen[0]][chosen[1]]
                # Apply the terrain change
                new_terrain = random.choice(['sandy', 'hazardous', 'trap'])
                self.grid[chosen[0]][chosen[1]] = new_terrain
                self.change_triggered = True

                # Refresh the grid fully
                self.draw_grid()

                # Highlight the changed cell for visual clarity
                x1 = chosen[1] * self.cell_size
                y1 = chosen[0] * self.cell_size
                x2, y2 = x1 + self.cell_size, y1 + self.cell_size
                self.canvas.create_rectangle(x1, y1, x2, y2, outline="yellow", width=3)

                # Re-draw the rover + old path
                self.draw_planned_path(self.planned_path, self.heur_box.get())
                self.canvas.update()


                self.status.config(text=f"Terrain at {chosen} changed from {old_terrain} to {self.grid[chosen[0]][chosen[1]]}. Replanning...")
                messagebox.showinfo("Terrain Change", f"Terrain changed at {chosen}. Rover replanning...")

                
                self.pos = self.pos  # keep current position reference fresh
                self.root.after(3000, lambda: self.replan_after_change(resume=True))
                return



        if terrain in ('hazardous', 'trap'):
        
            self.avoid_cells.add(self.pos)
            messagebox.showinfo(
                "Alert",
                f"{terrain.capitalize()} terrain! Avoiding and replanning."
            )
            self.status.config(text=f"{terrain.capitalize()} detected – replanning…")

            # 2. Back-track 2-3 steps to a safe cell
            back_steps = random.randint(2, 3)
            safe_idx = max(0, self.planned_path.index(self.pos) - back_steps)
            safe_pos = self.planned_path[safe_idx]

            # Visual back-track
            for _ in range(back_steps):
                if self.trails:
                    self.canvas.delete(self.trails.pop())
                if len(self.planned_path) > 1:
                    cur_idx = self.planned_path.index(self.pos)
                    if cur_idx > 0:
                        self.pos = self.planned_path[cur_idx - 1]
                        self._redraw_rover()
                        self.canvas.update()
                        time.sleep(self.delay_var.get() * 0.4)

            # 3. Re-plan from safe cell
            self.pos = safe_pos
            self.root.after(300, lambda: self.replan_after_change(resume=True))
            return

        elif terrain == 'trap':
            self.mark_as_avoided(self.pos)
            messagebox.showinfo("Alert", "Trap triggered! Marking cell and replanning.")
            self.status.config(text="Trap detected. Marking and replanning...")

            back_steps = min(3, len(self.trails))
            for _ in range(back_steps):
                if self.trails:
                    self.canvas.delete(self.trails.pop())
                if len(self.planned_path) > 1:
                    idx = max(0, self.planned_path.index(self.pos) - 1)
                    self.pos = self.planned_path[idx]
                    self.draw_grid()
                    self.canvas.update()
                    time.sleep(self.delay_var.get())

            self.root.after(500, lambda: self.replan_after_change(resume=True))
            return

        # ------------------------------------------------------------
        # Battery and recharge logic 
        # ------------------------------------------------------------
        if self.heading_to_recharge:
            if terrain == 'recharge':
                self.battery = BATTERY_FULL
                self.battery_bar['value'] = self.battery
                self.replan_attempts = 0
                self.status.config(text="Arrived at recharge - battery refilled.")
                self.heading_to_recharge = False

                h = self.heur_box.get()
                recharge_points = [(r, c) for r in range(self.rows) for c in range(self.cols)
                                if self.grid[r][c] == 'recharge']
                p, c, info = plan_with_recharges(self.grid, self.pos, self.goal,
                                            self.battery, HEUR_FN_MAP[h], recharge_points)
                if p:
                    self.planned_path = p
                    self.draw_planned_path(p, h)
                    self.status.config(text=f"Replanned from recharge: {info}, est cost={c:.1f}")
                else:
                    self.status.config(text=f"Cannot reach goal from recharge: {info}")
                    self.running = False
                    self.heading_to_recharge = False
                    return

            elif self.battery <= 0:
                messagebox.showwarning("Low Power", "Battery depleted while heading to recharge!")
                self.running = False
                self.heading_to_recharge = False
                return

        else:
            recharge_points = [(r, c) for r in range(self.rows) for c in range(self.cols)
                        if self.grid[r][c] == 'recharge']
            allow_move = True

            if self.battery < 20 or (20 <= self.battery <= 25 and self.find_nearest_recharge_within(self.pos, 2.0)):
                rc, path_to_rc, c = find_nearest_recharge_within(
                    self.grid, self.pos, recharge_points,
                    9999.0 if self.battery < 20 else 2.0
                )

                if path_to_rc:
                    if c <= self.battery or (20 <= self.battery <= 25 and self.find_nearest_recharge_within(self.pos, 2.0)):
                        self.planned_path = path_to_rc + self.planned_path[next_idx:]
                        self.draw_planned_path(self.planned_path, self.heur_box.get())
                        self.status.config(text=f"Battery {self.battery:.1f}%, rerouting to recharge at {rc}")
                        self.heading_to_recharge = True
                    else:
                        self.status.config(text="Recharge within range but insufficient battery to reach.")
                        allow_move = False
                        self.running = False
                        return

                elif self.battery < 20:
                    self.status.config(text="No reachable recharge; stopping.")
                    allow_move = False
                    self.running = False
                    return

            if not allow_move or (step_cost > self.battery and terrain != 'recharge' and not self.heading_to_recharge):
                h = self.heur_box.get()
                p, c, info = plan_with_recharges(self.grid, self.pos, self.goal,
                                            self.battery, HEUR_FN_MAP[h], recharge_points)
                if p:
                    self.planned_path = p
                    self.draw_planned_path(p, h)
                    self.status.config(text=f"Rerouted to include recharge(s): {info}, est cost={c:.1f}")
                else:
                    self.status.config(text=f"No reachable recharge: {info}")
                    self.running = False
                    return

        # ------------------------------------------------------------
        # Proceed with move
        # ------------------------------------------------------------
        self.battery -= step_cost
        self.total_cost_used += step_cost
        self.update_battery_gui()


        if terrain == 'recharge' and not self.heading_to_recharge:
            self.battery = BATTERY_FULL
            self.battery_bar['value'] = self.battery
            self.status.config(text="Arrived at recharge - battery refilled.")
            h = self.heur_box.get()
            p, c, info = plan_with_recharges(self.grid, self.pos, self.goal,
                                        self.battery, HEUR_FN_MAP[h], recharge_points)
            if p:
                self.planned_path = p
                self.draw_planned_path(p, h)
                self.status.config(text=f"Replanned from recharge: {info}, est cost={c:.1f}")
            else:
                self.status.config(text=f"Cannot reach goal from recharge: {info}")
                self.running = False
                return

        if self.pos == self.goal:
            self.status.config(text="Goal reached!")
            self.running = False
            self.heading_to_recharge = False
            self.replan_attempts = 0
            return

        self.canvas.update()
        self.update_radar()
        self.root.after(int(self.delay_var.get() * 1000), self.animate_rover)


    
    def find_nearest_recharge_within(self, current, max_euclid_dist):
        recharge_points = [(r, c) for r in range(self.rows) for c in range(self.cols) if self.grid[r][c] == 'recharge']
        return find_nearest_recharge_within(self.grid, current, recharge_points, max_euclid_dist)

    def clear_trails(self):
        for t in self.trails:
            self.canvas.delete(t)
        self.trails = []
        self.status.config(text="Paths cleared")
        
    def _redraw_rover(self):
        
        if self.rover:
            self.canvas.delete(self.rover)
        x = self.pos[1] * self.cell_size + self.cell_size // 2
        y = self.pos[0] * self.cell_size + self.cell_size // 2
        self.rover = self.canvas.create_oval(
            x - 10, y - 10, x + 10, y + 10,
            fill=HEUR_COLORS.get(self.heur_box.get(), 'lime'),
            outline='black'
        )

    def clear_results(self):
        self.results = []
        self.start = (0, 0)
        self.replan_attempts = 0
        self.pos = self.start
        self.heading_to_recharge = False
        self.draw_grid()
        self.terrain_change_scheduled = False
        self.change_events = []
        self.status.config(text="Results cleared and start reset to (0,0)")

    def show_report(self):
        if not self.results:
            messagebox.showinfo("Report", "No results to show")
            return
        top = Toplevel(self.root)
        top.title("Heuristic Report")
        text = tk.Text(top, width=60, height=20)
        text.pack()
        text.insert('end', f"{'Heuristic':20}{'Moves':>8}{'Cost':>8}{'Info':>20}\n")
        for h, m, c, info in self.results:
            text.insert('end', f"{h:20}{m:8}{c:8.1f}{info:>20}\n")
        text.config(state='disabled')

    def compare_all_heuristics(self):
        
            self.clear_trails()
            paths_dict = {}
            recharge_points = [(r, c) for r in range(self.rows) for c in range(self.cols) if self.grid[r][c] == 'recharge']

            for h in HEUR_FN_MAP.keys():
                path, cost, info = plan_with_recharges(self.grid, self.start, self.goal, BATTERY_FULL, HEUR_FN_MAP[h], recharge_points)
                if path:
                    paths_dict[h] = (path, cost, info)

            if not paths_dict:
                messagebox.showinfo("Info  ", "No paths found for any heuristic!")
                return

            top = Toplevel(self.root)
            top.title("Heuristic Path Comparison")
            canvas_size = min(self.cols * self.cell_size, 500)
            cell_size = canvas_size // self.cols

            main_frame = tk.Frame(top)
            main_frame.pack(padx=12, pady=12, fill=tk.BOTH, expand=True)

           
            sidebar = tk.Frame(main_frame, bg='#e0e0e0', width=180)
            sidebar.pack(side='left', fill='y', padx=(0, 10))
            sidebar.pack_propagate(False)

            tk.Label(sidebar, text="Select Heuristic:", bg='#e0e0e0', font=('Arial', 10, 'bold')).pack(pady=(0, 8))

            self.compare_var = tk.StringVar(value=list(paths_dict.keys())[0])  # default = first
            self.compare_mode = "single"  # 'single' or 'all'

            def draw_single_path():
                self.compare_mode = "single"
                draw_path(self.compare_var.get())

            def draw_all_paths():
                self.compare_mode = "all"
                for item in self.compare_trails:
                    self.compare_canvas.delete(item)
                self.compare_trails = []

                
                self._draw_compare_grid(cell_size)

                
                for h_name, (path, cost, info) in paths_dict.items():
                    color = HEUR_COLORS[h_name]
                    for idx in range(len(path) - 1):
                        r1, c1 = path[idx]
                        r2, c2 = path[idx + 1]
                        x1 = c1 * cell_size + cell_size // 2
                        y1 = r1 * cell_size + cell_size // 2
                        x2 = c2 * cell_size + cell_size // 2
                        y2 = r2 * cell_size + cell_size // 2
                        self.compare_trails.append(
                            self.compare_canvas.create_line(x1, y1, x2, y2, fill=color, width=2, capstyle=tk.ROUND)
                        )
                self.compare_info.config(text=f"ALL PATHS SHOWN\n{len(paths_dict)} heuristics")

            def draw_path(heur):
                for item in self.compare_trails:
                    self.compare_canvas.delete(item)
                self.compare_trails = []

                
                self._draw_compare_grid(cell_size)

                if heur in paths_dict:
                    path, cost, info = paths_dict[heur]
                    color = HEUR_COLORS[heur]
                    for idx in range(len(path) - 1):
                        r1, c1 = path[idx]
                        r2, c2 = path[idx + 1]
                        x1 = c1 * cell_size + cell_size // 2
                        y1 = r1 * cell_size + cell_size // 2
                        x2 = c2 * cell_size + cell_size // 2
                        y2 = r2 * cell_size + cell_size // 2
                        self.compare_trails.append(
                            self.compare_canvas.create_line(x1, y1, x2, y2, fill=color, width=3)
                        )
                    moves = len(path) - 1
                    self.compare_info.config(text=f"{heur}\nMoves: {moves}\nCost: {cost:.1f}\n{info}")

           
            for h in paths_dict.keys():
                rb = tk.Radiobutton(
                    sidebar,
                    text=h,
                    variable=self.compare_var,
                    value=h,
                    bg='#e0e0e0',
                    selectcolor=HEUR_COLORS[h],
                    indicatoron=0,
                    width=18,
                    command=draw_single_path
                )
                rb.pack(pady=2, anchor='w')

            
            tk.Button(
                sidebar,
                text="Show All Paths",
                command=draw_all_paths,
                bg='#4CAF50',
                fg='white',
                font=('Arial', 9, 'bold'),
                width=18
            ).pack(pady=(15, 5))

            
            self.compare_canvas = tk.Canvas(main_frame, width=canvas_size, height=canvas_size, bg='white')
            self.compare_canvas.pack(side='left')
            self.compare_trails = []

            
            self.compare_info = tk.Label(main_frame, text="", bg='#f0f0f0', justify='left', font=('Arial', 9))
            self.compare_info.pack(pady=8, fill='x')

            
            def _draw_compare_grid(cell_size):
                for r in range(self.rows):
                    for c in range(self.cols):
                        color = TERRAIN_COLORS[self.grid[r][c]]
                        x1, y1 = c * cell_size, r * cell_size
                        x2, y2 = x1 + cell_size, y1 + cell_size
                        self.compare_trails.append(
                            self.compare_canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline='#ddd')
                        )
                
                self.compare_trails.append(
                    self.compare_canvas.create_text(
                        self.start[1] * cell_size + cell_size // 2,
                        self.start[0] * cell_size + cell_size // 2,
                        text="S", font=('Arial', 10, 'bold'), fill='black'
                    )
                )
                self.compare_trails.append(
                        self.compare_canvas.create_text(
                        self.goal[1] * cell_size + cell_size // 2,
                        self.goal[0] * cell_size + cell_size // 2,
                        text="G", font=('Arial', 10, 'bold'), fill='black'
                    )
                )

            self._draw_compare_grid = _draw_compare_grid  

            
            draw_single_path()
        
        

    def draw_path(self, path, heur, old=False):
        color = "gray" if old else HEUR_COLORS.get(heur, 'lime')
        style = (2, 3) if old else ()
        for idx in range(len(path) - 1):
            r1, c1 = path[idx]
            r2, c2 = path[idx + 1]
            x1 = c1 * self.cell_size + self.cell_size / 2
            y1 = r1 * self.cell_size + self.cell_size / 2
            x2 = c2 * self.cell_size + self.cell_size / 2
            y2 = r2 * self.cell_size + self.cell_size / 2
            self.trails.append(
                self.canvas.create_line(
                    x1, y1, x2, y2, fill=color, width=2,
                    dash=style
                )
            )

    def rover_death_animation(self):
        
            if not self.rover:
                return
            x1, y1, x2, y2 = self.canvas.coords(self.rover)
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            radius = 10

            # Flash red 3 times
            def flash(step=0):
                if step >= 6:
                    self.canvas.delete(self.rover)
                    self.rover = None
                    return
                fill = "red" if step % 2 == 0 else "darkred"
                if self.rover:
                    self.canvas.delete(self.rover)
                self.rover = self.canvas.create_oval(
                    center_x - radius, center_y - radius,
                    center_x + radius, center_y + radius,
                    fill=fill, outline="black"
                )
                self.root.after(200, lambda: flash(step + 1))
            flash()

    def replan_after_change(self, resume=False):
        self.replan_attempts += 1
        if self.replan_attempts > self.max_replan_attempts:
            self.status.config(text="Too many replan failures – rover dies!")
            messagebox.showwarning("Rover Failed", "Could not find a safe path after 3 attempts.\nRover has run out of options and dies.")
            self.rover_death_animation()
            self.running = False
            self.change_triggered = False
            return

        h = self.heur_box.get()
        recharge_points = [(r, c) for r in range(self.rows) for c in range(self.cols)
                            if self.grid[r][c] == 'recharge']
        p, c, info = plan_with_recharges(self.grid, self.pos, self.goal, self.battery,
                                    HEUR_FN_MAP[h], recharge_points, avoid_cells=self.avoid_cells)

        if p:
            
            self.draw_path(self.planned_path, h, old=True)
            
            self.draw_path(p, h, old=False)
            self.planned_path = p
            self.status.config(text=f"Replanned after hazard (attempt {self.replan_attempts}): {info}")
            self.change_triggered = False
            if resume:
                self.root.after(int(self.delay_var.get() * 1000), self.animate_rover)
        else:
            
            self.status.config(text=f"Replan failed (attempt {self.replan_attempts}) – trying again...")
            self.root.after(500, lambda: self.replan_after_change(resume=resume))



# --- Run ---
if __name__ == "__main__":
    root = tk.Tk()
    app = RoverSim(root)
    root.mainloop()