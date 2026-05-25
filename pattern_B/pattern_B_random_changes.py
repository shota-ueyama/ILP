import random

def get_neighbors(c, h, w):
    neighbors = []
    row, col = divmod(c, w)
    if row > 0: neighbors.append(c - w)
    if row < h - 1: neighbors.append(c + w)
    if col > 0: neighbors.append(c - 1)
    if col < w - 1: neighbors.append(c + 1)
    return neighbors

def generate_random_connected_layout(H, W, CELLS, MODULES, MODULE_CELL_REQUIREMENTS):
    while True:
        layout = {} 
        available_cells = set(CELLS)
        success = True

        for m_idx in MODULES:
            req = MODULE_CELL_REQUIREMENTS[m_idx]
            if not available_cells:
                success = False; break

            start_node = random.choice(list(available_cells))
            current_module_cells = {start_node}

            while len(current_module_cells) < req:
                candidates = set()
                for c in current_module_cells:
                    for n in get_neighbors(c, H, W):
                        if n in available_cells and n not in current_module_cells:
                            candidates.add(n)
                if not candidates:
                    success = False; break
                next_node = random.choice(list(candidates))
                current_module_cells.add(next_node)

            if not success: break
            for c in current_module_cells:
                layout[c] = m_idx
                available_cells.remove(c)

        if success:
            return layout

def run_single_simulation():
    H, W = 5, 5
    MAX_LOAD_LIMIT = 50
    NUM_STATES = 5
    MODULE_LOADS = [10, 9, 3, 2, 1]
    MODULE_CELL_REQUIREMENTS = [2, 2, 3, 2, 1]
    NUM_CELLS = H * W
    CELLS = list(range(NUM_CELLS))
    MODULES = range(len(MODULE_LOADS))

    all_states_layouts = [generate_random_connected_layout(H, W, CELLS, MODULES, MODULE_CELL_REQUIREMENTS) for _ in range(NUM_STATES)]

    cell_load_factors = [0] * NUM_CELLS
    for layout in all_states_layouts:
        for c, m_idx in layout.items():
            cell_load_factors[c] += MODULE_LOADS[m_idx]

    max_total_load_factor = max(cell_load_factors)
    d_per_state = MAX_LOAD_LIMIT // max_total_load_factor if max_total_load_factor > 0 else MAX_LOAD_LIMIT
    
    return d_per_state * NUM_STATES

def main():
    NUM_TRIALS = 10
    results = []

    print(f"{'試行回数':<8} | {'合計動作サイクル数':<15}")
    print("-" * 30)

    for i in range(1, NUM_TRIALS + 1):
        total_cycles = run_single_simulation()
        results.append(total_cycles)
        print(f"Trial {i:>2} | {total_cycles:>15}")

    average_cycles = sum(results) / len(results)
    print("-" * 30)
    print(f"10回の平均合計動作サイクル数: {average_cycles:.2f}")

if __name__ == "__main__":
    main()
