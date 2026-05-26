import gurobipy as gp
from gurobipy import GRB
from datetime import datetime

#------------------------------------------------------------
# パターンA: コンフィギュレーション数固定・継続時間固定モデル
#（ユーザー設定による長方形率制御・高速化版）
#------------------------------------------------------------

def solve_integrated_optimization_fixed_both():
    # 1. モデルの初期化
    model = gp.Model("Grid_Fixed_Configuration_Fixed_Duration")

    # パラメータ設定 (0: 出力抑制, 1: 標準出力)
    model.Params.OutputFlag = 1
    # 【追加】非凸な二次制約・等式制約を大域的最適化で解くための設定
    model.Params.NonConvex = 2

    # 2. 定数・集合の定義（ユーザー設定項目）
    H, W = 8, 8                      # グリッドサイズ（縦H, 横W）
    MAX_LOAD_LIMIT = 80              # 各セルの最大累積負荷
    NUM_STATES = 5                   # 状態遷移数 (固定)
    UNIT_DURATION = 1.0              # 各コンフィギュレーションの継続時間 (固定)

    # 【新規追加】長方形率の下限値 (0.0 から 1.0 の範囲で設定)
    # 1.0: 完全な長方形を強制
    # 0.8: 要求セル数が外接矩形面積の80%以上であれば許容（凸凹やL字を一部許容）
    RECT_RATIO_LIMIT = 0.8

    # モジュール設定
    MODULE_LOADS = [8, 7, 6, 5, 4, 13, 2, 9, 5, 9]
    MODULE_CELL_REQUIREMENTS = [1, 2, 3, 4, 5, 3, 4, 7, 8, 9]

    NUM_CELLS = H * W
    CELLS = range(NUM_CELLS)
    MODULES = range(len(MODULE_LOADS))
    STATES = range(NUM_STATES)

    # 隣接リスト取得関数
    def get_neighbors(c, h, w):
        neighbors = []
        row, col = divmod(c, w)
        if row > 0: neighbors.append(c - w)     # 上
        if row < h - 1: neighbors.append(c + w) # 下
        if col > 0: neighbors.append(c - 1)     # 左
        if col < w - 1: neighbors.append(c + 1) # 右
        return neighbors

    # 物理的に存在する隣接エッジのみを事前に列挙（高速化）
    EDGES = []
    for c in CELLS:
        for cn in get_neighbors(c, H, W):
            EDGES.append((c, cn))

    # 3. 決定変数の定義
    x = model.addVars(CELLS, STATES, MODULES, vtype=GRB.BINARY, name="x")
    v = model.addVars(CELLS, STATES, MODULES, vtype=GRB.BINARY, name="v")
    f = model.addVars(EDGES, STATES, MODULES, lb=0, name="flow")

    # 外接矩形（Bounding Box）のための整数変数定義
    r_min = model.addVars(STATES, MODULES, vtype=GRB.INTEGER, lb=0, ub=H-1, name="r_min")
    r_max = model.addVars(STATES, MODULES, vtype=GRB.INTEGER, lb=0, ub=H-1, name="r_max")
    c_min = model.addVars(STATES, MODULES, vtype=GRB.INTEGER, lb=0, ub=W-1, name="c_min")
    c_max = model.addVars(STATES, MODULES, vtype=GRB.INTEGER, lb=0, ub=W-1, name="c_max")
    w_box = model.addVars(STATES, MODULES, vtype=GRB.INTEGER, lb=1, ub=W, name="w_box")
    h_box = model.addVars(STATES, MODULES, vtype=GRB.INTEGER, lb=1, ub=H, name="h_box")

    # 4. 目的関数の設定
    model.setObjective(0.0, GRB.MAXIMIZE)

    # 5. 制約条件の追加
    for s in STATES:
        for m in MODULES:
            Rm = MODULE_CELL_REQUIREMENTS[m]

            # (A) モジュール配置必要数制約
            model.addConstr(gp.quicksum(x[c, s, m] for c in CELLS) == Rm)

            # (B) 連結性（塊）制約
            model.addConstr(gp.quicksum(v[c, s, m] for c in CELLS) == 1)

            # (E) 長方形幾何制約（設定された閾値に基づく不等式制御）
            model.addConstr(w_box[s, m] == c_max[s, m] - c_min[s, m] + 1)
            model.addConstr(h_box[s, m] == r_max[s, m] - r_min[s, m] + 1)

            # 長方形率制約の適用式: 要求セル数 >= 外接矩形面積 × 長方形率下限
            model.addConstr(Rm >= RECT_RATIO_LIMIT * (w_box[s, m] * h_box[s, m]))

            for c in CELLS:
                model.addConstr(v[c, s, m] <= x[c, s, m])

                adj = get_neighbors(c, H, W)
                out_flow = gp.quicksum(f[c, cn, s, m] for cn in adj)
                in_flow = gp.quicksum(f[cp, c, s, m] for cp in adj)

                # フロー収支
                model.addConstr(
                    out_flow - in_flow == (Rm - 1) * v[c, s, m] - (x[c, s, m] - v[c, s, m])
                )
                model.addConstr(out_flow <= (Rm - 1) * x[c, s, m])

                # 包含制約 (Big-M定式化)
                row_idx, col_idx = divmod(c, W)
                model.addConstr(row_idx >= r_min[s, m] - H * (1 - x[c, s, m]))
                model.addConstr(row_idx <= r_max[s, m] + H * (1 - x[c, s, m]))
                model.addConstr(col_idx >= c_min[s, m] - W * (1 - x[c, s, m]))
                model.addConstr(col_idx <= c_max[s, m] + W * (1 - x[c, s, m]))

        # (C) セル排他制約
        for c in CELLS:
            model.addConstr(gp.quicksum(x[c, s, m] for m in MODULES) <= 1)

    # (D) 累積負荷制約
    for c in CELLS:
        model.addConstr(
            gp.quicksum(x[c, s, m] * UNIT_DURATION * MODULE_LOADS[m] for s in STATES for m in MODULES) <= MAX_LOAD_LIMIT
        )

    # 6. 最適化の実行
    model.optimize()

    # 7. 結果出力
    if model.Status == GRB.OPTIMAL or model.Status == GRB.TIME_LIMIT:
        timestamp = datetime.now().strftime("RR_%Y%m%d_%H%M")
        filename = f"{timestamp}.rep"

        with open(filename, "w", encoding="utf-8") as f_out:
            f_out.write("Optimization Result Report\n")
            f_out.write("==========================\n\n")

            f_out.write("[Model Parameters]\n")
            f_out.write(f"Grid Size: {H}x{W}\n")
            f_out.write(f"Max Load Limit: {MAX_LOAD_LIMIT}\n")
            f_out.write(f"Module Loads: {MODULE_LOADS}\n")
            f_out.write(f"Module Requirements: {MODULE_CELL_REQUIREMENTS}\n")
            f_out.write(f"Fixed States: {NUM_STATES}\n")
            f_out.write(f"Fixed Unit Duration: {UNIT_DURATION}\n")
            f_out.write(f"Rectangular Ratio Limit: {RECT_RATIO_LIMIT:.2f}\n") # レポートへ追加
            f_out.write("\n")

            f_out.write("Objective Value: Feasible Solution Found\n\n")

            f_out.write("[Performance Metrics]\n")
            f_out.write(f"Execution Time (s): {model.Runtime:.4f}\n")
            if hasattr(model, "MIPGap"):
                f_out.write(f"MIP Gap: {model.MIPGap:.4%}\n")
            f_out.write(f"Nodes Explored: {model.NodeCount}\n")
            f_out.write("\n")

            for s in STATES:
                f_out.write(f"--- State {s} ---\n")
                for i in range(H):
                    row_str = ""
                    for j in range(W):
                        c = i * W + j
                        found = False
                        for m in MODULES:
                            if x[c, s, m].X > 0.5:
                                row_str += f"[M{m:01d}(L{MODULE_LOADS[m]:>2})] "
                                found = True
                                break
                        if not found:
                            row_str += "[ Empty ] "
                    f_out.write(row_str + "\n")
                f_out.write("\n")

        print(f"\nレポートファイル '{filename}' を出力しました。")
        print(f"\n実行可能解が見つかりました（長方形率下限: {RECT_RATIO_LIMIT:.2f}）。")
    else:
        print("解が見つかりませんでした。")

if __name__ == "__main__":
    solve_integrated_optimization_fixed_both()
