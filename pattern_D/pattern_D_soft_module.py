import math
import gurobipy as gp
from gurobipy import GRB

def solve_rectilinear_floorplanning_optimized():
    # 1. モデルの初期化
    model = gp.Model("Rectilinear_Soft_Module_Floorplanning_Optimization")

    # パラメータ設定
    model.Params.OutputFlag = 1
    model.Params.TimeLimit = 300  # 実行時間制限（秒）

    # 2. 定数・集合の定義
    H, W = 5, 5                       # フロアプラン領域（Fixed-outline） [cite: 57]
    MAX_LOAD_LIMIT = 50               # 各セルの最大累積負荷
    MIN_DURATION = 1.0                # 各状態の最小継続時間

    # 論文に基づく形状制約パラメータ [cite: 275, 280]
    AR_LOW = 1/3.0                    # アスペクト比下限 (AR_l)
    AR_HIGH = 3.0                     # アスペクト比上限 (AR_h)
    RR_LOW = 0.4                      # 長方形比率下限 (RR_l)

    # モジュール設定 (最小面積と負荷)
    MODULE_MIN_AREAS = [1, 2, 3, 4, 5] # Area_min(mi)
    MODULE_LOADS = [10, 9, 8, 7, 6]

    # Big-M定数
    MAX_DURATION = MAX_LOAD_LIMIT / min(MODULE_LOADS)
    MAX_STATES_LIMIT = 5

    NUM_CELLS = H * W
    CELLS = range(NUM_CELLS)
    MODULES = range(len(MODULE_LOADS))
    STATES = range(MAX_STATES_LIMIT)

    # 3. 決定変数の定義
    # 配置フラグ (x_i)
    x = model.addVars(CELLS, STATES, MODULES, vtype=GRB.BINARY, name="x")
    # 状態有効フラグ (z_s)
    z = model.addVars(STATES, vtype=GRB.BINARY, name="z")
    # 継続時間 (d_s)
    d = model.addVars(STATES, vtype=GRB.CONTINUOUS, lb=0.0, name="d")
    # 稼働時間 (y = x * d) [線形化用]
    y = model.addVars(CELLS, STATES, MODULES, vtype=GRB.CONTINUOUS, lb=0.0, name="y")

    # 論文のバウンディングボックス (BB_i) 変数 [cite: 61, 62]
    min_r = model.addVars(STATES, MODULES, lb=0, ub=H-1, name="min_r")
    max_r = model.addVars(STATES, MODULES, lb=0, ub=H-1, name="max_r")
    min_c = model.addVars(STATES, MODULES, lb=0, ub=W-1, name="min_c")
    max_c = model.addVars(STATES, MODULES, lb=0, ub=W-1, name="max_c")

    w_bb = model.addVars(STATES, MODULES, lb=1, ub=W, name="w_bb")
    h_bb = model.addVars(STATES, MODULES, lb=1, ub=H, name="h_bb")

    area_bb = model.addVars(STATES, MODULES, lb=1, ub=NUM_CELLS, name="area_bb")

    # 連結性用変数 (Flow-based) [cite: 65]
    v = model.addVars(CELLS, STATES, MODULES, vtype=GRB.BINARY, name="v")
    f = model.addVars(CELLS, CELLS, STATES, MODULES, lb=0, name="flow")

    # 4. 目的関数の設定
    # 総稼働時間の最大化
    model.setObjective(gp.quicksum(d[s] for s in STATES), GRB.MAXIMIZE)

    # 5. 制約条件の追加
    def get_neighbors(c, h, w):
        neighbors = []
        row, col = divmod(c, w)
        if row > 0: neighbors.append(c - w)
        if row < h - 1: neighbors.append(c + w)
        if col > 0: neighbors.append(c - 1)
        if col < w - 1: neighbors.append(c + 1)
        return neighbors

    for s in STATES:
        # (A) 状態の順序性と継続時間
        if s > 0:
            model.addConstr(z[s] <= z[s-1], name=f"order_{s}")
        model.addConstr(d[s] >= MIN_DURATION * z[s])
        model.addConstr(d[s] <= MAX_DURATION * z[s])

        for m in MODULES:
            # (B) 最小面積制約 [cite: 66, 119]
            # Area(mi) >= Area_min(mi)
            actual_area = gp.quicksum(x[c, s, m] for c in CELLS)
            model.addConstr(actual_area >= MODULE_MIN_AREAS[m] * z[s])

            # (C) バウンディングボックスの特定 (Big-M法)
            for c_idx in CELLS:
                r, c = divmod(c_idx, W)
                model.addConstr(min_r[s, m] <= r + H * (1 - x[c_idx, s, m]))
                model.addConstr(max_r[s, m] >= r - H * (1 - x[c_idx, s, m]))
                model.addConstr(min_c[s, m] <= c + W * (1 - x[c_idx, s, m]))
                model.addConstr(max_c[s, m] >= c - W * (1 - x[c_idx, s, m]))

            # BBの寸法計算
            model.addConstr(w_bb[s, m] == max_c[s, m] - min_c[s, m] + 1)
            model.addConstr(h_bb[s, m] == max_r[s, m] - min_r[s, m] + 1)

            # (D) アスペクト比制約 (AR_i = h_i / w_i) [cite: 67, 129]
            model.addConstr(h_bb[s, m] >= AR_LOW * w_bb[s, m] - H * (1 - z[s]))
            model.addConstr(h_bb[s, m] <= AR_HIGH * w_bb[s, m] + H * (1 - z[s]))

            # (E) 長方形比率制約 (RR_i = Area / Area_BB) [cite: 68, 80]
            # Area_BB = w_bb * h_bb
            model.addGenConstrMul(w_bb[s, m], h_bb[s, m], area_bb[s, m])
            model.addConstr(actual_area >= RR_LOW * area_bb[s, m] - NUM_CELLS * (1 - z[s]))

            # (F) 連結性制約 (Simple Rectilinear Polygon) [cite: 64, 65]
            model.addConstr(gp.quicksum(v[c, s, m] for c in CELLS) == z[s])
            for c in CELLS:
                model.addConstr(v[c, s, m] <= x[c, s, m])
                adj = get_neighbors(c, H, W)
                out_flow = gp.quicksum(f[c, cn, s, m] for cn in adj)
                in_flow = gp.quicksum(f[cp, c, s, m] for cp in adj)
                # Area-1 のフローを循環させることで連結を保証 (Rmを実際の面積変数で代用)
                # 簡易化のため定数Rmを使用。厳密にはactual_area-1とする
                Rm_const = MODULE_MIN_AREAS[m]
                model.addConstr(out_flow - in_flow == (Rm_const - 1) * v[c, s, m] - (x[c, s, m] - v[c, s, m]))
                model.addConstr(out_flow <= NUM_CELLS * x[c, s, m])

            # (G) 変数の線形化 (y = x * d)
            for c in CELLS:
                model.addConstr(y[c, s, m] <= MAX_DURATION * x[c, s, m])
                model.addConstr(y[c, s, m] <= d[s])
                model.addConstr(y[c, s, m] >= d[s] - MAX_DURATION * (1 - x[c, s, m]))

        # (H) セル排他制約
        for c in CELLS:
            model.addConstr(gp.quicksum(x[c, s, m] for m in MODULES) <= 1)

    # (I) 累積負荷制約 (セルの寿命)
    for c in CELLS:
        model.addConstr(
            gp.quicksum(y[c, s, m] * MODULE_LOADS[m] for s in STATES for m in MODULES) <= MAX_LOAD_LIMIT
        )

    # 6. 最適化の実行
    model.optimize()

    # 7. 結果出力
    if model.Status in [GRB.OPTIMAL, GRB.TIME_LIMIT]:
        print(f"\n総稼働時間: {model.ObjVal:.4f}")
        for s in STATES:
            if z[s].X < 0.5: continue
            print(f"\n--- State {s} (Duration: {d[s].X:.4f}) ---")
            for i in range(H):
                row_str = ""
                for j in range(W):
                    c = i * W + j
                    found = False
                    for m in MODULES:
                        if x[c, s, m].X > 0.5:
                            row_str += f"[M{m}(AR:{h_bb[s,m].X/w_bb[s,m].X:.1f})] "
                            found = True
                            break
                    if not found: row_str += "[  Empty   ] "
                print(row_str)
            for m in MODULES:
                if z[s].X > 0.5:
                    print(f" Module {m}: Area={sum(x[c,s,m].X for c in CELLS)}, BB={w_bb[s,m].X}x{h_bb[s,m].X}, RR={sum(x[c,s,m].X for c in CELLS)/area_bb[s,m].X:.2f}")
    else:
        print("解が見つかりませんでした。")

if __name__ == "__main__":
    solve_rectilinear_floorplanning_optimized()
