import math
import gurobipy as gp
from gurobipy import GRB
from datetime import datetime

#-------------------------------------------------------------
# パターンD: コンフィギュレーション数可変・継続時間可変モデル
#-------------------------------------------------------------

def solve_integrated_optimization_variable_duration():
# 1. モデルの初期化
    model = gp.Model("Grid_Configuration_Count_and_Duration_Optimization")

    # パラメータ設定 (0: 出力抑制, 1: 標準出力)
    model.Params.OutputFlag = 1

# 2. 定数・集合の定義
    H, W = 5, 5                       # グリッドサイズ
    MAX_LOAD_LIMIT = 20               # 各セルの最大累積負荷

    # モジュール設定
    MODULE_LOADS = [10, 8, 6, 4, 2]
    MODULE_CELL_REQUIREMENTS = [1, 2, 3, 4, 5]

    # 継続時間の理論的最大値 (Big-Mとして使用)
    MAX_DURATION = MAX_LOAD_LIMIT / min(MODULE_LOADS)

    # 探索対象となる最大コンフィギュレーション数（上限値）
    MAX_STATES_LIMIT = math.floor(MAX_LOAD_LIMIT / min(MODULE_LOADS)) + 1

    NUM_CELLS = H * W
    CELLS = range(NUM_CELLS)
    MODULES = range(len(MODULE_LOADS))
    STATES = range(MAX_STATES_LIMIT)



# 3. 決定変数の定義
    # x[c,s,m]: 配置フラグ (バイナリ)
    x = model.addVars(CELLS, STATES, MODULES, vtype=GRB.BINARY, name="x")

    # z[s]: 状態sを使用するかどうかのフラグ (バイナリ)
    z = model.addVars(STATES, vtype=GRB.BINARY, name="z")

    # d[s]: 状態sの継続時間 (連続変数)
    d = model.addVars(STATES, vtype=GRB.INTEGER, lb=0.0, name="d")

    # y[c,s,m]: x[c,s,m] と d[s] の積を表現する連続変数 (稼働時間)
    y = model.addVars(CELLS, STATES, MODULES, vtype=GRB.INTEGER, lb=0.0, name="y")

    # v[c,s,m]: 連結性判定のための根(Root)フラグ
    v = model.addVars(CELLS, STATES, MODULES, vtype=GRB.BINARY, name="v")

    # f[c,cn,s,m]: 隣接セル間のフロー量
    f = model.addVars(CELLS, CELLS, STATES, MODULES, lb=0, name="flow")



# 4. 目的関数の設定
    # システムの総稼働時間（各状態の継続時間の和）を最大化
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
        # (A) コンフィギュレーションの順序性および継続時間制約
        if s > 0:
            model.addConstr(z[s] <= z[s-1], name=f"order_state_{s}")

            model.addConstr(d[s] <= d[s-1], name=f"symmetry_duration_{s}")

        # 状態sが有効な場合のみ継続時間が割り当てられる
        model.addConstr(d[s] <= MAX_DURATION * z[s])

        for m in MODULES:
            Rm = MODULE_CELL_REQUIREMENTS[m]

            # (B) モジュール配置必要数制約
            model.addConstr(gp.quicksum(x[c, s, m] for c in CELLS) == Rm * z[s])

            # (C) 連結性制約
            #どこか1つのセルをRootとする
            model.addConstr(gp.quicksum(v[c, s, m] for c in CELLS) == z[s])

            for c in CELLS:
                #根（ルート）となるセルは、必ずモジュールが配置されているセルの中から選ばなければならない
                model.addConstr(v[c, s, m] <= x[c, s, m])

                adj = get_neighbors(c, H, W)
                out_flow = gp.quicksum(f[c, cn, s, m] for cn in adj)
                in_flow = gp.quicksum(f[cp, c, s, m] for cp in adj)

                # フロー収支
                model.addConstr(
                    out_flow - in_flow == (Rm - 1) * v[c, s, m] - (x[c, s, m] - v[c, s, m])
                )
                model.addConstr(out_flow <= (Rm - 1) * x[c, s, m])

                # (D) 総稼働時間に関する変数の線形化制約: y[c,s,m] = x[c,s,m] * d[s] のBig-M表現
                model.addConstr(y[c, s, m] <= MAX_DURATION * x[c, s, m])
                model.addConstr(y[c, s, m] <= d[s])
                model.addConstr(y[c, s, m] >= d[s] - MAX_DURATION * (1 - x[c, s, m]))

        # (E) セル排他制約
        for c in CELLS:
            model.addConstr(gp.quicksum(x[c, s, m] for m in MODULES) <= 1)

    # (F) 累積負荷制約 (各セルの寿命管理)
    for c in CELLS:
        model.addConstr(
            gp.quicksum(y[c, s, m] * MODULE_LOADS[m] for s in STATES for m in MODULES) <= MAX_LOAD_LIMIT
        )



# 6. 最適化の実行
    model.optimize()



# 7. 結果出力
    if model.Status == GRB.OPTIMAL or model.Status == GRB.TIME_LIMIT:

        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        filename = f"{timestamp}.rep"

        with open(filename, "w", encoding="utf-8") as f:
            # ヘッダー情報
            f.write("Optimization Result Report\n")
            f.write("==========================\n\n")

            #パラメータ情報
            f.write("[Model Parameters]\n")
            f.write(f"Grid Size: {H}x{W}\n")
            f.write(f"Max Load Limit: {MAX_LOAD_LIMIT}\n")
            f.write(f"Module Loads: {MODULE_LOADS}\n")
            f.write(f"Module Requirements: {MODULE_CELL_REQUIREMENTS}\n")
            f.write("\n")

            # サマリー情報
            active_states = sum(1 for s in STATES if z[s].X > 0.5)
            f.write(f"Objective Value (Total Duration): {model.ObjVal:.4f}\n")

            f.write(f"Number of Active States: {active_states}\n\n")

            # --- 追加: パフォーマンスメトリクス ---
            f.write("[Performance Metrics]\n")
            f.write(f"Execution Time (s): {model.Runtime:.4f}\n")
            f.write(f"MIP Gap: {model.MIPGap:.4%}\n")      # 最適値との乖離（0%に近いほど良い）
            f.write(f"Nodes Explored: {model.NodeCount}\n") # 探索したノード数
            f.write("\n")

            # 詳細コンフィギュレーション
            for s in STATES:
                if z[s].X < 0.5: continue

                f.write(f"--- State {s} ---\n")

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
                    f.write(row_str + "\n")
                f.write("\n")

        print(f"\nレポートファイル '{filename}' を出力しました。")

        active_states = sum(1 for s in STATES if z[s].X > 0.5)
        total_duration = model.ObjVal

        print(f"\nコンフィギュレーション数: {active_states}")
        print(f"総システム稼働時間: {total_duration:.4f}")

        for s in STATES:
            if z[s].X < 0.5: continue

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
    else:
        print("解が見つかりませんでした。")

if __name__ == "__main__":
    solve_integrated_optimization_variable_duration()
