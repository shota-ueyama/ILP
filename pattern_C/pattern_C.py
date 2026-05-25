import math
import gurobipy as gp
from gurobipy import GRB
from datetime import datetime

#-------------------------------------------------------------
# パターンC: コンフィギュレーション数可変・継続時間固定モデル
#-------------------------------------------------------------

def solve_integrated_optimization_fixed_duration():
    # 1. モデルの初期化
    model = gp.Model("Grid_Configuration_Count_Optimization")

    # パラメータ設定 (0: 出力抑制, 1: 標準出力)
    model.Params.OutputFlag = 1

    # 2. 定数・集合の定義
    H, W = 5, 5                       # グリッドサイズ
    MAX_LOAD_LIMIT = 50              # 各セルの最大累積負荷
    UNIT_DURATION = 1.0                 # 各コンフィギュレーションの固定継続時間

    # モジュール設定
    MODULE_LOADS = [8, 7, 6, 5, 4]
    MODULE_CELL_REQUIREMENTS = [1, 2, 3, 4, 5]

    # 探索対象となる最大コンフィギュレーション数（上限値）
    # 理論上の最大値や計算コストを考慮して設定
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

    # v[c,s,m]: 連結性判定のための根(Root)フラグ
    v = model.addVars(CELLS, STATES, MODULES, vtype=GRB.BINARY, name="v")

    # f[c,cn,s,m]: 隣接セル間のフロー量
    f = model.addVars(CELLS, CELLS, STATES, MODULES, lb=0, name="flow")

    # 4. 目的関数の設定
    # 使用されるコンフィギュレーションの総数を最大化
    model.setObjective(gp.quicksum(z[s] for s in STATES), GRB.MAXIMIZE)

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
        # コンフィギュレーションの順序性を保証（s番目が不使用ならs+1番目も不使用）
        if s > 0:
            model.addConstr(z[s] <= z[s-1], name=f"order_state_{s}")

        for m in MODULES:
            Rm = MODULE_CELL_REQUIREMENTS[m]

            # (A) モジュール配置必要数制約
            # 状態sが有効 (z[s]=1) の場合のみ配置を行う
            model.addConstr(gp.quicksum(x[c, s, m] for c in CELLS) == Rm * z[s])

            # (B) 連結性制約
            model.addConstr(gp.quicksum(v[c, s, m] for c in CELLS) == z[s])

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

        # (C) セル排他制約
        for c in CELLS:
            model.addConstr(gp.quicksum(x[c, s, m] for m in MODULES) <= 1)

    # (D) 累積負荷制約 (各セルの寿命管理)
    # y = x * d の代わりに、dが固定値 UNIT_DURATION となる
    for c in CELLS:
        model.addConstr(
            gp.quicksum(x[c, s, m] * UNIT_DURATION * MODULE_LOADS[m]
                        for s in STATES for m in MODULES) <= MAX_LOAD_LIMIT
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
            f.write(f"d[s]:{UNIT_DURATION}\n")

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



        active_states = int(model.ObjVal)
        print(f"\nコンフィギュレーション数: {active_states}")
        print(f"合計動作サイクル数: {active_states * UNIT_DURATION}")

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
    solve_integrated_optimization_fixed_duration()
