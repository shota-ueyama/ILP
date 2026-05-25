import gurobipy as gp
from gurobipy import GRB
from datetime import datetime

#------------------------------------------------------------
# パターンB: コンフィギュレーション数固定・継続時間可変モデル
#------------------------------------------------------------

def solve_integrated_optimization():
    # 1. モデルの初期化
    model = gp.Model("Grid_Connectivity_Optimization")

    # パラメータ設定 (0: 出力抑制, 1: 標準出力)
    model.Params.OutputFlag = 1

    # 2. 定数・集合の定義
    H, W = 4, 4                      # グリッドサイズ（縦H, 横W）
    MAX_LOAD_LIMIT = 50             # 各セルの最大累積負荷
    NUM_STATES = 5                   # 状態遷移数

    # モジュール設定
    MODULE_LOADS = [10, 8, 6, 4, 2]
    MODULE_CELL_REQUIREMENTS = [1, 2, 3, 4, 5]

    NUM_CELLS = H * W
    CELLS = range(NUM_CELLS)
    MODULES = range(len(MODULE_LOADS))
    STATES = range(NUM_STATES)


    # 3. 決定変数の定義
    # x[c,s,m]: 配置フラグ (バイナリ)
    x = model.addVars(CELLS, STATES, MODULES, vtype=GRB.BINARY, name="x")

    # d[s]: 状態sの共通稼働サイクル数 (整数)
    d = model.addVars(STATES, lb=0, ub=MAX_LOAD_LIMIT, vtype=GRB.INTEGER, name="d")

    # y[c,s,m]: 線形化補助変数 (x * d)
    y = model.addVars(CELLS, STATES, MODULES, lb=0, ub=MAX_LOAD_LIMIT, vtype=GRB.INTEGER, name="y")

    # v[c,s,m]: 連結性判定のための根(Root)フラグ
    v = model.addVars(CELLS, STATES, MODULES, vtype=GRB.BINARY, name="v")

    # f[c,c_next,s,m]: 隣接セル間のフロー量
    f = model.addVars(CELLS, CELLS, STATES, MODULES, lb=0, name="flow")

    # 4. 目的関数の設定
    model.setObjective(gp.quicksum(d[s] for s in STATES), GRB.MAXIMIZE)

    # 5. 制約条件の追加

    # 隣接リスト取得関数
    def get_neighbors(c, h, w):
        neighbors = []
        row, col = divmod(c, w)
        if row > 0: neighbors.append(c - w)     # 上
        if row < h - 1: neighbors.append(c + w) # 下
        if col > 0: neighbors.append(c - 1)     # 左
        if col < w - 1: neighbors.append(c + 1) # 右
        return neighbors

    for s in STATES:
        for m in MODULES:
            Rm = MODULE_CELL_REQUIREMENTS[m]

            # (A) モジュール配置必要数制約
            model.addConstr(gp.quicksum(x[c, s, m] for c in CELLS) == Rm)

            # (B) 連結性（塊）制約: フロー保存則
            model.addConstr(gp.quicksum(v[c, s, m] for c in CELLS) == 1) # 根は1つ

            for c in CELLS:
                model.addConstr(v[c, s, m] <= x[c, s, m]) # 根は配置セルから選ぶ

                adj = get_neighbors(c, H, W)
                out_flow = gp.quicksum(f[c, cn, s, m] for cn in adj)
                in_flow = gp.quicksum(f[cp, c, s, m] for cp in adj)

                # フロー収支: 根はRm-1供給、他は1消費
                model.addConstr(
                    out_flow - in_flow == (Rm - 1) * v[c, s, m] - (x[c, s, m] - v[c, s, m])
                )
                # 配置外セルへのフロー禁止
                model.addConstr(out_flow <= (Rm - 1) * x[c, s, m])

        # (C) セル排他制約
        for c in CELLS:
            model.addConstr(gp.quicksum(x[c, s, m] for m in MODULES) <= 1)

            # (D) 線形化制約 (y = x * d)
            for m in MODULES:
                model.addConstr(y[c, s, m] <= MAX_LOAD_LIMIT * x[c, s, m])
                model.addConstr(y[c, s, m] <= d[s])
                model.addConstr(y[c, s, m] >= d[s] - MAX_LOAD_LIMIT * (1 - x[c, s, m]))

    # (E) 累積負荷制約 (各セルの寿命管理)
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
            f.write(f"Objective Value (Total Duration): {model.ObjVal:.4f}\n")
            f.write(f"Duration: {d[s].X:.4f}\n")

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


        print(f"\n合計動作サイクル数: {int(model.ObjVal)}")

        for s in STATES:
            ds_val = int(d[s].X)
            if ds_val == 0: continue

            print(f"\n--- 状態 {s} (期間: {ds_val}) ---")
            # グリッド形式で表示
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
    solve_integrated_optimization()
