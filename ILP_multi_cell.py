import gurobipy as gp
from gurobipy import GRB

#タイミング最適化　（配置バラバラ）

def solve_with_gurobi():
    # 1. モデルの初期化
    model = gp.Model("MultiCell_Optimization_Gurobi")

    # パラメータ設定 (出力を抑制する場合は0)
    model.Params.OutputFlag = 1

    # 2. 定数・集合の定義
    MAX_LOAD = 100
    MODULE_LOADS = [3, 4, 5, 6, 7]
    NUM_STATES = 5
    NUM_CELLS = 16
    MODULE_CELL_REQUIREMENTS = [2, 1, 8, 1, 4]   #各モジュールの必要なセル数

    CELLS = range(NUM_CELLS)
    MODULES = range(len(MODULE_LOADS))
    STATES = range(NUM_STATES)

    # 3. 決定変数の定義
    # x[c, s, m]: バイナリ変数
    x = model.addVars(CELLS, STATES, MODULES, vtype=GRB.BINARY, name="x")
    # d[s]: 整数変数
    d = model.addVars(STATES, lb=0, ub=MAX_LOAD, vtype=GRB.INTEGER, name="d")
    # y[c, s, m]: 線形化用変数 (x * d)
    y = model.addVars(CELLS, STATES, MODULES, lb=0, ub=MAX_LOAD, vtype=GRB.INTEGER, name="y")


    # 4. 目的関数の設定
    model.setObjective(gp.quicksum(d[s] for s in STATES), GRB.MAXIMIZE)

    # 5. 制約条件の追加
    for s in STATES:
        # 各モジュールは必ず必要な数のセルを使って配置
        for m in MODULES:
            model.addConstr(
                gp.quicksum(x[c, s, m] for c in CELLS) == MODULE_CELL_REQUIREMENTS[m]
            )

        # 各セルは最大1つのモジュールを保持
        for c in CELLS:
            model.addConstr(gp.quicksum(x[c, s, m] for m in MODULES) <= 1)

            # 線形化制約 (y[c,s,m] = x[c,s,m] * d[s])
            for m in MODULES:
                model.addConstr(y[c, s, m] <= MAX_LOAD * x[c, s, m])
                model.addConstr(y[c, s, m] <= d[s])
                model.addConstr(y[c, s, m] >= d[s] - MAX_LOAD * (1 - x[c, s, m]))

    for c in CELLS:
        # 累積負荷制約
        model.addConstr(
            gp.quicksum(y[c, s, m] * MODULE_LOADS[m] for s in STATES for m in MODULES) <= MAX_LOAD
        )

    """
    # 初期配置 (s=0)
    for m in MODULES:
        for k in range(MODULE_CELL_REQUIREMENTS[m]):
            model.addConstr(x[m + k, 0, m] == 1)
   """

    # 6. 最適化の実行
    model.optimize()

    # 7. 結果出力
    if model.Status == GRB.OPTIMAL:
        print(f"\nステータス: {model.Status}")
        print(f"合計動作サイクル数: {int(model.ObjVal)}\n")

        for s in STATES:
            ds_val = int(d[s].X)
            print(f"--- 状態{s} (d[{s}] = {ds_val}) ---")
            for c in CELLS:
                assigned = False
                for m in MODULES:
                    if x[c, s, m].X > 0.5: # バイナリ変数の判定
                        print(f"  セル {c}: 負荷{MODULE_LOADS[m]}")
                        assigned = True
                if not assigned:
                    print(f"  セル {c}: ")
    else:
        print("最適解が見つかりませんでした。")

if __name__ == "__main__":
    solve_with_gurobi()
