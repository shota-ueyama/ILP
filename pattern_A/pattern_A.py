import gurobipy as gp
from gurobipy import GRB
from datetime import datetime

#------------------------------------------------------------
# パターンA: コンフィギュレーション数固定・継続時間固定モデル
#------------------------------------------------------------

def solve_integrated_optimization_fixed_both():
    # 1. モデルの初期化
    model = gp.Model("Grid_Fixed_Configuration_Fixed_Duration")

    # パラメータ設定 (0: 出力抑制, 1: 標準出力)
    model.Params.OutputFlag = 1

    # 2. 定数・集合の定義
    H, W = 5, 5                      # グリッドサイズ（縦H, 横W）
    MAX_LOAD_LIMIT = 50              # 各セルの最大累積負荷
    NUM_STATES = 5                   # 状態遷移数 (固定)
    UNIT_DURATION = 1.0              # 各コンフィギュレーションの継続時間 (固定)

    # モジュール設定
    MODULE_LOADS = [8, 7, 6, 5, 4]
    MODULE_CELL_REQUIREMENTS = [1, 2, 3, 4, 5]

    NUM_CELLS = H * W
    CELLS = range(NUM_CELLS)
    MODULES = range(len(MODULE_LOADS))
    STATES = range(NUM_STATES)

    # 3. 決定変数の定義
    # x[c,s,m]: 配置フラグ (バイナリ)
    x = model.addVars(CELLS, STATES, MODULES, vtype=GRB.BINARY, name="x")

    # v[c,s,m]: 連結性判定のための根(Root)フラグ
    v = model.addVars(CELLS, STATES, MODULES, vtype=GRB.BINARY, name="v")

    # f[c,cn,s,m]: 隣接セル間のフロー量
    f = model.addVars(CELLS, CELLS, STATES, MODULES, lb=0, name="flow")

    # 4. 目的関数の設定
    # 本モデルは制約充足問題（実行可能解の探索）となるため、定数を設定する
    model.setObjective(0.0, GRB.MAXIMIZE)

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
            # 各状態において、所定のセル数だけモジュールを配置する
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
        # 1つのセルには同時に最大1つのモジュールしか配置できない
        for c in CELLS:
            model.addConstr(gp.quicksum(x[c, s, m] for m in MODULES) <= 1)

    # (D) 累積負荷制約 (各セルの寿命管理)
    # コンフィギュレーション数および継続時間が固定のため、線形化制約は不要となる
    for c in CELLS:
        model.addConstr(
            gp.quicksum(x[c, s, m] * UNIT_DURATION * MODULE_LOADS[m] for s in STATES for m in MODULES) <= MAX_LOAD_LIMIT
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

            # パラメータ情報
            f.write("[Model Parameters]\n")
            f.write(f"Grid Size: {H}x{W}\n")
            f.write(f"Max Load Limit: {MAX_LOAD_LIMIT}\n")
            f.write(f"Module Loads: {MODULE_LOADS}\n")
            f.write(f"Module Requirements: {MODULE_CELL_REQUIREMENTS}\n")
            f.write(f"Fixed States: {NUM_STATES}\n")
            f.write(f"Fixed Unit Duration: {UNIT_DURATION}\n")
            f.write("\n")

            # サマリー情報
            f.write("Objective Value: Feasible Solution Found\n\n")

            # パフォーマンスメトリクス
            f.write("[Performance Metrics]\n")
            f.write(f"Execution Time (s): {model.Runtime:.4f}\n")
            if hasattr(model, "MIPGap"):
                f.write(f"MIP Gap: {model.MIPGap:.4%}\n")
            f.write(f"Nodes Explored: {model.NodeCount}\n")
            f.write("\n")

            # 詳細コンフィギュレーション
            for s in STATES:
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
        print(f"\n実行可能解が見つかりました（コンフィギュレーション数: {NUM_STATES}, 各動作時間: {UNIT_DURATION}）。")

        for s in STATES:
            print(f"\n--- 状態 {s} ---")
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
                print(row_str)
    else:
        print("解が見つかりませんでした（実行不能または制限時間超過）。")

if __name__ == "__main__":
    solve_integrated_optimization_fixed_both()
