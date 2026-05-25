import math

#----------------------------------
#パターン2の比較用
#配置変更なし
#--------------------------------


def solve_static_placement():
    # 1. 定数・集合の定義
    H, W = 5, 5                          # グリッドサイズ
    MAX_LOAD_LIMIT = 50                 # 最大累積負荷
    NUM_STATES = 1                       # 状態数は1（配置変更なし）

    # モジュール設定    # 添字 0:M0, 1:M1, 2:M2, 3:M3, 4:M4
    MODULE_LOADS = [10, 8, 6, 4, 2]
    MODULE_CELL_REQUIREMENTS = [1, 2, 3, 4, 5]
    NUM_MODULES = len(MODULE_LOADS)

    # 2. 静的な配置の定義 (各モジュールが占有するセル番号のリスト)
    # ※連結性を考慮したサンプル配置
    placement = {
        0: [0],                      # M0: 2セル
        1: [9,10],                         # M1: 1セル
        2: [18, 19, 20], # M2: 8セル
        3: [1, 2, 3, 4],                        # M3: 1セル
        4: [11, 12, 13, 14, 15]               # M4: 4セル
    }

    # 3. セルごとの負荷係数の算出
    # 各セルにどのモジュールが割り当てられているかを格納
    cell_to_module = {}
    for m, cells in placement.items():
        for c in cells:
            cell_to_module[c] = m

    # 4. 最大稼働サイクル数 d の計算
    # 各セルの制約: d * MODULE_LOADS[m] <= MAX_LOAD_LIMIT
    # したがって d <= MAX_LOAD_LIMIT / MODULE_LOADS[m]
    # 全体での最大 d は、全配置セルのうち最も厳しい（負荷係数が大きい）セルの制限に従う

    max_d = MAX_LOAD_LIMIT
    for m, cells in placement.items():
        if len(cells) > 0:
            limit_for_m = MAX_LOAD_LIMIT // MODULE_LOADS[m]
            if limit_for_m < max_d:
                max_d = limit_for_m

    # 5. 結果出力
    print(f"最大稼働サイクル数 (d): {max_d}")

    for i in range(H):
        row_str = ""
        for j in range(W):
            c = i * W + j
            if c in cell_to_module:
                m = cell_to_module[c]
                load = MODULE_LOADS[m]
                accumulated_load = max_d * load
                row_str += f"[M{m}(L{load:>2}):{accumulated_load:>3}] "
            else:
                row_str += "[  Empty  :  0] "


if __name__ == "__main__":
    solve_static_placement()
