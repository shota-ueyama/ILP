# このツールの使い方
 
[Gurobi Optimizer](https://www.gurobi.com/)
を使用

```linux64```ディレクトリにおいて

```text
$ export GRB_LICENSE_FILE=$PWD/gurobi.lic
```
を実行後、pythonで実行可能

FPGAと回路モジュールに関する情報を入力することで、各状態におけるコンフィグレーションと合計稼働サイクル数を出力
レポートファイルとして出力


| コンフィグレーション数<br>（配置変更の回数） | 動作させる時間：固定 | 動作させる時間：可変 |
| :---: | :---: | :---: |
| **固定** | [パターンA](https://github.com/shota-ueyama/ILP/tree/master/pattern_A) | [パターンB](https://github.com/shota-ueyama/ILP/tree/master/pattern_B) |
| **可変** | [パターンC](https://github.com/shota-ueyama/ILP/tree/master/pattern_C) | [パターンD](https://github.com/shota-ueyama/ILP/tree/master/pattern_D) |
