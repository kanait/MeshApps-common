# MeshApps-common

[MeshApps-private](https://github.com/kanait/MeshApps-private) / [MeshApps-public](https://github.com/kanait/MeshApps-public) 向けの共通ライブラリ（ヘッダ中心）です。通常は親レポジトリの submodule として使います。

```bash
# 親側で
git submodule update --init --recursive
```

ネストした submodule として [triangle](https://github.com/kanait/triangle)（Shewchuk Triangle のフォーク）を含みます。UV scaffold（`optcuts` / `uvat`）の平面 CDT に使います。

---

## ディレクトリ構成

```
common/
├── meshL/           # 半エッジメッシュ (MeshL) と I/O (SMFLIO など)
├── render_Eigen/    # OpenGL ビューア基盤 (GLPanel, GLMeshL, シェーダ)
├── octree/          # 八分木
├── kdtree2d/        # Kd-tree 可視化ヘルパ
├── param/           # UV 共通 (MeshCut, SymDirichlet*, MeshParam, UvScaffold, …)
├── util/            # ユーティリティ (Eigen ラッパ, Triangle ラッパ, タイマーなど)
├── external/
│   ├── glad/        # OpenGL ローダ
│   ├── stb/         # 画像 I/O
│   └── triangle/    # 2D CDT (submodule → kanait/triangle)
└── data/            # サンプル OBJ / FBX / テクスチャ
```

### meshL

- `MeshL` / `VertexL` / `FaceL` / `HalfedgeL` など半エッジ構造
- `SMFLIO` … OBJ/SMF 入出力。頂点色は `v x y z r g b` を読み書き可能（書き出しは `isSaveColor`）
- `FBXLIO` … Assimp 経由の FBX（スキニング用）

### render_Eigen

Eigen 依存の軽量 OpenGL ヘルパ。`GLPanel` がカメラ・ライト・シェーダを管理します。

- ヘッドライト（eye-space、デフォルト ON）: `isHeadlight` / `toggleHeadlight`
- 詳細は [common/render_Eigen/README.md](common/render_Eigen/README.md)

### param

パラメータ化アプリ共有のアルゴリズム部品。

| ヘッダ | 内容 |
|--------|------|
| `MeshCut.hxx` | シームカット・シート展開 |
| `SymDirichletEnergy.hxx` / `SymDirichletParam.hxx` | Symmetric Dirichlet |
| `MeshParam.hxx` | パラメータ化ユーティリティ |
| `UvScaffold.hxx` | air-mesh scaffold（Triangle / lightweight） |
| `UvDistortion.hxx` | 歪み指標 |

### data

サンプルメッシュ・テクスチャ・FBX。用途の目安は親の `README.md` を参照。

---

## 依存関係

親の CMake が解決します。common 単体ではアプリをビルドしません。

| ライブラリ | 用途 |
|------------|------|
| Eigen3 | 線形代数・メッシュ座標 |
| GLFW + OpenGL + glad | ビューア |
| Assimp | FBX / LBS（親の `lbs`） |
| Triangle (submodule) | UV scaffold の CDT |

---

## 注意：解答コードの公開禁止

授業課題でこの共通コードを使う場合、解答コードや完成済み実装を public repository、Pull Request、Issue、Gist、Web サイト等に掲載しないでください。GitHub 等で作業する場合は private repository を使用してください。

---

## License

MIT License — 詳しくは [LICENSE](LICENSE) を参照。
