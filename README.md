# FastDynamicMesh (FDM)

基于 ANSYS Fluent UDF 的快速动网格工具：用模态叠加（Wilson-θ）驱动网格变形，并从流体侧积分模态力，实现弹性结构/流固耦合计算。

当前仓库以**分支区分算例**：代码基本一致，`mode/` 下的 APDL 和模型输入随分支不同。

---

## 1. 目录结构

```
.
├─ CMakeLists.txt            # 顶层构建配置（Fluent 路径/版本、算例类型）
├─ CMakePresets.json         # 预设的 MinGW 构建配置
├─ import/                   # CMake 模块（编译器选项、UDF 函数表生成）
├─ src/                      # UDF 源码
│  ├─ FastDynamicMesh.c
│  ├─ fdmUtils.h
│  └─ CMakeLists.txt
├─ mode/                     # 模态计算 APDL + 算例输入（大部分文件不入库）
│  ├─ ModeCalculation.apdl   # 模态求解 + 导出节点/模态数据
│  ├─ ModeOutput.apdl        # 从已有结果重新导出节点/模态数据
│  └─ beam_and_fluid.in / system.in / Roter37Mesh.in（分属不同分支）
├─ ModeCalculation.bat       # 调用 MAPDL 的批处理
├─ libudf/                   # UDF 编译输出（生成，忽略）
├─ build/                    # CMake 构建目录（生成，忽略）
└─ .vscode/                  # launch.json / tasks.json（项目级配置）
```

---

## 2. 环境要求

| 组件 | 版本 / 说明 |
|---|---|
| 操作系统 | Windows 10/11 x64 |
| ANSYS Fluent | **2022 R1 (v221)**，需包含 UDF SDK（`fluent22.1.0/src/...` 头文件、`ntbin/win64/resolve.exe`、`fl*.lib`） |
| ANSYS Mechanical APDL | 2022 R1，仅模态计算需要（`ModeCalculation.bat`） |
| C/C++ 编译器 | **MinGW-w64 gcc/g++ 8.1.0**，参考路径 `C:/Program Files/mingw64` |
| CMake | **≥ 3.21**（参考环境 3.28），生成器 `MinGW Makefiles` |
| Git | 任意较新版本 |
| 许可证 | 运行 Fluent / MAPDL 需要有效的 ANSYS 许可证 |

> 注意：
> - ANSYS 的 SDK 头文件和库**不能随仓库分发**，必须在本机安装完整 Fluent。
> - 只有 Fluent 运行库（`win64/`）而没有 `src/` 头文件的“精简安装”**无法编译 UDF**，会报 `fatal error: udf.h: No such file or directory`。
> - 如果 `PATH` 里存在多个 gcc（例如 StrawberryPerl 自带的），请使用绝对路径或 CMake preset 中指定的 MinGW 路径，否则会链接到错误的工具链。

---

## 3. 获取代码与选择分支

默认分支 `master` **不包含 `mode/` 目录**，`git clone` 后请先切换到 `Alpha`（或目标算例分支）：

```powershell
git clone <repo-url>
cd 202311_FastDynamicMesh
git branch -a
git switch Alpha
```

分支与算例对应关系：

| 分支 | 版本 | 算例 | `mode/` 模型输入 | 说明 |
|---|---|---|---|---|
| `master` | 1.3.2 | — | 无 | 默认分支，已删除 `mode/`，不要直接用于计算 |
| `Alpha` | 1.3.2 | 涡激弹性梁（flow around beam） | `beam_and_fluid.in` | 各算例分支的基线 |
| `DamFailure` | 1.3.2 | 溃坝（dam failure） | `system.in` | APDL 的组件名/材料号与梁算例不同 |
| `Rotor37` | 1.3.2 | Rotor 37 压气机 | `Roter37Mesh.in` | 使用另一套网格 |
| `Semi-Implicit` | 1.4.1 | 弹性梁 + 半隐式迭代 | `beam_and_fluid.in` | 在 Alpha 基础上增加半隐式迭代 |

**重要**：`mode/*.csv` 不在版本控制中，切换分支不会切换算例数据。使用哪个算例，就要把该算例的 CSV 数据放进 `mode/`，并使用**同一分支**的 APDL（不同算例的组件名、约束面、材料号、模态数都不同，APDL 不能混用）。

---

## 4. 构建

### 4.1 使用预设（推荐）

```powershell
cmake --preset mingw-debug
cmake --build --preset mingw-debug
```

产物：

```
libudf/win64/3ddp_host/libudf.dll
libudf/win64/3ddp_node/libudf.dll
```

Release 构建：`cmake --preset mingw-release` + `cmake --build --preset mingw-release`。

### 4.2 手动配置

```powershell
cmake -S . -B build -G "MinGW Makefiles" `
      -DCMAKE_BUILD_TYPE=Debug `
      -DCMAKE_C_COMPILER="C:/Program Files/mingw64/bin/gcc.exe" `
      -DCMAKE_CXX_COMPILER="C:/Program Files/mingw64/bin/g++.exe" `
      -DFLUENT_ROOT="C:/Program Files/ANSYS Inc/v221/fluent" `
      -DRELEASE=22.1.0 -DLIB_RELEASE=2210
cmake --build build -j
```

### 4.3 可配置变量

| 变量 | 默认值 | 说明 |
|---|---|---|
| `FLUENT_ROOT` | `C:/Program Files/ANSYS Inc/v221/fluent` | Fluent 安装根目录；也可用同名环境变量 |
| `FLUENT_ARCH` | `win64` | Fluent 平台目录 |
| `RELEASE` | `22.1.0` | Fluent 版本目录名（`fluent${RELEASE}`） |
| `LIB_RELEASE` | `2210` | 链接库版本号（`fl${LIB_RELEASE}`） |
| `DIMENSION` | `3d` | `2d` / `3d`，必须与 case 一致 |
| `PRECISE` | `dp` | `dp`（双精度）或留空（单精度） |
| `PARALLEL_NODE` | `msmpi` | `intel` / `msmpi` / `none` / `net` |

换 Fluent 版本时至少要同时修改：`FLUENT_ROOT`、`RELEASE`、`LIB_RELEASE`（版本号去点，如 24.1.0 → 2410），并重新生成 UDF。

> `CMakePresets.json` 中的编译器路径是参考环境的绝对路径。如果 MinGW 装在别处，请修改 preset，或在命令行用 `-DCMAKE_C_COMPILER=...` 覆盖。

### 4.4 自动生成的文件

以下文件现在生成在**构建目录**，不再写回源码树，也不需要提交：

```
build/src/generated/udf_names.c   # 由 CMake 扫描 DEFINE_* 宏生成
build/src/generated/ud_io1.h      # 由 Fluent 的 resolve.exe 生成
```

如果配置时提示 `resolve.exe 执行失败`，说明 `FLUENT_ROOT/ntbin/win64/resolve.exe` 不存在，请检查 Fluent 安装路径。

---

## 5. 运行流程

1. **模态计算（MAPDL）**
   - 确认 `mode/` 中已放入当前算例的模型输入（见分支表）。
   - 设置 `MAPDL_EXE` 环境变量（或直接修改 `ModeCalculation.bat` 中的默认路径）。
   - 运行 `ModeCalculation.bat`，生成：
     ```
     mode/FluidNodeCoor.csv
     mode/FluidNodeDisp1..N.csv
     mode/StructNodeCoor.csv
     mode/StructNodeDisp1..N.csv
     ```

2. **准备 UDF 输入**
   - `mode/FluidPara.csv`：`idFSI`、`idFluid`、各阶模态初速度。该文件**不随仓库分发**，需按算例从归档中复制或手工创建。
   - 结构数据可选：缺少 `StructNodeCoor.csv` 时 `nModeStruct = 0`，结构力反馈自动关闭，程序仍可计算。

3. **编译 UDF**（见第 4 节），确保已生成 `libudf/win64/3ddp_host` 和 `3ddp_node`。

4. **Fluent 中加载 UDF**
   - 读取 case/data（需与 `DIMENSION`/`PRECISE`/`PARALLEL_NODE` 匹配）。
   - 读取 data 后 `AutoPreprocess` 会自动调用 `Preprocess`；如果先读 case 后读 data，也可手动执行 On Demand → `Preprocess`。
   - 计算过程中由 `FDM_method`（Grid Motion）驱动网格，`Set_next_time_step` 在每步结束时更新模态状态。
   - 结束时会写 `Result.csv`；`FDMOutput.txt` 是 Fluent 控制台日志，属于运行产物。

5. **可用输出**
   - `Result.csv`：时间、模态力、模态位移
   - `FluidOutputNodeHost.csv`：排序后的流体节点坐标
   - `debug/`：仅在定义 `DEBUG_FDM` 时产生

---

## 6. 版本控制约定

### 6.1 不提交的文件

`.gitignore` 已忽略：

- `build/`、`libudf/`、`debug/`、`exe/`
- `*.csv`（所有算例的模态/节点数据、`FluidPara.csv`）
- `*.msh`、`*.h5`、`*.trn`、`*.log`、`*.zip`、`*.jou`
- `FDMOutput.txt`（Fluent 日志）
- `src/udf_names.c`、`src/ud_io1.h`（生成物）
- 个人 `.vscode/settings.json`、`c_cpp_properties.json`、snippets

算例数据（CSV、case、mesh）请和 `杂项/` 归档一起保存，按算例分支取用。

### 6.2 项目级 VSCode 配置

已纳入版本控制：

- `.vscode/launch.json`：gdb attach 到 `libudf.dll` 的调试配置
- `.vscode/tasks.json`：单文件 g++ 编译任务

个人设置（`settings.json` 等）仍保持忽略。

### 6.3 大文件与 Git LFS（建议，尚未执行）

`mode/` 下的 `.in` 网格文件较大且已进入历史：

| 文件 | 大小 |
|---|---|
| `mode/system.in` | 49.8 MB |
| `mode/beam_and_fluid.in` | 23.0 MB |
| `mode/Roter37Mesh.in`（Rotor37 分支） | 35.7 MB |

后续如果要进一步瘦身，可迁移到 Git LFS：

```powershell
git lfs install
git lfs track "mode/*.in"
git add .gitattributes
git lfs migrate import --include="mode/*.in" --everything
```

> `git lfs migrate` 会**重写历史**（所有 commit hash 改变），需要所有协作者重新 clone，并强制推送。执行前务必备份，并确认远程仓库支持 LFS。当前仓库未执行该迁移。

---

## 7. 常见问题

**Q：编译报 `fatal error: udf.h: No such file or directory`**
A：Fluent 安装不完整，缺少 UDF SDK 头文件。确认 `FLUENT_ROOT` 指向包含 `fluent22.1.0/src/main/udf.h` 的目录。

**Q：链接时提示找不到 `fl2210` / `mport`**
A：`RELEASE` 与 `LIB_RELEASE` 和实际安装不匹配，或 `FLUENT_ARCH` 选错（应为 `win64`）。

**Q：`resolve.exe 执行失败` 警告**
A：`FLUENT_ROOT/ntbin/win64/resolve.exe` 不存在。该文件用于生成 `ud_io1.h`；不会阻止编译，但说明 Fluent 安装不完整。

**Q：配置用了错误的 gcc**
A：`PATH` 中可能先命中了 StrawberryPerl 的 gcc。使用 `CMakePresets.json` 中的绝对路径，或 `-DCMAKE_C_COMPILER=...` 指定。

**Q：换了算例，计算结果明显不对**
A：检查是否同时切换了 `mode/` 数据、APDL 和分支。不同算例的组件名/材料号/模态数不同，混用会读错模型或读不到数据。

**Q：结构 CSV 缺失**
A：程序允许缺失，此时 `nModeStruct = 0`，结构力反馈关闭；如需结构耦合，请补 `StructNodeCoor.csv`、`StructNodeDisp*.csv` 及 `StructStiff/StructDamp/StructLoad.csv`。
