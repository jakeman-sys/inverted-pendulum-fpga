# 基于高云 FPGA 的倒立摆实时姿态控制系统

本工程面向 2026 年全国大学生嵌入式芯片与系统设计竞赛 FPGA 赛道选题一，使用高云 Tang Primer 25K（GW5A-25）核心板与自研扩展板，实现倒立摆的自动起摆、自平衡、位置闭环与速度闭环控制。

控制算法全部由 Verilog RTL 实现，不依赖 MCU 或软核：角度内环 5 ms，位置/速度外环 50 ms，20 kHz PWM 驱动 TB6612；起摆采用三帧极值检测 + 峰值补能 + 三帧平滑接管；三页 TFT 显示状态与参数，5 个按键完成启停、翻页、加减与应用；带角度窗口、OTR、行程限位、故障码与自动恢复。

## 目录结构

```
PID-project-github/
├─ src/                      Verilog 源码、管脚约束（.cst）、时序约束（.sdc）
├─ tb/                       ModelSim testbench
├─ sim/                      仿真/门级测试脚本
│  ├─ regress.ps1            RTL 回归
│  ├─ gate.ps1               门级网表测试
│  ├─ make_gate_defaults.ps1 从顶层参数生成 gate_defaults.vh
│  └─ gate_defaults.vh       门级测试参数
├─ build.ps1                 一键综合/布局布线/时序检查
├─ build.tcl                 Gowin 工程构建脚本
└─ inverted_pendulum_dual_mode_autoswing.gprj
```

## 硬件平台

| 项目 | 说明 |
|---|---|
| FPGA | GW5A-LV25MG121NES（GW5A-25，Tang Primer 25K 核心板） |
| 系统时钟 | 50 MHz 有源晶振（引脚 E2） |
| 角度采集 | AD9220，12 位并行，1 MHz 采样时钟，64 点平均 |
| 电机驱动 | TB6612FNG（A 通道：PWMA/AIN1/AIN2/STBY） |
| 显示 | ST7789，2.0 英寸 240×320，SPI |
| 编码器 | 增量式 A/B 相，3.3 V，四倍频解码 |
| 按键 | 5 个（S1/S2/F1/F2/E3） |

## 编译

工具：Gowin EDA 1.9.12（GowinSynthesis + Place & Route）。

```powershell
# 默认查找 E:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe
.\build.ps1

# 或显式指定 gw_sh.exe
.\build.ps1 -Gowin "D:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe"
```

- 顶层模块：`inverted_pendulum_autoswing_top`
- 输出：`impl\pnr\inverted_pendulum_dual_mode_autoswing.fs`
- `build.tcl` 中已设置顶层、CPU 作为 GPIO、输出文件名。

## 仿真

工具：ModelSim（脚本默认 `E:\modelsim\win32`，按本机修改）。

```powershell
# RTL 回归（src + tb，排除 *gate*）
.\sim\regress.ps1 -ModelSim "D:\modelsim\win32"

# 门级测试（先完成 build.ps1，生成 impl\gwsynthesis\*.vg）
.\sim\gate.ps1 -ModelSim "D:\modelsim\win32" -Primitives "D:\Gowin\...\prim_sim.v"
```

`sim\make_gate_defaults.ps1` 会从顶层参数重新生成 `gate_defaults.vh`，修改起摆占空比/脉宽/间隔后需重新运行。

## 关键参数（顶层默认值）

| 参数 | 值 |
|---|---|
| `CENTER_CODE` / `WINDOW_CODE` | 1404 / ±350 码 |
| 角度内环 / 位置速度外环 | 5 ms（200 Hz） / 50 ms（20 Hz） |
| PWM | 20 kHz，占空比限幅 60% |
| 起摆 | 43% 占空比，88 ms/段，10 ms 换向间隔 |
| 接管判定 | 20 ms 帧节拍，±110 码内连续 3 帧 |
| 位置/速度步进 | 5.0° / 5 dps |
| 保护 | 起摆 ±120°，位置 ±720°，故障码 F1–F5，自动恢复 3 次 |

最近一次 Gowin 布局布线结果：50 MHz 约束下 Actual Fmax 50.073 MHz，setup/hold 违例 0；Logic 6844/23040（30%），Register 3399/23280（15%），DSP 12，BSRAM 0。

## 上传到 GitHub

仓库设为 **Public**。网页上传：New repository → “uploading an existing file” → 把本目录内容拖进去 → Commit。

或使用 Git：

```bash
git init
git add .
git commit -m "FPGA inverted pendulum"
git branch -M main
git remote add origin https://github.com/<用户名>/<仓库名>.git
git push -u origin main
```

## 说明

- `impl/`、`sim/work/`、`sim/gatework/` 是综合/仿真生成物，未提交；运行脚本后会自动生成。
- `.gprj.user`、`modelsim.ini` 是本机配置，未提交。
- `build.ps1`、`sim/*.ps1` 内含有本机绝对路径，换电脑后请修改或通过参数传入。
- 历史验证快照与旧日志未提交，仓库只保留当前源码、testbench 与构建/仿真脚本，避免与当前参数混淆。
- 部分 testbench（如 `tb_autoswing_top.v`、`tb_autoswing_gate.v`）中的角度常量按 `CENTER_CODE=1403` 编写，当前顶层为 1404；运行这些用例前请同步常量，或临时把顶层改回 1403。
- 本工程仅用于本次竞赛评审与学习交流，复用时请注明来源。
