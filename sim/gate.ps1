param([string]$ModelSim='E:\modelsim\win32',
      [string]$Primitives='E:\Gowin\Gowin_V1.9.12.03_x64\IDE\simlib\gw5a\prim_sim.v')
$ErrorActionPreference='Stop'
Set-Location (Split-Path $PSScriptRoot -Parent)
& (Join-Path $PSScriptRoot 'make_gate_defaults.ps1')
if(!(Test-Path 'sim\gatework')){& "$ModelSim\vlib.exe" 'sim\gatework'}
& "$ModelSim\vmap.exe" gatework 'sim\gatework'
& "$ModelSim\vlog.exe" -work gatework '+incdir+sim' $Primitives 'impl\gwsynthesis\inverted_pendulum_dual_mode_autoswing.vg' 'tb\tb_autoswing_gate.v'
if($LASTEXITCODE){throw 'gate compile failed'}
& "$ModelSim\vsim.exe" -c -l 'sim\gate_sign.log' gatework.tb_autoswing_gate -do 'run -all; quit -f'
if($LASTEXITCODE){throw 'gate simulation failed'}
$result=Get-Content -LiteralPath 'sim\gate_sign.log' -Raw
if($result -match 'FAIL|\*\* Error|\*\* Fatal' -or $result -notmatch 'PASS gate'){throw 'gate assertions failed'}
Write-Output 'PASS synthesized automatic swing and original PID'
