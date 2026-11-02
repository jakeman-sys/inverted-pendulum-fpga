param([string]$ModelSim='E:\modelsim\win32')
$ErrorActionPreference='Stop'
Set-Location $PSScriptRoot
if(!(Test-Path work)){& "$ModelSim\vlib.exe" work}
$sources=Get-ChildItem -LiteralPath '..\src' -Filter '*.v' | ForEach-Object FullName
$benches=Get-ChildItem -LiteralPath '..\tb' -Filter '*.v' | Where-Object Name -NotLike '*gate*' | ForEach-Object FullName
& "$ModelSim\vlog.exe" -work work '+incdir+../src' @sources @benches
if($LASTEXITCODE){throw 'vlog failed'}
foreach($bench in @('tb_adc_deadzone','tb_pid_feedback_guard','tb_adc_pid_handoff','tb_button_release','tb_inner_dynamic_target','tb_pid_baseline_equivalence','tb_adc_peak_swingup','tb_auto_recovery_fsm','tb_auto_recovery_top','tb_immediate_recovery','tb_swing_pwm','tb_autoswing_top','tb_limit_recovery','tb_speed_mode_limits','tb_pid_position_handoff','tb_dual_mode','tb_encoder_zero','tb_tft_three_page')){
    & "$ModelSim\vsim.exe" -c -l "$bench.log" "work.$bench" -do 'run -all; quit -f'
    if($LASTEXITCODE){throw "vsim failed: $bench"}
    $result=Get-Content -LiteralPath "$bench.log" -Raw
    if($result -match 'FAIL|\*\* Error|\*\* Fatal' -or $result -notmatch 'PASS'){throw "assertion failed: $bench"}
}
Write-Output 'PASS regression'
