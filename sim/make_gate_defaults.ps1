# 从实际顶层读取起摆配置，供综合网表测试使用；不修改任何RTL参数。
$ErrorActionPreference='Stop'
$taskTop=Get-Content -LiteralPath (Join-Path $PSScriptRoot '..\src\inverted_pendulum_autoswing_top.v') -Raw
function Get-TopDefault([string]$taskName){
    $taskMatch=[regex]::Match($taskTop,"\b$taskName\s*=\s*(\d+)\b")
    if(!$taskMatch.Success){throw "Top parameter missing: $taskName"}
    return [long]$taskMatch.Groups[1].Value
}
$taskDuty=Get-TopDefault 'SWING_DUTY_PERCENT'
$taskPulse=(Get-TopDefault 'SWING_PULSE_MS')*50000
$taskGap=(Get-TopDefault 'SWING_REVERSE_GAP_MS')*50000
$taskLines=@(
    '// Generated from top-level defaults by make_gate_defaults.ps1.',
    ('`define TEST_SWING_DUTY_PERCENT '+$taskDuty),
    ('`define TEST_SWING_PULSE_CYCLES '+$taskPulse),
    ('`define TEST_SWING_GAP_CYCLES '+$taskGap)
)
$taskLines | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'gate_defaults.vh') -Encoding ascii
