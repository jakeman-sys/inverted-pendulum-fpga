param([string]$Gowin='E:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe')
$ErrorActionPreference='Stop'
Set-Location $PSScriptRoot
& $Gowin build.tcl
if($LASTEXITCODE){throw 'Gowin build failed'}
$base=Join-Path $PSScriptRoot 'impl\pnr\inverted_pendulum_dual_mode_autoswing'
if(!(Test-Path -LiteralPath "$base.fs")){throw 'bitstream missing'}
$timing=Get-Content -LiteralPath "${base}_tr_content.html" -Raw
$fmax=[regex]::Match($timing,'<td>50\.000\(MHz\)</td>\s*<td[^>]*>([\d.]+)\(MHz\)')
if(!$fmax.Success -or [double]$fmax.Groups[1].Value -lt 50){throw '50MHz timing not met'}
foreach($kind in @('Setup','Hold')){if($timing -notmatch "<td>$kind</td>\s*<td>0\.000</td>\s*<td>0</td>"){throw "$kind violations"}}
# gw_sh会把GUI配置重写为默认值；构建成功后同步实际使用的三个选项。
$configPath=Join-Path $PSScriptRoot 'impl\inverted_pendulum_dual_mode_autoswing_process_config.json'
$configText=[IO.File]::ReadAllText($configPath)
$configText=[regex]::Replace($configText,'("CPU"\s*:\s*)(true|false)','${1}true')
$configText=[regex]::Replace($configText,'("TopModule"\s*:\s*)"[^"]*"','${1}"inverted_pendulum_autoswing_top"')
$configText=[regex]::Replace($configText,'("OUTPUT_BASE_NAME"\s*:\s*)"[^"]*"','${1}"inverted_pendulum_dual_mode_autoswing"')
[IO.File]::WriteAllText($configPath,$configText,[Text.UTF8Encoding]::new($false))
Write-Output "PASS build: Fmax=$($fmax.Groups[1].Value)MHz, zero setup/hold violations"
Get-FileHash -LiteralPath "$base.fs" -Algorithm SHA256
