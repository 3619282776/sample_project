# 一键烧录：先清理占用 COM 端口的残留 monitor/esptool 进程，再执行 idf.py flash
# 用法（在项目根目录）：  .\flash.ps1             # 默认 COM8
#                         .\flash.ps1 -Port COM9  # 指定端口
param(
    [string]$Port = "COM8"
)

$ErrorActionPreference = "Stop"
$IdfPath = "C:\esp\v6.1\esp-idf"

# 1) 清理占用该端口的残留进程（VS Code ESP-IDF 插件关闭终端时遗留的 monitor/esptool）
$leftover = Get-CimInstance Win32_Process -Filter "Name = 'python.exe'" |
    Where-Object { $_.CommandLine -match "$Port|esptool|idf_monitor|esp_idf_monitor" }

if ($leftover) {
    Write-Host "清理 $($leftover.Count) 个占用 $Port 的残留进程..." -ForegroundColor Yellow
    $leftover | ForEach-Object {
        Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
    }
    Start-Sleep -Milliseconds 500
} else {
    Write-Host "$Port 未被占用" -ForegroundColor Green
}

# 2) 加载 ESP-IDF 环境
Write-Host "加载 ESP-IDF 环境..." -ForegroundColor Cyan
. (Join-Path $IdfPath "export.ps1") | Out-Null

# 3) 烧录
Write-Host "烧录到 $Port ..." -ForegroundColor Cyan
idf.py -p $Port flash
