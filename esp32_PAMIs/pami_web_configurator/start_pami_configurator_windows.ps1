$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$BackendDir = Join-Path $ScriptDir "backend"
$FrontendDir = Join-Path $ScriptDir "frontend"

function Resolve-PythonCommand {
    if (Get-Command python -ErrorAction SilentlyContinue) {
        return @{ FilePath = "python"; PrefixArgs = @() }
    }

    if (Get-Command python3 -ErrorAction SilentlyContinue) {
        return @{ FilePath = "python3"; PrefixArgs = @() }
    }

    if (Get-Command py -ErrorAction SilentlyContinue) {
        return @{ FilePath = "py"; PrefixArgs = @("-3") }
    }

    throw "Python is required (python, python3, or py launcher)."
}

function Install-NpmIfMissing {
    if (Get-Command npm -ErrorAction SilentlyContinue) {
        return
    }

    Write-Host "npm not found. Attempting automatic installation..."

    if (Get-Command winget -ErrorAction SilentlyContinue) {
        winget install --id OpenJS.NodeJS.LTS --exact --accept-source-agreements --accept-package-agreements
    } elseif (Get-Command choco -ErrorAction SilentlyContinue) {
        choco install -y nodejs-lts
    } elseif (Get-Command scoop -ErrorAction SilentlyContinue) {
        scoop install nodejs-lts
    } else {
        throw "No supported package manager found. Install Node.js LTS manually, then rerun this script."
    }

    if (-not (Get-Command npm -ErrorAction SilentlyContinue)) {
        throw "npm installation failed. Install Node.js LTS manually, then rerun this script."
    }

    Write-Host "npm installation successful."
}

$PythonCmd = Resolve-PythonCommand
Install-NpmIfMissing

if (-not (Test-Path $BackendDir) -or -not (Test-Path $FrontendDir)) {
    throw "backend or frontend directory not found next to this script."
}

Write-Host "Starting backend on http://localhost:8000 ..."
$backendArgs = @($PythonCmd.PrefixArgs + @("-m", "uvicorn", "main:app", "--reload", "--host", "0.0.0.0", "--port", "8000"))
$BackendProc = Start-Process -FilePath $PythonCmd.FilePath -ArgumentList $backendArgs -WorkingDirectory $BackendDir -PassThru

Write-Host "Starting frontend on http://localhost:5173 ..."
$FrontendProc = Start-Process -FilePath "npm" -ArgumentList @("run", "dev") -WorkingDirectory $FrontendDir -PassThru

Write-Host "Backend PID: $($BackendProc.Id)"
Write-Host "Frontend PID: $($FrontendProc.Id)"
Write-Host "Press Ctrl+C to stop both services."

try {
    while ($true) {
        if ($BackendProc.HasExited) {
            Write-Error "Backend exited. Stopping frontend..."
        }

        if ($FrontendProc.HasExited) {
            Write-Error "Frontend exited. Stopping backend..."
        }

        Start-Sleep -Seconds 1
        $BackendProc.Refresh()
        $FrontendProc.Refresh()
    }
} finally {
    Write-Host ""
    Write-Host "Stopping services..."

    if (-not $BackendProc.HasExited) {
        Stop-Process -Id $BackendProc.Id -Force
    }

    if (-not $FrontendProc.HasExited) {
        Stop-Process -Id $FrontendProc.Id -Force
    }
}
