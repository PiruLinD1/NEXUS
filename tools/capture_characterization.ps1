param(
    [string]$Port,
    [string]$LogPath,
    [string]$ProsPath
)
$ErrorActionPreference = 'Stop'

# PROS terminal decodes the firmware's COBS framing. This script does not upload
# firmware, send motion commands or run autonomous. R1 on the controller starts
# and supervises each separate trial.
if (!$ProsPath) {
    $installed = Get-Command pros -ErrorAction SilentlyContinue
    if ($installed) { $ProsPath = $installed.Source }
    else {
        $ProsPath = Join-Path $env:APPDATA 'Code/User/globalStorage/sigbots.pros/install/pros-cli-windows/pros.exe'
    }
}
if (!(Test-Path -LiteralPath $ProsPath -PathType Leaf)) {
    throw 'PROS CLI non trovata: specifica -ProsPath con il percorso di pros.exe.'
}
if (!$LogPath) {
    $LogPath = Join-Path $PSScriptRoot ('../docs/characterization-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '.log')
}
$LogPath = [System.IO.Path]::GetFullPath($LogPath)
if (Test-Path -LiteralPath $LogPath) { throw "Il log esiste gia: $LogPath" }
$parentDirectory = Split-Path -Parent $LogPath
if (!(Test-Path -LiteralPath $parentDirectory -PathType Container)) {
    throw "La cartella di destinazione non esiste: $parentDirectory"
}
$terminalArguments = @('terminal', '--backend', 'solo', '--no-banner', '--output', $LogPath)
if ($Port) { $terminalArguments += $Port }
Write-Host "Registrazione USB: $LogPath"
Write-Host 'Sul controller: L1+R1 per 1 s, rilascia, DOWN. LEFT/RIGHT seleziona la prova.'
Write-Host 'Tieni R1 fino a Prova conclusa. Rilascia R1 o premi B per interrompere.'
Write-Host 'Riposiziona il robot fermo tra le prove. Ctrl+C termina solo la registrazione sul PC.'
try {
    & $ProsPath @terminalArguments
    if ($LASTEXITCODE -ne 0) { Write-Warning "Terminale PROS concluso con codice $LASTEXITCODE; il log resta disponibile." }
} finally {
    Write-Host "Log conservato: $LogPath"
    Write-Host 'Analisi: python tools/analyze_characterization.py PERCORSO_LOG --output docs/characterization-results'
    Write-Host 'Aggiungi --mass-kg PESO per stimare anche la forza resistente effettiva.'
}
